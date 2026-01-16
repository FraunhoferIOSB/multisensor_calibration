#include "multisensor_calibration/visualizers/PointCloud2PointCloudDistance.h"

// Std
#include <chrono>
#include <cmath>
#include <iostream>

// ROS
#include <multisensor_calibration/common/utils.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// PCL
#include <pcl/PolygonMesh.h>
#include <pcl/common/centroid.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/gp3.h>
#include <pcl_ros/transforms.hpp>

// OpenCV
#include <opencv2/core.hpp>

#define UNUSED(expr) (void)(expr);

//--- TOPIC NAMES
static const std::string INPUT_TOPIC_NAME_PREFIX      = "cloud";
static const std::string OUTPUT_CLOUD_TOPIC_SUFFIX    = "enhanced";
static const std::string INPUT_CALIBRATION_TOPIC_NAME = "calibration";

//--- DEFAULT VALUES
static int DEFAULT_NUMBER_CLOUDS    = 2;
static int DEFAULT_PROCESSING_RATE  = 1;
static int DEFAULT_DISTANCE_MEASURE = static_cast<int>(
  multisensor_calibration::visualizers::PointCloud2PointCloudDistanceNode::POINT_2_POINT);
static int DEFAULT_NN               = 5;
static float DEFAULT_MAX_DISTANCE   = 5.f;
static float DEFAULT_CLAMP_DISTANCE = FLT_MAX;

static std::vector<std::string> distanceMeasureStrList = {"POINT_2_POINT",
                                                          "POINT_2_SURFACE"};

//==================================================================================================
multisensor_calibration::visualizers::PointCloud2PointCloudDistanceNode::PointCloud2PointCloudDistanceNode(rclcpp::NodeOptions options, std::string nodeName) :
  rclcpp::Node(nodeName, options),
  useTemporaryTransform_(false)
{
    std::setlocale(LC_ALL, "en_US.UTF-8");
    tfBuffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    onInit();
}

//==================================================================================================
multisensor_calibration::visualizers::PointCloud2PointCloudDistanceNode::~PointCloud2PointCloudDistanceNode()
{
}

//==================================================================================================
void multisensor_calibration::visualizers::PointCloud2PointCloudDistanceNode::onInit()
{
    //--- read launch parameters
    int distanceMeasureInt, processingRate;
    std::vector<double> tmpTransformCoeffs;
    nClouds_                = this->declare_parameter<int>("number_of_clouds", DEFAULT_NUMBER_CLOUDS);
    processingRate          = this->declare_parameter<int>("processing_rate", DEFAULT_PROCESSING_RATE);
    distanceMeasureInt      = this->declare_parameter<int>("distance_measure", DEFAULT_DISTANCE_MEASURE);
    nNearestNeighbors_      = this->declare_parameter<int>("num_nearest_neighbors", DEFAULT_NN);
    maxDistance_            = this->declare_parameter<float>("max_distance", DEFAULT_MAX_DISTANCE);
    clampDistanceThreshold_ = this->declare_parameter<float>("clamp_distance_threshold", DEFAULT_CLAMP_DISTANCE);
    tmpTransformCoeffs =
      this->declare_parameter<std::vector<double>>("temp_transform", std::vector<double>{});
    if (!tmpTransformCoeffs.empty())
    {
        useTemporaryTransform_ = true;
    }

    //--- sanity check for params
    if (nClouds_ < 2)
    {
        RCLCPP_WARN(this->get_logger(), "(number_of_clouds < 2). Setting number clouds to default: %i ",
                    DEFAULT_NUMBER_CLOUDS);
        nClouds_ = DEFAULT_NUMBER_CLOUDS;
    }
    if (processingRate <= 0)
    {
        RCLCPP_WARN(this->get_logger(), "(processing_rate <= 0). Setting processing rate to default: %i Hz",
                    DEFAULT_PROCESSING_RATE);
        processingRate = DEFAULT_PROCESSING_RATE;
    }
    if (distanceMeasureInt < 0 || distanceMeasureInt > 1)
    {
        RCLCPP_WARN(this->get_logger(), "(distance_measure < 0 || distance_measure > 1). "
                                        "Setting number distance measure to default: %s ",
                    distanceMeasureStrList[DEFAULT_DISTANCE_MEASURE].c_str());
        distanceMeasureInt = DEFAULT_DISTANCE_MEASURE;
    }
    if (nNearestNeighbors_ <= 0)
    {
        RCLCPP_WARN(this->get_logger(), "(num_nearest_neighbors <= 0). Setting number nearest neighbors to default: %i ",
                    DEFAULT_NN);
        nNearestNeighbors_ = DEFAULT_NN;
    }
    if (maxDistance_ <= 0)
    {
        RCLCPP_WARN(this->get_logger(), "(max_distance <= 0). Setting max. distance to default: %f ",
                    DEFAULT_MAX_DISTANCE);
        maxDistance_ = DEFAULT_MAX_DISTANCE;
    }
    if (clampDistanceThreshold_ < maxDistance_)
    {
        RCLCPP_WARN(this->get_logger(), "(clamp_distance_threshold < max_distance). Setting clamp. distance threshold to "
                                        "max. distance: %f ",
                    maxDistance_);
        clampDistanceThreshold_ = maxDistance_;
    }

    //--- get temporary transform from tmpTransformStr
    if (useTemporaryTransform_)
    {
        //--- check if temp_transform is of correct size
        if (tmpTransformCoeffs.size() == 7)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Using temporary transform (XYZ | RPY) "
                        "between cloud_1 (child) and cloud_0 (parent): %f %f %f | "
                        "%f %f %f %f",
                        tmpTransformCoeffs[0], tmpTransformCoeffs[1],
                        tmpTransformCoeffs[2], tmpTransformCoeffs[3],
                        tmpTransformCoeffs[4], tmpTransformCoeffs[5],
                        tmpTransformCoeffs[6]);

            temporaryTransform_.setOrigin(tf2::Vector3(tmpTransformCoeffs[0],
                                                       tmpTransformCoeffs[1],
                                                       tmpTransformCoeffs[2]));
            temporaryTransform_.setRotation(
              tf2::Quaternion(tmpTransformCoeffs[3], tmpTransformCoeffs[4],
                              tmpTransformCoeffs[5], tmpTransformCoeffs[6]));
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "Wrong format of temp_transform. Please provide as \"X Y Z "
                        "R P Y\". "
                        "Extracting transform from frame IDs instead.");
            useTemporaryTransform_ = false;
        }
    }

    //--- initialize subscriber and register callback
    cloudSubscrs_ = std::vector<PointCloudSubscription::SharedPtr>(nClouds_);
    for (int i = 0; i < nClouds_; ++i)
    {
        std::string topicName = INPUT_TOPIC_NAME_PREFIX + "_" + std::to_string(i);

        cloudSubscrs_[i] = this->create_subscription<PointCloud>(topicName, 10,
                                                                 [this, i](const PointCloud::ConstSharedPtr ipCloudMsg)
                                                                 {
                                                                     // Call your existing callback function with additional argument
                                                                     cloudMessageCallback(ipCloudMsg, i);
                                                                 });
    }

    pCalibSub_ = create_subscription<multisensor_calibration_interface::msg::CalibrationResult>(
      INPUT_CALIBRATION_TOPIC_NAME,
      1,
      std::bind(
        &multisensor_calibration::visualizers::PointCloud2PointCloudDistanceNode::onCalibrationReceived,
        this,
        std::placeholders::_1));

    //--- initialize publishers
    pubs_ = std::vector<PointCloudPublisher::SharedPtr>(nClouds_);
    for (int i = 0; i < nClouds_; ++i)
    {
        std::string topicName = INPUT_TOPIC_NAME_PREFIX + "_" + std::to_string(i) + "_" +
                                OUTPUT_CLOUD_TOPIC_SUFFIX;

        pubs_[i] = this->create_publisher<PointCloud>(topicName, 10);
    }

    //--- initialize list of point clouds and the metadata
    pPointClouds_ = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>(nClouds_, nullptr);
    // pKdTrees_           = std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr>(nClouds_);
    // pCloudNormals_      = std::vector<pcl::PointCloud<pcl::Normal>::Ptr>(nClouds_);
    pointCloudMsgHeaders_ = std::vector<std_msgs::msg::Header>(nClouds_);

    //--- initialize processing timer (no autostart)
    processingTimer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / processingRate), std::bind(&PointCloud2PointCloudDistanceNode::processCloudsCallback, this));
    processingTimer_->cancel();
    isTimerActive_ = false;

    //--- select appropriate function to calculate the distance measure
    distanceMeasure_ = static_cast<EDistanceMeasure>(distanceMeasureInt);
    switch (distanceMeasure_)
    {
    case POINT_2_POINT:
        calculateDistanceFn_ = &calculatePoint2PointDistance;
        break;

    default:
    case POINT_2_SURFACE:
        calculateDistanceFn_ = &calculatePoint2SurfaceDistance;
        break;
    }
}

//==================================================================================================
void multisensor_calibration::visualizers::PointCloud2PointCloudDistanceNode::cloudMessageCallback(
  const PointCloud::ConstSharedPtr ipCloudMsg, int idx)
{
#ifdef DEBUG_BUILD
    RCLCPP_INFO(this->get_logger(), "%s | idx: %i | %i-%i", __PRETTY_FUNCTION__, idx,
                ipCloudMsg->header.stamp.sec, ipCloudMsg->header.stamp.nanosec);
#endif

    //--- lock mutex
    if (!pointCloudMutex_.try_lock())
        return;

    //--- copy point cloud into vector
    pPointClouds_[idx].reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*ipCloudMsg, *pPointClouds_[idx]);

    //--- store message headers
    pointCloudMsgHeaders_[idx] = ipCloudMsg->header;

    //--- if processing timer is not yet started, check if data is available for all clouds and
    //--- start timer if so
    if (!isTimerActive_)
    {
        bool isCloudDataAvailable = true;
        for (pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud : pPointClouds_)
        {
            if (pCloud == nullptr)
            {
                isCloudDataAvailable = false;
                break;
            }
        }
        if (isCloudDataAvailable)
        {
            processingTimer_->reset();
            isTimerActive_ = true;
        }
    }

    //--- unlock mutex
    pointCloudMutex_.unlock();
}

//==================================================================================================
void multisensor_calibration::visualizers::PointCloud2PointCloudDistanceNode::onCalibrationReceived(
  const CalibrationResult_Message_T::ConstSharedPtr& ipCalibMsg)
{
    if (!ipCalibMsg->is_successful)
        return;

    //--- Unpack the ref - local transformation
    tf2::Transform local2RefTransform, ref2LocalTransform;
    multisensor_calibration::utils::cvtGeometryTransform2TfTransform(ipCalibMsg->sensor_extrinsics, local2RefTransform);
    ref2LocalTransform = local2RefTransform.inverse();

    //--- If base is present, the calibration is src - base and not src - ref
    if (!ipCalibMsg->base_frame_id.empty())
    {
        geometry_msgs::msg::TransformStamped t;
        try
        {
            t = tfBuffer_->lookupTransform(ipCalibMsg->base_frame_id, ipCalibMsg->ref_frame_id,
                                           tf2::TimePointZero);
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_ERROR(this->get_logger(), "tf2::TransformException: %s",
                         ex.what());
            return;
        }

        tf2::Transform tmpTransform(tf2::Quaternion(t.transform.rotation.x,
                                                    t.transform.rotation.y,
                                                    t.transform.rotation.z,
                                                    t.transform.rotation.w),
                                    tf2::Vector3(t.transform.translation.x,
                                                 t.transform.translation.y,
                                                 t.transform.translation.z));

        ref2LocalTransform = ref2LocalTransform * tmpTransform;
    }
    std::cout << "Tmp Transform (XYZW | XYZW): " << ref2LocalTransform.getOrigin().x()
              << " " << ref2LocalTransform.getOrigin().y() << " " << ref2LocalTransform.getOrigin().z() << " "
              << ref2LocalTransform.getOrigin().w() << " | " << ref2LocalTransform.getRotation().x() << " "
              << ref2LocalTransform.getRotation().y() << " " << ref2LocalTransform.getRotation().z() << " "
              << ref2LocalTransform.getRotation().w() << std::endl;

    //--- Update the src - ref
    temporaryTransform_ = ref2LocalTransform;
}

//==================================================================================================
void multisensor_calibration::visualizers::PointCloud2PointCloudDistanceNode::processCloudsCallback()
{

#ifdef DEBUG_BUILD
    RCLCPP_INFO(this->get_logger(), "%s", __PRETTY_FUNCTION__);
#endif

    //--- lock mutex
    if (!pointCloudMutex_.try_lock())
        return;

    //--- loop over point clouds and do distance calculation to each of the other point clouds
    //--- store the minimum of all distances inside the intensity field
    for (size_t s = 0; s < pPointClouds_.size(); ++s)
    {
        // pointer to source point cloud for which the distance is to be computed
        pcl::PointCloud<pcl::PointXYZ>::Ptr pSourceXyzCloud = pPointClouds_[s];

        //--- initialize output cloud

        // source point cloud enhanced with the distance in the intensity field
        pcl::PointCloud<pcl::PointXYZI>::Ptr pEnhancedSourceXyziCloud(
          new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*pSourceXyzCloud, *pEnhancedSourceXyziCloud);

        // point indices that exceed clamp distance threshold
        pcl::IndicesPtr pClampedPointIndices(new pcl::Indices);

        // point-wise winner-takes-it-all distance
        std::vector<float> wtaDistance = std::vector<float>(pSourceXyzCloud->size(), FLT_MAX);

        for (size_t t = 0; t < pPointClouds_.size(); ++t)
        {
            //--- skip if target (t) equals to source (s)
            if (t == s)
                continue;

            //--- get transform from targetCloud to sourceCloud with tfListener
            //--- since the source cloud is to be enhanced with distance information, the target
            //--- cloud should be transformed into the coordinate system of the source cloud. In
            //--- this way, the source cloud does not need to be transformed back prior to
            //--- publishment.
            geometry_msgs::msg::TransformStamped transform;

            //--- if temporary transform is to be used, construct output from stamped transform,
            //--- else use tfListener_ to look up transform
            //--- temporary transform is only available if two clouds are used
            if (useTemporaryTransform_)
            {
                //--- if t is greater than s, i.e. if cloud2 to cloud1 is considered, use inverse
                if (s < t)
                {
                    transform.header.frame_id = pointCloudMsgHeaders_[s].frame_id;
                    transform.child_frame_id  = pointCloudMsgHeaders_[t].frame_id;

                    transform.header.stamp            = this->get_clock()->now();
                    transform.transform.rotation.w    = temporaryTransform_.getRotation().getW();
                    transform.transform.rotation.x    = temporaryTransform_.getRotation().getX();
                    transform.transform.rotation.y    = temporaryTransform_.getRotation().getY();
                    transform.transform.rotation.z    = temporaryTransform_.getRotation().getZ();
                    transform.transform.translation.x = temporaryTransform_.getOrigin().getX();
                    transform.transform.translation.y = temporaryTransform_.getOrigin().getY();
                    transform.transform.translation.z = temporaryTransform_.getOrigin().getZ();
                }
                else
                {
                    transform.header.frame_id = pointCloudMsgHeaders_[s].frame_id;
                    transform.child_frame_id  = pointCloudMsgHeaders_[t].frame_id;

                    transform.header.stamp            = this->get_clock()->now();
                    transform.transform.rotation.w    = temporaryTransform_.inverse().getRotation().getW();
                    transform.transform.rotation.x    = temporaryTransform_.inverse().getRotation().getX();
                    transform.transform.rotation.y    = temporaryTransform_.inverse().getRotation().getY();
                    transform.transform.rotation.z    = temporaryTransform_.inverse().getRotation().getZ();
                    transform.transform.translation.x = temporaryTransform_.inverse().getOrigin().getX();
                    transform.transform.translation.y = temporaryTransform_.inverse().getOrigin().getY();
                    transform.transform.translation.z = temporaryTransform_.inverse().getOrigin().getZ();
                }
            }
            else
            {
                try
                {
                    transform = tfBuffer_->lookupTransform(pointCloudMsgHeaders_[s].frame_id,
                                                           pointCloudMsgHeaders_[t].frame_id,
                                                           rclcpp::Time(0));
                }
                catch (tf2::TransformException& ex)
                {
                    RCLCPP_ERROR(this->get_logger(), "tf2::TransformException: %s", ex.what());
                    return;
                }
            }

            // target point cloud transformed into frame of source point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr pTransformedXyzTargetCloud(
              new pcl::PointCloud<pcl::PointXYZ>);
            pcl_ros::transformPointCloud(*pPointClouds_[t], *pTransformedXyzTargetCloud, transform);

            //--- initialize KD Search tree
            pcl::search::KdTree<pcl::PointXYZ>::Ptr pKdTree(new pcl::search::KdTree<pcl::PointXYZ>);
            pKdTree->setInputCloud(pTransformedXyzTargetCloud);

            //--- compute point normals Normal estimation ( if distance measure is set to point2surface)
            pcl::PointCloud<pcl::Normal>::Ptr pTargetNormals(new pcl::PointCloud<pcl::Normal>);
            if (distanceMeasure_ == POINT_2_SURFACE)
            {
                pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
                normalEstimation.setInputCloud(pTransformedXyzTargetCloud);
                normalEstimation.setSearchMethod(pKdTree);
                normalEstimation.setKSearch(nNearestNeighbors_);
                normalEstimation.compute(*pTargetNormals);
            }

            //--- loop over source point cloud
            //--- for each point, find n nearest neighbors in transformed point cloud, calculate distance
            //--- of source point to this subset, create new point in pEnhancedSourceXyziCloud and store
            //--- normalized distance in intensity value

#pragma omp parallel shared(pKdTree, pTransformedXyzTargetCloud, \
                              pEnhancedSourceXyziCloud, pTargetNormals, calculateDistanceFn_)
            {
#pragma omp for
                for (pcl::PointCloud<pcl::PointXYZI>::iterator srcPntItr = pEnhancedSourceXyziCloud->begin();
                     srcPntItr != pEnhancedSourceXyziCloud->end();
                     ++srcPntItr)
                {
                    // XYZ-Poitn from source point cloud
                    pcl::PointXYZ srcPntXyz(srcPntItr->x, srcPntItr->y, srcPntItr->z);

                    // index of currently processed point in point cloud
                    int pntIdx = (srcPntItr - pEnhancedSourceXyziCloud->begin());

                    //--- find nearest neighbors
                    pcl::IndicesPtr pNearestNeighborsIndices(new pcl::Indices(nNearestNeighbors_));
                    std::vector<float> nearestNeighborsDistances(nNearestNeighbors_);
                    pKdTree->nearestKSearch(srcPntXyz, nNearestNeighbors_,
                                            *pNearestNeighborsIndices, nearestNeighborsDistances);

                    //--- calculate distance
                    float distance = calculateDistanceFn_(srcPntXyz, pTransformedXyzTargetCloud,
                                                          pTargetNormals,
                                                          pNearestNeighborsIndices,
                                                          nearestNeighborsDistances);

                    //--- truncate, normalize and store in intensity, if smaller than the wta-distance
                    if (distance < wtaDistance[pntIdx])
                    {
                        srcPntItr->intensity = (std::min(distance, maxDistance_) / maxDistance_);

                        wtaDistance[pntIdx] = distance;
                    }
                }
            }
        }

        //--- find points whose wta distance exceed clamp distance threshold and add to
        //--- pClampedPointIndices
        //--- see: https://stackoverflow.com/questions/12990148/get-all-positions-of-elements-in-stl-vector-that-are-greater-than-a-value
        auto distanceCmpFn = [&](float distance)
        {
            return distance > clampDistanceThreshold_;
        };
        auto itr = std::find_if(std::begin(wtaDistance), std::end(wtaDistance), distanceCmpFn);
        while (itr != std::end(wtaDistance))
        {
            pClampedPointIndices->emplace_back(std::distance(std::begin(wtaDistance), itr));
            itr = std::find_if(std::next(itr), std::end(wtaDistance), distanceCmpFn);
        }

        //--- remove point that have been marked by exceeding clampDistanceThreshold_ threshold
        pcl::ExtractIndices<pcl::PointXYZI> indexExtraction;
        indexExtraction.setIndices(pClampedPointIndices);
        indexExtraction.setNegative(true);
        indexExtraction.filterDirectly(pEnhancedSourceXyziCloud);

        //--- publish colorized point cloud
        if (!pEnhancedSourceXyziCloud->empty())
        {
            sensor_msgs::msg::PointCloud2 cloudMsg;
            pcl::toROSMsg(*pEnhancedSourceXyziCloud, cloudMsg);
            cloudMsg.header = pointCloudMsgHeaders_[s];
            pubs_[s]->publish(cloudMsg);
        }
    }

    //--- unlock mutex
    pointCloudMutex_.unlock();
}

//==================================================================================================
float multisensor_calibration::visualizers::PointCloud2PointCloudDistanceNode::calculatePoint2PointDistance(
  const pcl::PointXYZ& iSourcePnt,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& iTargetCloud,
  const pcl::PointCloud<pcl::Normal>::Ptr& iTargetNormals,
  const pcl::IndicesPtr& iNnIndices,
  const std::vector<float>& iNnDistances)
{
    UNUSED(iSourcePnt)
    UNUSED(iTargetCloud)
    UNUSED(iTargetNormals)
    UNUSED(iNnIndices)

    return iNnDistances[0];
}

//==================================================================================================
float multisensor_calibration::visualizers::PointCloud2PointCloudDistanceNode::calculatePoint2SurfaceDistance(
  const pcl::PointXYZ& iSourcePnt,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& iTargetCloud,
  const pcl::PointCloud<pcl::Normal>::Ptr& iTargetNormals,
  const pcl::IndicesPtr& iNnIndices,
  const std::vector<float>& iNnDistances)
{
    UNUSED(iNnDistances)

    //--- return value
    float wtaDistance = FLT_MAX;

    //--- loop over normal vectors and calculate distance by computing the dot product between
    //--- the query point and the normal vector
    //--- select the absolut smallest distance as return value
    for (pcl::Indices::iterator idxItr = iNnIndices->begin();
         idxItr != iNnIndices->end();
         ++idxItr)
    {
        Eigen::Vector3f srcPointVec(iSourcePnt.x, iSourcePnt.y, iSourcePnt.z);
        Eigen::Vector3f targetPointVec(iTargetCloud->at(*idxItr).x,
                                       iTargetCloud->at(*idxItr).y,
                                       iTargetCloud->at(*idxItr).z);
        Eigen::Vector3f targetNormalVec(iTargetNormals->at(*idxItr).normal_x,
                                        iTargetNormals->at(*idxItr).normal_y,
                                        iTargetNormals->at(*idxItr).normal_z);
        targetNormalVec.normalize();

        float d = -1 * targetPointVec.dot(targetNormalVec);

        float distance = std::abs(srcPointVec.dot(targetNormalVec) + d);
        if (distance < wtaDistance)
            wtaDistance = distance;
    }

    return wtaDistance;
}

//==================================================================================================
std::vector<float> multisensor_calibration::visualizers::PointCloud2PointCloudDistanceNode::splitStringToFloat(
  const std::string& iStr,
  const char& iDelimiter) const
{
    std::vector<float> fltList;

    std::istringstream strStream(iStr);
    std::string s;
    while (getline(strStream, s, iDelimiter))
    {
        try
        {
            fltList.push_back(std::stof(s));
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(this->get_logger(), "%s: %s is not a number!",
                        __PRETTY_FUNCTION__, s.c_str());
        }
    }

    return fltList;
}

RCLCPP_COMPONENTS_REGISTER_NODE(multisensor_calibration::visualizers::PointCloud2PointCloudDistanceNode)
