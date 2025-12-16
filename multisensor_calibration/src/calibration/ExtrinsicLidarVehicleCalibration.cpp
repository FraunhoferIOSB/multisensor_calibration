/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/calibration/ExtrinsicLidarVehicleCalibration.h"

// Std
#define USE_MATH_DEFINES
#include <cmath>
#include <functional>

// PCL
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.hpp>

// ROS
#include <pcl_conversions/pcl_conversions.h>

// small_gicp
#include <small_gicp/registration/registration_helper.hpp>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/sensor_data_processing/LocalPlaneSacModel.h"

namespace multisensor_calibration
{

//==================================================================================================
ExtrinsicLidarVehicleCalibration::ExtrinsicLidarVehicleCalibration(
  const std::string& nodeName,
  const rclcpp::NodeOptions& options) :
  Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>(
    EXTRINSIC_LIDAR_VEHICLE_CALIBRATION),
  rclcpp::Node(nodeName, options),
  sensorDataProcessingMutex_(dataProcessingMutex_),
  pClickedPointSubsc_(nullptr),
  pRegionMarkerSrv_(nullptr),
  pSrcCloudSubsc_(nullptr),
  pRefCloudSubsc_(nullptr),
  pSrcRegionPub_(nullptr),
  pRefRegionPub_(nullptr),
  srcRegionMarkers_(),
  srcRegionSeedCloudPtrs_(),
  pSrcRegionsCloud_(nullptr),
  refRegionMarkers_(),
  refRegionSeedCloudPtrs_(),
  pRefRegionsCloud_(nullptr),
  regionMarkerHistory_(),
  pRefDataTransform_(nullptr)
{
    //--- set verbosity level for pcl
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    CalibrationBase::init(this);
}

//==================================================================================================
ExtrinsicLidarVehicleCalibration::ExtrinsicLidarVehicleCalibration(
  const rclcpp::NodeOptions& options) :
  ExtrinsicLidarVehicleCalibration(
    CALIB_TYPE_2_NODE_NAME.at(EXTRINSIC_LIDAR_VEHICLE_CALIBRATION),
    options)
{
}

//==================================================================================================
ExtrinsicLidarVehicleCalibration::~ExtrinsicLidarVehicleCalibration()
{
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibration::computeRegionsCloud(
  const InputCloud_Message_T::ConstSharedPtr& ipCloudMsg,
  const std::vector<InputPointType>& iRegionMarkers,
  std::vector<pcl::PointCloud<InputPointType>::Ptr>& oRegionSeedCloudPtrs,
  pcl::PointCloud<RegionPointType>::Ptr& opRegionsCloud) const
{
    // pointer to input point cloud from source sensor
    pcl::PointCloud<InputPointType>::Ptr pPointCloud(new pcl::PointCloud<InputPointType>);
    pcl::fromROSMsg(*ipCloudMsg, *pPointCloud);

    // pointer KD Search tree
    pcl::search::Search<InputPointType>::Ptr pSearchKdTree(new pcl::search::KdTree<InputPointType>);
    pSearchKdTree->setInputCloud(pPointCloud);

    //--- reset regions cloud
    opRegionsCloud.reset(new pcl::PointCloud<RegionPointType>);

    //--- resize list of seed clouds to size size of marker list
    while (oRegionSeedCloudPtrs.size() < iRegionMarkers.size())
        oRegionSeedCloudPtrs.push_back(nullptr);

    //--- loop over marker points, find nearest neighbor in cloud and do region growing
    for (std::vector<InputPointType>::const_iterator markerItr = iRegionMarkers.cbegin();
         markerItr != iRegionMarkers.cend();
         ++markerItr)
    {
        // Id of cluster
        int clusterIdx = std::distance(iRegionMarkers.cbegin(), markerItr);

        //--- reset cloud of region seed points
        oRegionSeedCloudPtrs.at(clusterIdx).reset(new pcl::PointCloud<InputPointType>);

        //--- find nearest neighbors
        std::vector<int> nnIndices(registrationParams_.region_num_neighbors.value);            // nearest neighbor indices
        std::vector<float> nnSquaredDistances(registrationParams_.region_num_neighbors.value); // nearest neighbor distances
        pSearchKdTree->nearestKSearch(*markerItr, registrationParams_.region_num_neighbors.value,
                                      nnIndices, nnSquaredDistances);

        // smart pointer to cluster indices
        pcl::PointIndices::Ptr pTmpPntIndices(new pcl::PointIndices());
        pTmpPntIndices->indices = nnIndices;

        //--- extract cluster indices
        pcl::ExtractIndices<InputPointType> extractIndices;
        extractIndices.setInputCloud(pPointCloud);
        extractIndices.setIndices(pTmpPntIndices);
        extractIndices.filter(*oRegionSeedCloudPtrs.at(clusterIdx));

        //--- detect plane around seed point with RANSAC
        //--- first only use the k nearest neighbors around seed point, then select select all
        //--- points that are within a distance threshold of the model.
        //--- this can further be restricted by using a confined local plane
        Eigen::VectorXf planeParameters;
        pcl::SampleConsensusModelPlane<InputPointType>::Ptr pPlaneSacModel = nullptr;

        //--- check which plane model to use
        if (registrationParams_.region_use_local_plane.value)
        {
            LocalPlaneSacModel<InputPointType>::Ptr pLocalPlaneSacModel(
              new LocalPlaneSacModel<InputPointType>(oRegionSeedCloudPtrs.at(clusterIdx)));
            Eigen::Vector3f seedPnt = Eigen::Vector3f(pPointCloud->at(nnIndices.front()).x,
                                                      pPointCloud->at(nnIndices.front()).y,
                                                      pPointCloud->at(nnIndices.front()).z);
            pLocalPlaneSacModel->setCenter(seedPnt);
            pLocalPlaneSacModel->setRadius(static_cast<float>(
              registrationParams_.local_plane_radius.value));
            pLocalPlaneSacModel->setIncrementalCenterUpdate(false);

            pPlaneSacModel = pLocalPlaneSacModel;
        }
        else
        {
            pPlaneSacModel.reset(
              new pcl::SampleConsensusModelPlane<InputPointType>(oRegionSeedCloudPtrs.at(clusterIdx)));
        }

        //--- compute model by Ransac
        pcl::RandomSampleConsensus<InputPointType> planeRansac(pPlaneSacModel);
        planeRansac.setDistanceThreshold(registrationParams_.local_plane_distance_thresh.value);
        bool isSuccessful = planeRansac.computeModel();
        if (!isSuccessful)
            continue;
        planeRansac.refineModel();
        planeRansac.getModelCoefficients(planeParameters);

        //--- select cluster from full input cloud by using estimated plane coefficients
        pcl::PointCloud<InputPointType>::Ptr pTmpClusterCloud(new pcl::PointCloud<InputPointType>);
        pTmpPntIndices.reset(new pcl::PointIndices());

        pPlaneSacModel->setInputCloud(pPointCloud);
        pPlaneSacModel->selectWithinDistance(planeParameters,
                                             registrationParams_.local_plane_distance_thresh.value,
                                             pTmpPntIndices->indices);

        extractIndices.setIndices(pTmpPntIndices);
        extractIndices.filter(*pTmpClusterCloud);

        //--- set index of region marker to intensity, normal vector of sac model and push to output
        std::for_each(pTmpClusterCloud->begin(), pTmpClusterCloud->end(),
                      [&](InputPointType& iPnt)
                      {
                          RegionPointType oPnt;
                          oPnt.x         = iPnt.x;
                          oPnt.y         = iPnt.y;
                          oPnt.z         = iPnt.z;
                          oPnt.intensity = static_cast<float>(clusterIdx);
#ifdef USE_NORMAL_INFO
                          oPnt.normal_x = planeParameters[0];
                          oPnt.normal_y = planeParameters[1];
                          oPnt.normal_z = planeParameters[2];
#endif

                          //--- added point to cumulative cloud
                          opRegionsCloud->push_back(oPnt);
                      });
    }
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibration::doCoarseCalibration()
{
    using namespace pcl::registration;

    // check that enough seed data is available
    if (srcRegionSeedCloudPtrs_.size() < calibrationItrCnt_ ||
        refRegionSeedCloudPtrs_.size() < calibrationItrCnt_)
        return;

    //--- merge seed clouds used for single calibration iteration in order to compute a
    //--- good guess for the gicp

    // Pointer to seed cloud of source region
    pcl::PointCloud<InputPointType>::Ptr pSrcRegionSeedCloud(new pcl::PointCloud<InputPointType>);
    // Pointer to seed cloud of reference region
    pcl::PointCloud<InputPointType>::Ptr pRefRegionSeedCloud(new pcl::PointCloud<InputPointType>);
    for (uint i = 0; i < calibrationItrCnt_; ++i)
    {
        pSrcRegionSeedCloud->insert(pSrcRegionSeedCloud->end(),
                                    srcRegionSeedCloudPtrs_[i]->begin(),
                                    srcRegionSeedCloudPtrs_[i]->end());
        pRefRegionSeedCloud->insert(pRefRegionSeedCloud->end(),
                                    refRegionSeedCloudPtrs_[i]->begin(),
                                    refRegionSeedCloudPtrs_[i]->end());
    }

    // Correspondences of marker corners between src and ref cloud. Since the marker corners are
    // extracted in order, the correspondence list simply consists of increasing index numbers
    pcl::Correspondences correspondences;
    for (uint i = 0; i < pSrcRegionSeedCloud->size(); ++i)
    {
        correspondences.push_back(pcl::Correspondence(i, i, 1.f));
    }

    //--- estimate sensor extrinsics from marker corners
    auto sensorExtrinsic = computeExtrinsicsFromPointCorrespondences<InputPointType>(
      pSrcRegionSeedCloud, pRefRegionSeedCloud, correspondences);
    sensorExtrinsics_.push_back(sensorExtrinsic);

    //--- increment iteration count
    calibrationItrCnt_++;

    //--- publish last sensor extrinsics
    ExtrinsicCalibrationBase::publishLastCalibrationResult();
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibration::finalizeCalibration()
{
    //--- do statistical outlier filtering
    pcl::StatisticalOutlierRemoval<RegionPointType> sorFilter;
    sorFilter.setMeanK(50);            // TODO: dynconfig
    sorFilter.setStddevMulThresh(1.0); // TODO: dynconfig
    sorFilter.setInputCloud(pSrcRegionsCloud_);
    sorFilter.filter(*pSrcRegionsCloud_);
    sorFilter.setInputCloud(pRefRegionsCloud_);
    sorFilter.filter(*pRefRegionsCloud_);

    if (pSrcRegionsCloud_->empty() || pRefRegionsCloud_->empty())
    {
        RCLCPP_ERROR(logger_, "Could not finalize calibration. No common observations available.");
        return false;
    }

    //--- run GICP
    double icpRmse = runIcp<RegionPointType>(
      pSrcRegionsCloud_, pRefRegionsCloud_,
      static_cast<small_gicp::RegistrationSetting::RegistrationType>(
        registrationParams_.registration_icp_variant.value),
      registrationParams_.registration_icp_max_correspondence_distance.value,
      registrationParams_.registration_icp_rotation_tolerance.value,
      registrationParams_.registration_icp_translation_tolerance.value);

    ExtrinsicCalibrationBase::updateCalibrationResult(std::make_pair("Root Mean Squared Error (in m)", icpRmse), static_cast<int>(refRegionMarkers_.size()));

    //--- publish last sensor extrinsics
    ExtrinsicCalibrationBase::publishLastCalibrationResult();

    return true;
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibration::initializePublishers(rclcpp::Node* ipNode)
{
    if (!ExtrinsicCalibrationBase::initializePublishers(ipNode))
        return false;

    pSrcRegionPub_ = ipNode->create_publisher<RoisCloud_Message_T>(
      "~/" + srcSensorName_ + "/" + ROIS_CLOUD_TOPIC_NAME, 10);

    pRefRegionPub_ = ipNode->create_publisher<RoisCloud_Message_T>(
      "~/" + refSensorName_ + "/" + ROIS_CLOUD_TOPIC_NAME, 10);

    return true;
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibration::initializeServices(rclcpp::Node* ipNode)
{
    if (!ExtrinsicCalibrationBase::initializeServices(ipNode))
        return false;

    //--- shutdown service to capture calibration target, since not needed
    CalibrationBase::pCaptureSrv_.reset();
    CalibrationBase::pCaptureSrv_ = nullptr;

    //--- add region marker service
    pRegionMarkerSrv_ = ipNode->create_service<interf::srv::AddRegionMarker>(
      "~/" + ADD_REGION_MARKER_SRV_NAME,
      std::bind(&ExtrinsicLidarVehicleCalibration::onRequestAddRegionMarker, this,
                std::placeholders::_1, std::placeholders::_2));

    return true;
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibration::initializeSubscribers(rclcpp::Node* ipNode)
{
    //--- subscribe to topics with clicked point from rviz
    pClickedPointSubsc_ = ipNode->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 1,
      std::bind(&ExtrinsicLidarVehicleCalibration::onPointClicked, this,
                std::placeholders::_1));

    //--- subscribe to topics with name cloudTopic
    pSrcCloudSubsc_ = ipNode->create_subscription<InputCloud_Message_T>(
      srcTopicName_, 1,
      std::bind(&ExtrinsicLidarVehicleCalibration::onSensorDataReceived, this,
                std::placeholders::_1));
    pRefCloudSubsc_ = ipNode->create_subscription<InputCloud_Message_T>(
      refTopicName_, 1,
      std::bind(&ExtrinsicLidarVehicleCalibration::onReferenceDataReceived, this,
                std::placeholders::_1));

    return true;
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibration::initializeWorkspaceObjects()
{
    bool retVal = true;

    retVal &= CalibrationBase::initializeWorkspaceObjects();

    //--- initialize calibration workspace
    fs::path calibWsPath = robotWsPath_;
    calibWsPath /=
      std::string(srcSensorName_ + "_" + refSensorName_ + "_extrinsic_calibration");
    pCalibrationWs_ =
      std::make_shared<ExtrinsicLidarVehicleCalibWorkspace>(calibWsPath, logger_);
    retVal &= (pCalibrationWs_ != nullptr);

    if (pCalibrationWs_)
    {
        //--- create backup
        retVal &= pCalibrationWs_->createBackup();
    }

    return retVal;
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibration::onPointClicked(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr& ipPointMsg)
{
    //--- create service message and call method
    std::shared_ptr<interf::srv::AddRegionMarker::Request> pReq =
      std::make_shared<interf::srv::AddRegionMarker::Request>();
    std::shared_ptr<interf::srv::AddRegionMarker::Response> pRes =
      std::make_shared<interf::srv::AddRegionMarker::Response>();

    pReq->sensor_frame_id = ipPointMsg->header.frame_id;
    pReq->point           = ipPointMsg->point;
    if (onRequestAddRegionMarker(pReq, pRes))
    {
        RCLCPP_INFO(logger_, "%s", pRes->msg.c_str());
    }
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibration::onRequestAddRegionMarker(
  const std::shared_ptr<interf::srv::AddRegionMarker::Request> ipReq,
  std::shared_ptr<interf::srv::AddRegionMarker::Response> opRes)
{
    // Marker point to store in list
    InputPointType pnt;
    pnt.x = static_cast<float>(ipReq->point.x);
    pnt.y = static_cast<float>(ipReq->point.y);
    pnt.z = static_cast<float>(ipReq->point.z);

    if (ipReq->sensor_frame_id == refFrameId_)
    {
        //--- get ownership of mutex
        std::lock_guard<std::mutex> guard(refDataProcessingMutex_);

        refRegionMarkers_.push_back(pnt);
        regionMarkerHistory_.push_back(refFrameId_);

        opRes->is_accepted = true;
        opRes->msg         = "Added marker position to reference.";
    }
    else if (ipReq->sensor_frame_id == srcFrameId_)
    {
        //--- get ownership of mutex
        std::lock_guard<std::mutex> guard(sensorDataProcessingMutex_);

        srcRegionMarkers_.push_back(pnt);
        regionMarkerHistory_.push_back(srcFrameId_);

        opRes->is_accepted = true;
        opRes->msg         = "Added marker position to source.";
    }
    else
    {
        opRes->is_accepted = false;
        opRes->msg         = "No valid sensor frame ID.";
    }

    return true;
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibration::onRequestRemoveObservation(
  const std::shared_ptr<interf::srv::RemoveLastObservation::Request> ipReq,
  std::shared_ptr<interf::srv::RemoveLastObservation::Response> opRes)
{
    UNUSED_VAR(ipReq);

    //--- get ownership of mutex
    std::lock_guard<std::mutex> srcGuard(sensorDataProcessingMutex_);
    std::lock_guard<std::mutex> refGuard(refDataProcessingMutex_);

    //--- check in history list for last frame ID and remove from appropriate list.
    if (!regionMarkerHistory_.empty())
    {
        opRes->is_accepted = true;

        if (regionMarkerHistory_.back() == refFrameId_)
        {
            refRegionMarkers_.pop_back();
            opRes->msg = "Removed last marker from reference.";
        }
        else if (regionMarkerHistory_.back() == srcFrameId_)
        {
            srcFrameId_.pop_back();
            opRes->msg = "Removed last marker from source.";
        }
        else
        {
            opRes->is_accepted = false;
            opRes->msg         = "Could not find frame ID in history list.";
        }

        regionMarkerHistory_.pop_back();
        if (calibrationItrCnt_ > 1)
        {
            calibrationItrCnt_--;
            sensorExtrinsics_.pop_back();
        }
    }
    else
    {
        opRes->is_accepted = false;
        opRes->msg         = "No region marker in history list.";
    }

    RCLCPP_INFO(logger_, "%s", opRes->msg.c_str());

    return true;
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibration::onReferenceDataReceived(
  const InputCloud_Message_T::ConstSharedPtr& ipRefCloudMsg)
{
    //--- check if node is initialized
    if (!isInitialized_)
    {
        RCLCPP_ERROR(logger_, "Node is not initialized.");
        return;
    }

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(refDataProcessingMutex_);

    //--- set frame ids and initialize sensor extrinsics if applicable
    if (refFrameId_ != ipRefCloudMsg->header.frame_id)
    {
        refFrameId_ = ipRefCloudMsg->header.frame_id;

        //--- if base frame id is not empty and unequal to refCloudFrameId also get transform
        //--- between refFrameID and baseFrameID and pass to refLidarProcessor.
        if (!baseFrameId_.empty() && baseFrameId_ != refFrameId_)
        {
            if (tfBuffer_->_frameExists(baseFrameId_))
            {
                try
                {
                    auto t = tfBuffer_->lookupTransform(baseFrameId_, refFrameId_,
                                                        tf2::TimePointZero);
                    pRefDataTransform_ =
                      std::make_shared<tf2::Transform>(tf2::Quaternion(t.transform.rotation.x,
                                                                       t.transform.rotation.y,
                                                                       t.transform.rotation.z,
                                                                       t.transform.rotation.w),
                                                       tf2::Vector3(t.transform.translation.x,
                                                                    t.transform.translation.y,
                                                                    t.transform.translation.z));
                }
                catch (tf2::TransformException& ex)
                {
                    RCLCPP_ERROR(logger_, "tf2::TransformException: %s",
                                 ex.what());
                }
            }
            else
            {
                RCLCPP_WARN(logger_, "Base Frame %s does not exists! "
                                     "Removing base frame and calibrating relative "
                                     "to reference cloud.",
                            baseFrameId_.c_str());
                baseFrameId_       = "";
                pRefDataTransform_ = nullptr;
            }
        }
    }

    //--- return immediately if no markers are set
    if (refRegionMarkers_.empty())
        return;

    //--- compute regions for calibration
    computeRegionsCloud(ipRefCloudMsg, refRegionMarkers_, refRegionSeedCloudPtrs_, pRefRegionsCloud_);

    //--- transform reference data if applicable
    if (pRefDataTransform_)
    {
        pcl::PointCloud<InputPointType>::Ptr pTmpCloud(new pcl::PointCloud<InputPointType>);
        pcl_ros::transformPointCloud(*pRefRegionsCloud_, *pTmpCloud, *pRefDataTransform_);

        //--- swap pointer between input and temporary cloud
        pTmpCloud.swap(pRefRegionsCloud_);
        pTmpCloud->clear();
    }

    //--- publish clustered regions
    if (pRefRegionsCloud_)
    {
        RoisCloud_Message_T cloudMsg;
        pcl::toROSMsg(*pRefRegionsCloud_, cloudMsg);
        cloudMsg.header.stamp    = ipRefCloudMsg->header.stamp;
        cloudMsg.header.frame_id = (pRefDataTransform_) ? baseFrameId_ : refFrameId_;
        pRefRegionPub_->publish(cloudMsg);
    }

    //--- If calibration iteration count is equal to the smallest number of region markers, i.e.
    //--- if a new region marker has been added and not yet been calibrated, run single calibration
    //--- iteration. In this the calibration iteration count will also be increased by one.
    if (std::min(srcRegionMarkers_.size(), refRegionMarkers_.size()) == calibrationItrCnt_)
        doCoarseCalibration();
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibration::onSensorDataReceived(
  const InputCloud_Message_T::ConstSharedPtr& ipSrcCloudMsg)
{
    //--- check if node is initialized
    if (!isInitialized_)
    {
        RCLCPP_ERROR(logger_, "Node is not initialized.");
        return;
    }

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(sensorDataProcessingMutex_);

    //--- set frame ids and initialize sensor extrinsics if applicable
    if (srcFrameId_ != ipSrcCloudMsg->header.frame_id)
    {
        srcFrameId_ = ipSrcCloudMsg->header.frame_id;
    }

    //--- return immediately if no markers are set
    if (srcRegionMarkers_.empty())
        return;

    //--- compute regions for calibration
    computeRegionsCloud(ipSrcCloudMsg, srcRegionMarkers_, srcRegionSeedCloudPtrs_, pSrcRegionsCloud_);

    //--- publish clustered regions
    if (pSrcRegionsCloud_)
    {
        RoisCloud_Message_T cloudMsg;
        pcl::toROSMsg(*pSrcRegionsCloud_, cloudMsg);
        cloudMsg.header = ipSrcCloudMsg->header;
        pSrcRegionPub_->publish(cloudMsg);
    }
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibration::saveCalibrationSettingsToWorkspace()
{
    if (!ExtrinsicCalibrationBase::saveCalibrationSettingsToWorkspace())
        return false;

    QSettings* pCalibSettings = pCalibrationWs_->settingsPtr();
    if (!pCalibSettings)
        return false;

    //--- source lidar sensor name
    pCalibSettings->setValue("source_lidar/sensor_name",
                             QString::fromStdString(srcSensorName_));

    //--- source lidar cloud topic
    pCalibSettings->setValue("source_lidar/cloud_topic",
                             QString::fromStdString(srcTopicName_));

    //--- reference name
    pCalibSettings->setValue("reference/name",
                             QString::fromStdString(refSensorName_));

    //--- reference cloud topic
    pCalibSettings->setValue("reference/cloud_topic",
                             QString::fromStdString(refTopicName_));

    //--- sync settings file
    pCalibSettings->sync();

    return true;
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibration::setupLaunchParameters(rclcpp::Node* ipNode) const
{
    ExtrinsicCalibrationBase::setupLaunchParameters(ipNode);

    //--- src_lidar_sensor_name
    auto srcSensorNameDesc = rcl_interfaces::msg::ParameterDescriptor{};
    srcSensorNameDesc.description =
      "Name of the source LiDAR sensor which is to be calibrated.\n"
      "Default: \"lidar\"";
    srcSensorNameDesc.read_only = true;
    ipNode->declare_parameter<std::string>("src_lidar_sensor_name", DEFAULT_LIDAR_SENSOR_NAME,
                                           srcSensorNameDesc);

    //--- src_lidar_cloud_topic
    auto srcCloudTopicDesc = rcl_interfaces::msg::ParameterDescriptor{};
    srcCloudTopicDesc.description =
      "Topic name of the corresponding LiDAR cloud.\n"
      "Default: \"/lidar/cloud\"";
    srcCloudTopicDesc.read_only = true;
    ipNode->declare_parameter<std::string>("src_lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC,
                                           srcCloudTopicDesc);

    //--- ref_lidar_cloud_topic
    auto refNameDesc = rcl_interfaces::msg::ParameterDescriptor{};
    refNameDesc.description =
      "Topic name of the reference cloud.\n"
      "Default: \"lidar\"";
    refNameDesc.read_only = true;
    ipNode->declare_parameter<std::string>("ref_lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC,
                                           refNameDesc);
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibration::setupDynamicParameters(rclcpp::Node* ipNode) const
{
    registrationParams_.declareDynamic(ipNode);
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibration::readLaunchParameters(const rclcpp::Node* ipNode)
{
    if (!ExtrinsicCalibrationBase::readLaunchParameters(ipNode))
        return false;

    //--- source_lidar_sensor_name
    srcSensorName_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "src_lidar_sensor_name", DEFAULT_LIDAR_SENSOR_NAME);

    //--- source_lidar_cloud_topic
    srcTopicName_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "src_lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC);

    //--- reference_lidar_cloud_topic
    refTopicName_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "ref_lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC);

    //--- set reference name to 'vehicle'
    refSensorName_ = "vehicle";

    return true;
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibration::setDynamicParameter(const rclcpp::Parameter& iParameter)
{
    if (CalibrationBase::setDynamicParameter(iParameter))
    {
        return true;
    }
    if (registrationParams_.tryToSetParameter(iParameter))
    {
        return true;
    }
    else
    {
        return false;
    }
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibration::reset()
{
    ExtrinsicCalibrationBase::reset();

    srcRegionMarkers_.clear();
    srcRegionSeedCloudPtrs_.clear();
    pSrcRegionsCloud_->clear();
    refRegionMarkers_.clear();
    refRegionSeedCloudPtrs_.clear();
    pRefRegionsCloud_->clear();

    regionMarkerHistory_.clear();
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibration::shutdownSubscribers()
{
    //--- check if node is initialized
    if (!isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> srcGuard(sensorDataProcessingMutex_);
    std::lock_guard<std::mutex> refGuard(refDataProcessingMutex_);

    //--- unsubscribe subscribers
    pSrcCloudSubsc_.reset();
    pSrcCloudSubsc_ = nullptr;

    pRefCloudSubsc_.reset();
    pRefCloudSubsc_ = nullptr;

    return true;
}

} // namespace multisensor_calibration

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(multisensor_calibration::ExtrinsicLidarVehicleCalibration)
