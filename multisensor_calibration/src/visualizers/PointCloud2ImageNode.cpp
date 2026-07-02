#include "multisensor_calibration/visualizers/PointCloud2ImageNode.h"

// Std
#include <cmath>
#include <iostream>

// ROS
#include <cv_bridge/cv_bridge.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// PCL
#include <pcl/conversions.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/range_image/range_image_planar.h>

// OpenCV
#include <opencv2/calib3d.hpp>

// multisensor_calibration
#define NO_QT
#include <multisensor_calibration/common/lib3D/core/visualization.hpp>
#include <multisensor_calibration/common/utils.hpp>

namespace multisensor_calibration
{
namespace visualizers
{

static const std::string INPUT_CLOUD_TOPIC_NAME       = "pointcloud";
static const std::string INPUT_IMAGE_TOPIC_NAME       = "image";
static const std::string INPUT_CALIBRATION_TOPIC_NAME = "calibration";

static const std::string OUTPUT_TOPIC_NAME = "fused_image";

static float DEFAULT_MIN_DEPTH = 0.1f;
static float DEFAULT_MAX_DEPTH = 20.f;

static int PROJECTED_PIXEL_RADIUS = 3;

//==================================================================================================
PointCloud2ImageNode::PointCloud2ImageNode(rclcpp::NodeOptions iNodeOptions, std::string iNodeName) :
  rclcpp::Node(iNodeName, iNodeOptions),
  isInitialized_(false),
  pImgCloudApproxSync_(nullptr),
  pImgCloudExactSync_(nullptr),
  tfBuffer_(this->get_clock()),
  tfListener_(tfBuffer_),
  imageState_(STR_2_IMG_STATE.at(DEFAULT_IMG_STATE_STR)),
  cameraNamespace_(""),
  minDepth_(DEFAULT_MIN_DEPTH),
  maxDepth_(DEFAULT_MAX_DEPTH),
  useTemporaryTransform_(false),
  syncQueueSize_(DEFAULT_SYNC_QUEUE_SIZE),
  useExactSync_(false)
{
    std::setlocale(LC_ALL, "en_US.UTF-8");
    onInit();
}

//==================================================================================================
PointCloud2ImageNode::~PointCloud2ImageNode()
{
    //--- reset pointers message filters before rest of the class in order to avoid seg fault during
    //--- disconnection of callbacks.
    pImgCloudApproxSync_.reset();
    pImgCloudExactSync_.reset();
}

//==================================================================================================
void PointCloud2ImageNode::computeCameraPoseForFrustumCulling(
  const lib3d::Camera& iCamera,
  Eigen::Matrix4f& oFrustumCullingPose) const
{
    Eigen::Matrix4f eigenRtMatrix;
    cv::cv2eigen(cv::Mat(iCamera.extrinsics.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF)),
                 eigenRtMatrix);
    Eigen::Matrix4f cam2robot; // see documentation of pcl::FrustumCulling
    cam2robot << 0, 0, 1, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    oFrustumCullingPose = eigenRtMatrix * cam2robot;
}

//==================================================================================================
void PointCloud2ImageNode::doFrustumCulling(
  const pcl::PointCloud<InputPointType>::Ptr& ipCloud,
  const lib3d::Camera& iCamera,
  const float& iMinDepth,
  const float& iMaxDepth,
  pcl::Indices& oCulledPointIndices) const
{
    pcl::FrustumCulling<InputPointType> frustumCulling; // object of frustum culling
    Eigen::Matrix4f frustumCullingCamPose;              // camera pose used for frustum culling
    computeCameraPoseForFrustumCulling(iCamera, frustumCullingCamPose);
    frustumCulling.setHorizontalFOV(static_cast<float>(iCamera.intrinsics.getHFov()) + 50);
    frustumCulling.setVerticalFOV(static_cast<float>(iCamera.intrinsics.getVFov()) + 50);
    frustumCulling.setCameraPose(frustumCullingCamPose);
    frustumCulling.setInputCloud(ipCloud);
    frustumCulling.setNearPlaneDistance(iMinDepth);
    frustumCulling.setFarPlaneDistance(iMaxDepth);
    frustumCulling.filter(oCulledPointIndices);
}

//==================================================================================================
static inline void getImageFromMessage(const sensor_msgs::msg::Image::ConstSharedPtr& ipImgMsg,
                                       cv::Mat& oImage,
                                       std::string& oFrameId)
{
    //--- get frame id from header
    oFrameId = ipImgMsg->header.frame_id;

    //--- get image from message
    cv_bridge::CvImageConstPtr pCvImg;
    try
    {
        pCvImg = cv_bridge::toCvShare(ipImgMsg);
    }
    catch (cv_bridge::Exception& e)
    {
        std::cerr << "[%s] cv_bridge::Exception: " << e.what() << std::endl;
        return;
    }

    //--- convert to RGB or simply copy to member variable
    if (pCvImg->encoding == "mono8")
        cv::cvtColor(pCvImg->image, oImage, CV_GRAY2RGB);
    else if (pCvImg->encoding == "bgr8")
        cv::cvtColor(pCvImg->image, oImage, CV_BGR2RGB);
    else
        pCvImg->image.copyTo(oImage);
}

//==================================================================================================
void PointCloud2ImageNode::onSensorDataReceived(const InputImage_Message_T::ConstSharedPtr& ipImgMsg,
                                                const InputCloud_Message_T::ConstSharedPtr& ipCloudMsg)
{
    //--- check if node is initialized
    if (!isInitialized_)
        return;

    //--- get camera image from message
    cv::Mat img;                 // object of camera image
    std::string imgFrameId = ""; // frame id of camera image
    if (ipImgMsg != nullptr)
        getImageFromMessage(ipImgMsg, img, imgFrameId);

    //--- get cloud data
    std::string cloudFrameId = ipCloudMsg->header.frame_id; // frame id of point cloud
    pcl::PointCloud<InputPointType>::Ptr pInputPointCloud(  // pointer to input point cloud
      new pcl::PointCloud<InputPointType>);
    pcl::fromROSMsg(*ipCloudMsg, *pInputPointCloud);

    //--- get transform between frameIds
    geometry_msgs::msg::TransformStamped cloudTransform;
    bool isSuccessful = true;
    if (!imgFrameId.empty())
        isSuccessful &= getTransformBetweenTfFrames(imgFrameId,
                                                    cloudFrameId,
                                                    cloudTransform);
    if (!isSuccessful)
        return;

    //--- set camera extrinsics from tf transform
    if (!imgFrameId.empty())
        utils::setCameraExtrinsicsFromTfTransform(
          cloudTransform, camera_.extrinsics);

    //--- perform view frustum culling for initial filtering of the point cloud
    pcl::Indices culledPntIndices;
    if (!imgFrameId.empty())
        doFrustumCulling(pInputPointCloud, camera_, minDepth_, maxDepth_, culledPntIndices);

    //--- loop over point cloud and create depth maps
    lib3d::DepthMap depthMap(img.size()); // depth map for image
#ifndef DEBUG
#pragma omp parallel shared(pInputPointCloud, culledPntIndices, depthMap, camera_)
#endif
    {
        //--- Project points that are only visible in vis image

#ifndef DEBUG
#pragma omp for
#endif
        for (std::vector<int>::const_iterator idxItr = culledPntIndices.begin();
             idxItr != culledPntIndices.end();
             ++idxItr)
        {
            projectCloudPointIntoDepthMap(pInputPointCloud->points[*idxItr],
                                          camera_, depthMap);
        }
    }

    //--- colorize depth maps and create fused images
    //--- invert colorization due to rgb instead of bgr
    cv::Mat fusedImage;
    float minVal = minDepth_;
    float maxVal = maxDepth_;
    if (!depthMap.empty())
        fusedImage = lib3d::colorizeRangeImg(depthMap, lib3d::COLORMAP_RAINBOW,
                                             true, &minVal, &maxVal,
                                             img, 1.f);

    //--- publish fused images
    if (!fusedImage.empty())
    {
        auto msgHeader = ipImgMsg->header;
        auto imgMsg    = cv_bridge::CvImage(msgHeader,
                                            sensor_msgs::image_encodings::RGB8,
                                            fusedImage)
                        .toImageMsg();
        pPub_->publish(*imgMsg);
    }
}

//==================================================================================================
bool PointCloud2ImageNode::getTransformBetweenTfFrames(
  const std::string& iTargetFrame,
  const std::string& iSourceFrame,
  geometry_msgs::msg::TransformStamped& oTransform) const
{
    //--- if temporary transform is to be used, construct output from stamped transform,
    //--- else use tfListener_ to look up transform
    if (useTemporaryTransform_)
    {
        oTransform                         = geometry_msgs::msg::TransformStamped();
        oTransform.header.stamp            = this->get_clock()->now();
        oTransform.header.frame_id         = iSourceFrame;
        oTransform.child_frame_id          = iTargetFrame;
        oTransform.transform.rotation.w    = temporaryTransform_.getRotation().getW();
        oTransform.transform.rotation.x    = temporaryTransform_.getRotation().getX();
        oTransform.transform.rotation.y    = temporaryTransform_.getRotation().getY();
        oTransform.transform.rotation.z    = temporaryTransform_.getRotation().getZ();
        oTransform.transform.translation.x = temporaryTransform_.getOrigin().getX();
        oTransform.transform.translation.y = temporaryTransform_.getOrigin().getY();
        oTransform.transform.translation.z = temporaryTransform_.getOrigin().getZ();
    }
    else
    {

        //--- get transform between cloudFrameId_ and imageFrameId_ from tfListener
        try
        {
            oTransform = tfBuffer_.lookupTransform(iTargetFrame, iSourceFrame, rclcpp::Time(0));
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_ERROR(this->get_logger(), "[%s] tf::TransformException: %s", this->get_name(), ex.what());
            return false;
        }
    }

    return true;
}

//==================================================================================================
template <typename T>
inline bool waitForMessage(rclcpp::Node* iNodePtr, std::string& iTopicName, T& msg, uint64_t iMsecondsToWait)
{
    auto sub = iNodePtr->create_subscription<T>(iTopicName, 1, [](const std::shared_ptr<T>) {});
    return rclcpp::wait_for_message<T>(msg, sub, iNodePtr->get_node_options().context(), std::chrono::milliseconds(iMsecondsToWait));
}

//==================================================================================================
bool PointCloud2ImageNode::initializeCameraData(
  const std::string& iImgTopic,
  const EImageState& iImgState,
  const std::string& iCameraNamespace,
  lib3d::Camera& oCamera)
{
    //--- construct camera info topic, if camera_namespace is empty construct from image topic
    std::string cameraInfoTopicName = utils::image2CameraInfoTopic(iImgTopic, iCameraNamespace);

    //--- get camera intrinsics from camera_info
    sensor_msgs::msg::CameraInfo camInfo;
    if (waitForMessage<sensor_msgs::msg::CameraInfo>(this, cameraInfoTopicName, camInfo, 10000))
    {
        utils::setCameraIntrinsicsFromCameraInfo(camInfo, oCamera.intrinsics, iImgState);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "[%s] Wait for message of camera_info topic has timed out. "
                                        "Camera intrinsics will not be set! ",
                    this->get_name());

        return false;
    }

    return true;
}

//==================================================================================================
bool PointCloud2ImageNode::initializePublishers()
{
    pPub_ = this->create_publisher<sensor_msgs::msg::Image>(OUTPUT_TOPIC_NAME, 10);
    return true;
}

//==================================================================================================
bool PointCloud2ImageNode::initializeSubscribers()
{
    //--- subscribe to topics
    rclcpp::QoS qos(rclcpp::KeepLast(1));

    imageSubsc_.subscribe(*this, INPUT_IMAGE_TOPIC_NAME, qos);
    cloudSubsc_.subscribe(*this, INPUT_CLOUD_TOPIC_NAME, qos);

    //--- initialize synchronizers
    //--- choose synchronization model and message callback dependent on fusion mode
    if (useExactSync_)
    {
        pImgCloudExactSync_.reset(new message_filters::Synchronizer<ImgCloudExactSync>(
          ImgCloudExactSync(10), imageSubsc_, cloudSubsc_));
        pImgCloudExactSync_->registerCallback(
          std::bind(&PointCloud2ImageNode::onSensorDataReceived, this, std::placeholders::_1, std::placeholders::_2));
    }
    else
    {
        pImgCloudApproxSync_.reset(new message_filters::Synchronizer<ImgCloudApproxSync>(
          ImgCloudApproxSync(syncQueueSize_), imageSubsc_, cloudSubsc_));
        pImgCloudApproxSync_->registerCallback(
          std::bind(&PointCloud2ImageNode::onSensorDataReceived, this, std::placeholders::_1, std::placeholders::_2));
    }

    pCalibSub_ = create_subscription<multisensor_calibration_interface::msg::CalibrationResult>(
      INPUT_CALIBRATION_TOPIC_NAME,
      1,
      std::bind(&PointCloud2ImageNode::onCalibrationReceived, this, std::placeholders::_1));

    return true;
}

//==================================================================================================
void PointCloud2ImageNode::onCalibrationReceived(const CalibrationResult_Message_T::ConstSharedPtr& ipCalibMsg)
{
    if (!ipCalibMsg->is_successful)
        return;

    tf2::Transform ref2LocalTransform, local2RefTransform;
    utils::cvtGeometryTransform2TfTransform(ipCalibMsg->sensor_extrinsics, local2RefTransform);
    ref2LocalTransform = local2RefTransform.inverse();

    if (!ipCalibMsg->base_frame_id.empty())
    {
        geometry_msgs::msg::TransformStamped t;
        try
        {
            t = tfBuffer_.lookupTransform(ipCalibMsg->base_frame_id, ipCalibMsg->ref_frame_id,
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

        ref2LocalTransform *= tmpTransform;
    }

    temporaryTransform_ = ref2LocalTransform;
}

//==================================================================================================
void PointCloud2ImageNode::onInit()
{
#ifdef DEBUG
    ROS_DEBUG("%s", __PRETTY_FUNCTION__);
#endif

    //--- read launch parameters
    isInitialized_ = readLaunchParameters();

    //--- initialize subscribers
    isInitialized_ &= initializeSubscribers();

    //--- initialized publishers
    isInitialized_ &= initializePublishers();

    //--- initialize camera info data
    isInitialized_ &= initializeCameraData(imageSubsc_.getTopic(), imageState_,
                                           cameraNamespace_, camera_);
}

//==================================================================================================
bool PointCloud2ImageNode::readLaunchParameters()
{
    //--- read launch parameters
    std::string imageStateStr;
    std::vector<double> tmpTransformCoeffs;
    imageStateStr    = this->declare_parameter<std::string>("image_state", DEFAULT_IMG_STATE_STR);
    cameraNamespace_ = this->declare_parameter<std::string>("camera_namespace", std::string(""));
    minDepth_        = this->declare_parameter<float>("min_depth", DEFAULT_MIN_DEPTH);
    maxDepth_        = this->declare_parameter<float>("max_depth", DEFAULT_MAX_DEPTH);
    syncQueueSize_   = this->declare_parameter<int>("sync_queue_size", DEFAULT_SYNC_QUEUE_SIZE);
    useExactSync_    = this->declare_parameter<bool>("use_exact_sync", false);
    tmpTransformCoeffs =
      this->declare_parameter<std::vector<double>>("temp_transform", std::vector<double>{});
    if (!tmpTransformCoeffs.empty())
    {
        useTemporaryTransform_ = true;
    }

    //--- sanity check for params
    if (minDepth_ <= 0)
    {
        RCLCPP_WARN(this->get_logger(), "[%s] (min_depth <= 0). Setting min. depth to default: %f ",
                    this->get_name(), DEFAULT_MIN_DEPTH);
        minDepth_ = DEFAULT_MIN_DEPTH;
    }
    if (maxDepth_ <= 0)
    {
        RCLCPP_WARN(this->get_logger(), "[%s] (max_depth <= 0). Setting max. depth to default: %f ",
                    this->get_name(), DEFAULT_MAX_DEPTH);
        maxDepth_ = DEFAULT_MAX_DEPTH;
    }
    if (minDepth_ >= maxDepth_)
    {

        RCLCPP_WARN(this->get_logger(), "[%s] (min_depth >= max_depth). Setting min. and max depth to default: [%f, %f]",
                    this->get_name(), DEFAULT_MIN_DEPTH, DEFAULT_MAX_DEPTH);
        minDepth_ = DEFAULT_MIN_DEPTH;
        maxDepth_ = DEFAULT_MAX_DEPTH;
    }
    if (syncQueueSize_ <= 0)
    {
        RCLCPP_WARN(this->get_logger(), "[%s] (sync_queue_size <= 0). Setting synchronization queue size to default: %i ",
                    this->get_name(), DEFAULT_SYNC_QUEUE_SIZE);
        syncQueueSize_ = DEFAULT_SYNC_QUEUE_SIZE;
    }

    //--- get image state from list
    auto stateFindItr = STR_2_IMG_STATE.find(imageStateStr);
    if (stateFindItr != STR_2_IMG_STATE.end())
        imageState_ = stateFindItr->second;

    //--- get temporary transform from tmpTransformStr
    if (useTemporaryTransform_)
    {
        //--- check if temp_transform is of correct size
        if (tmpTransformCoeffs.size() == 7)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[%s] Using temporary transform ((Trans) XYZ | (Rot) "
                        "XYZW): %f %f %f | "
                        "%f %f %f %f",
                        this->get_name(), tmpTransformCoeffs[0],
                        tmpTransformCoeffs[1], tmpTransformCoeffs[2],
                        tmpTransformCoeffs[3], tmpTransformCoeffs[4],
                        tmpTransformCoeffs[5], tmpTransformCoeffs[6]);

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
                        "[%s] Wrong format of temp_transform. Please provide as "
                        "\"X Y Z QX QY QZ QW\". "
                        "Extracting transform from frame IDs instead.",
                        this->get_name());
            useTemporaryTransform_ = false;
        }
    }

    return true;
}

//==================================================================================================
void PointCloud2ImageNode::projectCloudPointIntoDepthMap(const InputPointType& iCloudPnt,
                                                         const lib3d::Camera& iCamera,
                                                         lib3d::DepthMap& ioDepthMap) const
{
    //--- transform world lidar point to local point
    cv::Vec4d _worldPnt(iCloudPnt.x, iCloudPnt.y, iCloudPnt.z, 1.0);
    cv::Vec4d _localPnt = iCamera.extrinsics.getRTMatrix() * _worldPnt;

    //--- project local point to pixel and get depth from z coordinate
    cv::Point2i px = iCamera.projectLocal2Pixel(static_cast<float>(_localPnt(0)),
                                                static_cast<float>(_localPnt(1)),
                                                static_cast<float>(_localPnt(2)));
    float pntDepth = static_cast<float>(_localPnt(2));

    cv::Size depthMapSize = ioDepthMap.size();

    //--- check if pixel is within image and depth is positive
    if (px.x >= 0 && px.x < depthMapSize.width &&
        px.y >= 0 && px.y < depthMapSize.height)
    {
        //--- dilate pixel to size of 3x3
        for (int m = -PROJECTED_PIXEL_RADIUS; m <= PROJECTED_PIXEL_RADIUS; ++m)
        {
            for (int n = -PROJECTED_PIXEL_RADIUS; n <= PROJECTED_PIXEL_RADIUS; ++n)
            {
                //--- check for image boundaries
                cv::Point2i neighborPx = px + cv::Point2i(n, m);
                neighborPx.x           = std::min(std::max(0, neighborPx.x),
                                                  depthMapSize.width - 1);
                neighborPx.y           = std::min(std::max(0, neighborPx.y),
                                                  depthMapSize.height - 1);

                //--- if depth map already holds value at specified pixel, only assign depth if
                //--- new depth is smaller
                float currentDepthVal = ioDepthMap.valueAt(px);
                if (currentDepthVal == 0 || currentDepthVal > pntDepth)
                    ioDepthMap.valueAt(neighborPx) = pntDepth;
            }
        }
    }
}

//==================================================================================================
std::vector<float> PointCloud2ImageNode::splitStringToFloat(const std::string& iStr,
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
            RCLCPP_WARN(this->get_logger(), "[%s] %s: %s is not a number!",
                        this->get_name(), __PRETTY_FUNCTION__, s.c_str());
        }
    }

    return fltList;
}
} // namespace visualizers
} // namespace multisensor_calibration

RCLCPP_COMPONENTS_REGISTER_NODE(multisensor_calibration::visualizers::PointCloud2ImageNode)
