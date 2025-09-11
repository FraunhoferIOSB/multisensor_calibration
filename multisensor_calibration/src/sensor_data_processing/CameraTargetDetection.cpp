/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/sensor_data_processing/CameraTargetDetection.h"

// Std
#include <cmath>
#include <future>
// ROS
#include <cv_bridge/cv_bridge.hpp>

// OpenCV
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// multisensor_calibration
#include "multisensor_calibration/common/utils.hpp"

namespace multisensor_calibration
{

//==================================================================================================
CameraTargetDetection::CameraTargetDetection(const std::string& nodeName,
                                             const rclcpp::NodeOptions& options) :
  rclcpp::Node(nodeName, options),
  isInitialized_(false),
  cameraNamespace_(""),
  imageName_(""),
  imageFrameId_(""),
  pCamDataProcessor_(nullptr),
  imageState_(STR_2_IMG_STATE.at(DEFAULT_IMG_STATE_STR)),
  captureCalibrationTarget_(false)
{
    //--- setup launch and dynamic parameters
    setupLaunchParameters(this);

    //--- read launch parameters
    isInitialized_ = readLaunchParameters(this);

    //--- initialize camera data processor
    pCamDataProcessor_.reset(new CameraDataProcessor(
      this->get_name(), "", calibTargetFilePath_));
    if (pCamDataProcessor_)
    {
        pCamDataProcessor_->setImageState(imageState_);
        pCamDataProcessor_->initializePublishers(this);
    }
    else
    {
        isInitialized_ = false;
    }

    //--- initialize subscribers
    isInitialized_ &= initializeSubscribers(this);

    //--- initialize services
    isInitialized_ &= initializeServices(this);
}

//==================================================================================================
CameraTargetDetection::CameraTargetDetection(
  const rclcpp::NodeOptions& options) :
  CameraTargetDetection("camera_target_detection",
                        options)
{
}

//==================================================================================================
CameraTargetDetection::~CameraTargetDetection()
{
}

//==================================================================================================
bool CameraTargetDetection::initializeServices(rclcpp::Node* ipNode)
{
    //--- advertise services

    pCameraIntrSrv_ = ipNode->create_service<interf::srv::CameraIntrinsics>(
      "~/" + REQUEST_CAM_INTRINSICS_SRV_NAME,
      std::bind(&CameraTargetDetection::onRequestCameraIntrinsics, this,
                std::placeholders::_1, std::placeholders::_2));

    pCaptureSrv_ = ipNode->create_service<interf::srv::CaptureCalibTarget>(
      "~/" + CAPTURE_TARGET_SRV_NAME,
      std::bind(&CameraTargetDetection::onRequestTargetCapture, this,
                std::placeholders::_1, std::placeholders::_2));

    pStateSrv_ = ipNode->create_service<interf::srv::DataProcessorState>(
      "~/" + REQUEST_STATE_SRV_NAME,
      std::bind(&CameraTargetDetection::onRequestState, this,
                std::placeholders::_1, std::placeholders::_2));

    return true;
}

//==================================================================================================
bool CameraTargetDetection::initializeSubscribers(rclcpp::Node* ipNode)
{
    //--- subscribe to topics with name constructed from camera Namespace and imageName

    image_transport::ImageTransport imgTransp(ipNode->shared_from_this());
    imageSubsc_ = imgTransp.subscribe(
      cameraNamespace_ + "/" + imageName_, 1,
      std::bind(&CameraTargetDetection::onImageReceived,
                this, std::placeholders::_1));

    //--- left camera info message
    pCamInfoSubsc_ = ipNode->create_subscription<sensor_msgs::msg::CameraInfo>(
      cameraNamespace_ + "/camera_info", 1,
      std::bind(&CameraTargetDetection::onCameraInfoReceived, this,
                std::placeholders::_1));

    return true;
}

//==================================================================================================
void CameraTargetDetection::onImageReceived(
  const InputImage_Message_T::ConstSharedPtr& ipImgMsg)
{

#ifdef DEBUG_BUILD
    RCLCPP_INFO(this->get_logger(), "Message timestamp: %i-%i", __PRETTY_FUNCTION__,
                ipImgMsg->header.stamp.sec, ipImgMsg->header.stamp.nsec);
#endif

    //--- check if node is initialized
    if (!isInitialized_ || pCamDataProcessor_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "Node is not initialized.");
        return;
    }
    //--- check if camera intrinsics are set
    if (!pCamDataProcessor_->isCameraIntrinsicsSet())
    {
        RCLCPP_ERROR(this->get_logger(), "Camera intrinsics are not set.");
        return;
    }

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(imageCallbackMutex_);

    //--- get frame id from header
    imageFrameId_ = ipImgMsg->header.frame_id;

    //--- convert input messages to data
    bool isConversionSuccessful = true;

    // camera image
    cv::Mat cameraImage;
    isConversionSuccessful &= pCamDataProcessor_->getSensorDataFromMsg(ipImgMsg, cameraImage);

    if (!isConversionSuccessful)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Something went wrong in getting the sensor data from the input messages.");
        return;
    }

    //--- process data
    CameraDataProcessor::EProcessingLevel procLevel =
      (captureCalibrationTarget_)
        ? CameraDataProcessor::TARGET_DETECTION
        : CameraDataProcessor::PREVIEW;

    //--- process data asynchronously
    std::future<CameraDataProcessor::EProcessingResult> procFuture =
      std::async(&CameraDataProcessor::processData,
                 pCamDataProcessor_,
                 cameraImage,
                 procLevel);
    CameraDataProcessor::EProcessingResult procResult = procFuture.get();

    if (procResult == CameraDataProcessor::SUCCESS)
    {
        //--- publish preview data, i.e. annotated camera image
        pCamDataProcessor_->publishPreview(ipImgMsg->header);

        if (procLevel == CameraDataProcessor::TARGET_DETECTION)
        {

            //--- if calibration target cloud is empty, return and try again
            if (pCamDataProcessor_->getLastCalibrationTargetCloudPtr()->empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Calibration target cloud is empty.");
                return;
            }

            //--- publish result data of target detection
            pCamDataProcessor_->publishLastTargetDetection(ipImgMsg->header);
        }
    }

    //--- if data processor is not pending for more data, set capturing flag to false
    if (procResult != CameraDataProcessor::PENDING)
        captureCalibrationTarget_ = false;
}
//==================================================================================================
void CameraTargetDetection::onCameraInfoReceived(
  const sensor_msgs::msg::CameraInfo::SharedPtr pCamInfo)
{
    if (!pCamDataProcessor_)
        return;

    lib3d::Intrinsics cameraIntr;
    utils::setCameraIntrinsicsFromCameraInfo(*pCamInfo.get(),
                                             cameraIntr,
                                             imageState_);
    pCamDataProcessor_->setCameraIntrinsics(cameraIntr);
}

//==================================================================================================
bool CameraTargetDetection::onRequestCameraIntrinsics(
  const std::shared_ptr<interf::srv::CameraIntrinsics::Request> ipReq,
  std::shared_ptr<interf::srv::CameraIntrinsics::Response> opRes)
{
#ifdef DEBUG_BUILD
    RCLCPP_INFO(this->get_logger(), "%s", __PRETTY_FUNCTION__);
#endif

    UNUSED_VAR(ipReq);

    if (!isInitialized_ || pCamDataProcessor_ == nullptr)
        return false;
    if (!pCamDataProcessor_->isCameraIntrinsicsSet())
    {
        return false;
    }

    lib3d::Intrinsics cameraIntr = pCamDataProcessor_->getCameraIntrinsics();

    //--- image size
    opRes->intrinsics.width  = cameraIntr.getWidth();
    opRes->intrinsics.height = cameraIntr.getHeight();

    //--- K
    cv::Mat K = cv::Mat(cameraIntr.getK_as3x3());
    std::memcpy(opRes->intrinsics.k.data(), K.data, 9 * K.elemSize1());

    //--- distortion
    int nDistParams     = cameraIntr.getDistortionCoeffs().total();
    opRes->intrinsics.d = std::vector<double>(nDistParams);
    std::memcpy(opRes->intrinsics.d.data(),
                cameraIntr.getDistortionCoeffs().data,
                nDistParams * cameraIntr.getDistortionCoeffs().elemSize1());

    //--- P
    cv::Mat P = cv::Mat(cameraIntr.getK_as3x4());
    std::memcpy(opRes->intrinsics.p.data(), P.data, 12 * K.elemSize1());

    return true;
}

//==================================================================================================
bool CameraTargetDetection::onRequestTargetCapture(
  const std::shared_ptr<interf::srv::CaptureCalibTarget::Request> ipReq,
  std::shared_ptr<interf::srv::CaptureCalibTarget::Response> opRes)
{
    UNUSED_VAR(ipReq);

#ifdef DEBUG_BUILD
    RCLCPP_INFO(this->get_logger(), "%s", __PRETTY_FUNCTION__);
#endif

    if (!isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(imageCallbackMutex_);

    //--- store request
    captureCalibrationTarget_ = true;

    //--- write response
    opRes->is_accepted = true;
    opRes->msg         = "Start looking for calibration target!";

    RCLCPP_INFO(this->get_logger(), "%s", opRes->msg.c_str());

    return true;
}

//==================================================================================================
bool CameraTargetDetection::onRequestState(
  const std::shared_ptr<interf::srv::DataProcessorState::Request> ipReq,
  std::shared_ptr<interf::srv::DataProcessorState::Response> opRes)
{
#ifdef DEBUG_BUILD
    RCLCPP_INFO(this->get_logger(), "%s", __PRETTY_FUNCTION__);
#endif

    UNUSED_VAR(ipReq);

    //--- store response
    //--- is initialized if internal flag is true and imageFrameId is not empty
    opRes->is_initialized = (isInitialized_ && !imageFrameId_.empty());
    opRes->frame_id       = imageFrameId_;

    return true;
}

//==================================================================================================
void CameraTargetDetection::setupLaunchParameters(rclcpp::Node* ipNode) const
{
    //--- camera namespace
    auto cameraNameDesc = rcl_interfaces::msg::ParameterDescriptor{};
    cameraNameDesc.description =
      "Namespace of the camera.\n"
      "Default: \"/camera\"";
    cameraNameDesc.read_only = true;
    ipNode->declare_parameter<std::string>("camera", "/camera", cameraNameDesc);

    //--- image name
    auto imageNameDesc = rcl_interfaces::msg::ParameterDescriptor{};
    imageNameDesc.description =
      "Name of the image topic within the camera namespace.\n"
      "Default: \"image_color\"";
    imageNameDesc.read_only = true;
    ipNode->declare_parameter<std::string>("image", "image_color", imageNameDesc);

    //--- image state
    auto imageStateDesc = rcl_interfaces::msg::ParameterDescriptor{};
    imageStateDesc.description =
      "State of the camera images used.\n"
      "Default: \"DISTORTED\"";
    imageStateDesc.read_only = true;
    ipNode->declare_parameter<std::string>("image_state", DEFAULT_IMG_STATE_STR,
                                           imageStateDesc);

    //--- path to target configuration file
    auto targetConfigDesc = rcl_interfaces::msg::ParameterDescriptor{};
    targetConfigDesc.description =
      "Path to the file holding the configuration of the calibration target. "
      "E.g. \"$(find multisensor_calibration)/config/TargetWithCirclesAndAruco.yaml\"\n"
      "Default: \"\"";
    targetConfigDesc.read_only = true;
    ipNode->declare_parameter<std::string>("target_config_file", "", targetConfigDesc);
}

//==================================================================================================
bool CameraTargetDetection::readLaunchParameters(const rclcpp::Node* ipNode)
{
    // namespace of the camera to which to subscribe
    cameraNamespace_ = ipNode->get_parameter("camera").as_string();
    if (cameraNamespace_.front() != '/' && !cameraNamespace_.empty())
        cameraNamespace_ = "/" + cameraNamespace_; // prepend slash to namespace, if need be
    while (cameraNamespace_.back() == '/' && !cameraNamespace_.empty())
        cameraNamespace_.pop_back(); // remove slash(s) from back of namespace, if need be

    // name of image (within cameraNamespace) to subscribe to
    imageName_ = ipNode->get_parameter("image").as_string();
    while (imageName_.front() == '/' && !imageName_.empty())
        imageName_ = imageName_.substr(1); // remove slash(s) from from of imageName, if need be
    while (imageName_.back() == '/' && !imageName_.empty())
        imageName_.pop_back(); // remove slash(s) from from of imageName, if need be

    // image state
    std::string imageStateStr = ipNode->get_parameter("image_state").as_string();
    auto findItr              = STR_2_IMG_STATE.find(imageStateStr);
    if (findItr != STR_2_IMG_STATE.end())
        imageState_ = findItr->second;

    // path to target configuration file
    std::string targetFileStr = ipNode->get_parameter("target_config_file").as_string();
    if (targetFileStr.empty() || !fs::exists(targetFileStr))
    {
        RCLCPP_FATAL(this->get_logger(),
                     "Target configuration file path is empty or does not consist: %s",
                     targetFileStr.c_str());
        return false;
    }
    calibTargetFilePath_ = fs::absolute(targetFileStr);

    return true;
}

} // namespace multisensor_calibration

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(multisensor_calibration::CameraTargetDetection)