/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/sensor_data_processing/LidarTargetDetection.h"

// Std
#include <cmath>
#include <future>

// ROS
#include <pcl_conversions/pcl_conversions.h>

namespace multisensor_calibration
{

//==================================================================================================
LidarTargetDetection::LidarTargetDetection(const std::string& nodeName,
                                           const rclcpp::NodeOptions& options) :
  rclcpp::Node(nodeName, options),
  isInitialized_(false),
  cloudFrameId_(""),
  pLidarDataProcessor_(nullptr),
  captureCalibrationTarget_(false)
{
    //--- setup launch and dynamic parameters
    setupLaunchParameters(this);
    setupDynamicParameters(this);

    //--- register parameter change callback
    pParameterCallbackHandle_ = add_on_set_parameters_callback(
      std::bind(&LidarTargetDetection::handleDynamicParameterChange, this,
                std::placeholders::_1));

    //--- read launch parameters
    isInitialized_ = readLaunchParameters(this);

    //--- initialize lidar data processor
    pLidarDataProcessor_.reset(
      new LidarDataProcessor(this->get_name(), "", calibTargetFilePath_));
    if (pLidarDataProcessor_ != nullptr)
    {
        pLidarDataProcessor_->initializePublishers(this);
        pLidarDataProcessor_->setParameters(lidarTargetDetectionParams_);
    }
    else
    {
        isInitialized_ = false;
        return;
    }

    //--- initialize subscribers
    isInitialized_ &= initializeSubscribers(this);

    //--- initialize services
    isInitialized_ &= initializeServices(this);
}

//==================================================================================================
LidarTargetDetection::LidarTargetDetection(
  const rclcpp::NodeOptions& options) :
  LidarTargetDetection("lidar_target_detection",
                       options)
{
}

//==================================================================================================
LidarTargetDetection::~LidarTargetDetection()
{
}

//==================================================================================================
bool LidarTargetDetection::initializeServices(rclcpp::Node* ipNode)
{
    //--- advertise services

    pCaptureSrv_ = ipNode->create_service<interf::srv::CaptureCalibTarget>(
      "~/" + CAPTURE_TARGET_SRV_NAME,
      std::bind(&LidarTargetDetection::onRequestTargetCapture, this,
                std::placeholders::_1, std::placeholders::_2));

    pStateSrv_ = ipNode->create_service<interf::srv::DataProcessorState>(
      "~/" + REQUEST_STATE_SRV_NAME,
      std::bind(&LidarTargetDetection::onRequestState, this,
                std::placeholders::_1, std::placeholders::_2));

    return true;
}

//==================================================================================================
bool LidarTargetDetection::initializeSubscribers(rclcpp::Node* ipNode)
{
    //--- subscribe to topics with name cloudTopic

    pCloudSubsc_ = ipNode->create_subscription<InputCloud_Message_T>(
      cloudTopicName_, 1,
      std::bind(&LidarTargetDetection::onCloudReceived, this,
                std::placeholders::_1));

    return true;
}

//==================================================================================================
void LidarTargetDetection::onCloudReceived(
  const InputCloud_Message_T::ConstSharedPtr& ipCloudMsg)
{

#ifdef DEBUG_BUILD
    RCLCPP_INFO(this->get_logger(), "[%s] Message timestamp: %i.%i", __PRETTY_FUNCTION__,
                ipCloudMsg->header.stamp.sec, ipCloudMsg->header.stamp.nanosec);
#endif

    //--- check if node is initialized
    if (!isInitialized_ || pLidarDataProcessor_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "Node is not initialized.");
        return;
    }

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(cloudCallbackMutex_);

    //--- get frame id from header
    cloudFrameId_ = ipCloudMsg->header.frame_id;

    //--- convert input messages to data
    bool isConversionSuccessful = true;

    // point cloud
    pcl::PointCloud<InputPointType> pointCloud;
    isConversionSuccessful &=
      pLidarDataProcessor_->getSensorDataFromMsg(ipCloudMsg, pointCloud);

    if (!isConversionSuccessful)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Something went wrong in getting the sensor data from the input messages.");
        return;
    }

    //--- process data
    LidarDataProcessor::EProcessingLevel procLevel =
      (captureCalibrationTarget_)
        ? LidarDataProcessor::TARGET_DETECTION
        : LidarDataProcessor::PREVIEW;

    //--- process data asynchronously
    std::future<LidarDataProcessor::EProcessingResult> procFuture =
      std::async(&LidarDataProcessor::processData,
                 pLidarDataProcessor_,
                 pointCloud,
                 procLevel);
    LidarDataProcessor::EProcessingResult procResult = procFuture.get();

    if (procResult == LidarDataProcessor::SUCCESS)
    {
        //--- publish regions of interest
        pLidarDataProcessor_->publishPreview(ipCloudMsg->header);

        if (procLevel == LidarDataProcessor::TARGET_DETECTION)
        {
            //--- if calibration target cloud is empty, return and try again
            if (pLidarDataProcessor_->getLastCalibrationTargetCloud()->empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Calibration target cloud is empty.");
                return;
            }

            //--- if marker corners cloud is empty, return
            if (pLidarDataProcessor_->getLastMarkerCornersCloud()->empty())
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Cloud holding 3D points of marker corners is empty.");
                return;
            }

            //--- publish result data of target detection
            pLidarDataProcessor_->publishLastTargetDetection(ipCloudMsg->header);
        }
    }

    //--- if data processor is not pending for more data, set capturing flag to false
    if (procResult != LidarDataProcessor::PENDING)
        captureCalibrationTarget_ = false;
}

//==================================================================================================
bool LidarTargetDetection::onRequestTargetCapture(
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
    std::lock_guard<std::mutex> guard(cloudCallbackMutex_);

    //--- store request
    captureCalibrationTarget_ = true;

    //--- write response
    opRes->is_accepted = true;
    opRes->msg         = "Start looking for calibration target!";

    RCLCPP_INFO(this->get_logger(), "%s", opRes->msg.c_str());

    return true;
}

//==================================================================================================
bool LidarTargetDetection::onRequestState(
  const std::shared_ptr<interf::srv::DataProcessorState::Request> ipReq,
  std::shared_ptr<interf::srv::DataProcessorState::Response> opRes)
{
#ifdef DEBUG_BUILD
    RCLCPP_INFO(this->get_logger(), "%s", __PRETTY_FUNCTION__);
#endif

    UNUSED_VAR(ipReq);

    //--- store response
    //--- is initialized if internal flag is true and imageFrameId is not empty
    opRes->is_initialized = (isInitialized_ && !cloudFrameId_.empty());
    opRes->frame_id       = cloudFrameId_;

    return true;
}

//==================================================================================================
void LidarTargetDetection::setupLaunchParameters(rclcpp::Node* ipNode) const
{
    //--- cloud topic name
    auto cloudTopicNameDesc = rcl_interfaces::msg::ParameterDescriptor{};
    cloudTopicNameDesc.description =
      "Topic name of the LiDAR cloud messages in which the target is to be detected.\n"
      "Default: \"/cloud\"";
    cloudTopicNameDesc.read_only = true;
    ipNode->declare_parameter<std::string>("cloud_topic_name",
                                           "/cloud",
                                           cloudTopicNameDesc);

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
void LidarTargetDetection::setupDynamicParameters(rclcpp::Node* ipNode) const
{
    lidarTargetDetectionParams_.declareDynamic(ipNode);
}

//==================================================================================================
bool LidarTargetDetection::readLaunchParameters(const rclcpp::Node* ipNode)
{
    // topic name of input cloud
    cloudTopicName_ = ipNode->get_parameter("cloud_topic_name").as_string();

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

//==================================================================================================
rcl_interfaces::msg::SetParametersResult LidarTargetDetection::
  handleDynamicParameterChange(const std::vector<rclcpp::Parameter>& iParameters)
{
    RCLCPP_DEBUG(this->get_logger(), "%s", __PRETTY_FUNCTION__);

    // Return value that holds information on success.
    rcl_interfaces::msg::SetParametersResult retVal;
    retVal.successful = true;

    //-- loop through all parameters and set information accordingly
    for (rclcpp::Parameter param : iParameters)
    {
        if (this->setDynamicParameter(param))
        {
            retVal.successful &= true;
            break;
        }
        else
        {
            retVal.successful &= false;
            retVal.reason = std::string(this->get_name()) + ": Parameter '" + param.get_name() +
                            "' was not found in parameter list.";
        }
    }

    return retVal;
}

//==================================================================================================
bool LidarTargetDetection::setDynamicParameter(const rclcpp::Parameter& iParameter)
{
    if (lidarTargetDetectionParams_.tryToSetParameter(iParameter))
    {
        pLidarDataProcessor_->setParameters(lidarTargetDetectionParams_);
        return true;
    }
    else
    {
        return false;
    }
}

} // namespace multisensor_calibration

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(multisensor_calibration::LidarTargetDetection)
