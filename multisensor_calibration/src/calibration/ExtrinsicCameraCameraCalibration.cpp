/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/calibration/ExtrinsicCameraCameraCalibration.h"

// Std
#include <functional>
#include <future>

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

// ROS
#include <memory>
#include <tf2/LinearMath/Transform.hpp>

// Qt
#include <QFile>

// small_gicp
#include <small_gicp/registration/registration_helper.hpp>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/common/utils.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <multisensor_calibration_interface/msg/calibration_result.hpp>

namespace multisensor_calibration
{

//==================================================================================================
ExtrinsicCameraCameraCalibration::ExtrinsicCameraCameraCalibration(
  const std::string& nodeName,
  const rclcpp::NodeOptions& options) :
  Extrinsic2d2dCalibrationBase<CameraDataProcessor, CameraDataProcessor>(
    STEREO_CAMERA_CALIBRATION),
  rclcpp::Node(nodeName, options),
  syncQueueSize_(DEFAULT_SYNC_QUEUE_SIZE),
  useExactSync_(false)
{
    CalibrationBase::init(this);
}

//==================================================================================================

ExtrinsicCameraCameraCalibration::ExtrinsicCameraCameraCalibration(const rclcpp::NodeOptions& options) :
  ExtrinsicCameraCameraCalibration(
    CALIB_TYPE_2_NODE_NAME.at(STEREO_CAMERA_CALIBRATION),
    options)
{
}

//==================================================================================================

ExtrinsicCameraCameraCalibration::~ExtrinsicCameraCameraCalibration()
{
    pImageImageApproxSync_.reset();
    pImageImageExactSync_.reset();
    pSrcDataProcessor_.reset();
    pRefDataProcessor_.reset();
}

//==================================================================================================

void ExtrinsicCameraCameraCalibration::calibrateLastObservation()
{

    /* @TODO */
    /* Nothing to be done here, the whole calibration is performed at the end. */
    calibrationItrCnt_++;
}

//==================================================================================================

bool ExtrinsicCameraCameraCalibration::finalizeCalibration()
{
    /* @TODO */
    /* @TODO: Remember to tranform end calibration to baseFrameId_ if present */

    return false;
}

//==================================================================================================

bool ExtrinsicCameraCameraCalibration::initializeDataProcessors()
{

    bool isSuccessful = true;

    // Lambda function to initialize pointer to CamerDataProcessor
    auto initializeCameraDataProcessor = [&](std::shared_ptr<CameraDataProcessor>& iopProcessor,
                                             const std::string& iSensorName, EImageState& imageState)
    {
        iopProcessor = std::make_shared<CameraDataProcessor>(
          logger_.get_name(), iSensorName, calibTargetFilePath_);
        if (iopProcessor)
        {
            iopProcessor->initializeServices(this);
            iopProcessor->initializePublishers(this);
            iopProcessor->setImageState(imageState);
        }
        else
        {
            isSuccessful = false;
        }
    };

    //--- initialize data processors
    initializeCameraDataProcessor(pSrcDataProcessor_, srcSensorName_, srcImageState_);
    initializeCameraDataProcessor(pRefDataProcessor_, refSensorName_, refImageState_);

    return isSuccessful;
}

//==================================================================================================
void ExtrinsicCameraCameraCalibration::
  onSrcCameraInfoReceived(const sensor_msgs::msg::CameraInfo::SharedPtr pCamInfo)
{
    if (srcCameraInfo_.width != pCamInfo->width)
        srcCameraInfo_ = *pCamInfo;
}

//==================================================================================================
void ExtrinsicCameraCameraCalibration::
  onRefCameraInfoReceived(const sensor_msgs::msg::CameraInfo::SharedPtr pCamInfo)
{
    if (refCameraInfo_.width != pCamInfo->width)
        refCameraInfo_ = *pCamInfo;
}

//==================================================================================================

bool ExtrinsicCameraCameraCalibration::initializeSubscribers(rclcpp::Node* ipNode)
{
    //--- Camera info
    pSrcCamInfoSubsc_ = ipNode->create_subscription<sensor_msgs::msg::CameraInfo>(
      srcCameraInfoTopic_, 1,
      std::bind(&ExtrinsicCameraCameraCalibration::onSrcCameraInfoReceived,
                this,
                std::placeholders::_1));
    pRefCamInfoSubsc_ = ipNode->create_subscription<sensor_msgs::msg::CameraInfo>(
      refCameraInfoTopic_, 1,
      std::bind(&ExtrinsicCameraCameraCalibration::onRefCameraInfoReceived,
                this,
                std::placeholders::_1));

    //--- subscribe to topics
    srcImageSubsc_.subscribe(ipNode, srcTopicName_, "raw");
    refImageSubsc_.subscribe(ipNode, refTopicName_, "raw");

    //--- initialize synchronizers
    if (useExactSync_)
    {
        pImageImageExactSync_ =
          std::make_shared<message_filters::Synchronizer<ImageImageExactSync>>(
            ImageImageExactSync(10), srcImageSubsc_, refImageSubsc_);
        pImageImageExactSync_->registerCallback(
          std::bind(&ExtrinsicCameraCameraCalibration::onSensorDataReceived, this,
                    std::placeholders::_1, std::placeholders::_2));
    }
    else
    {
        pImageImageApproxSync_ =
          std::make_shared<message_filters::Synchronizer<ImageImageApproxSync>>(
            ImageImageApproxSync(syncQueueSize_), srcImageSubsc_, refImageSubsc_);
        pImageImageApproxSync_->registerCallback(
          std::bind(&ExtrinsicCameraCameraCalibration::onSensorDataReceived, this,
                    std::placeholders::_1, std::placeholders::_2));
    }

    return true;
}

//==================================================================================================

bool ExtrinsicCameraCameraCalibration::initializeWorkspaceObjects()
{
    bool retVal = true;

    retVal &= CalibrationBase::initializeWorkspaceObjects();

    //--- initialize calibration workspace
    fs::path calibWsPath = robotWsPath_;
    calibWsPath /=
      std::string(srcSensorName_ + "_" + refSensorName_ + "_extrinsic_calibration");
    pCalibrationWs_ =
      std::make_shared<ExtrinsicCameraCameraCalibWorkspace>(calibWsPath, logger_);
    retVal &= (pCalibrationWs_ != nullptr);

    if (pCalibrationWs_)
    {
        //--- create backup
        retVal &= pCalibrationWs_->createBackup();
    }

    return retVal;
}

//==================================================================================================

bool ExtrinsicCameraCameraCalibration::initializeServices(rclcpp::Node* ipNode)
{
    if (!ExtrinsicCalibrationBase::initializeServices(ipNode))
        return false;

    /* @TODO */

    return true;
}
//==================================================================================================
bool ExtrinsicCameraCameraCalibration::onRequestRemoveObservation(
  const std::shared_ptr<interf::srv::RemoveLastObservation::Request> ipReq,
  std::shared_ptr<interf::srv::RemoveLastObservation::Response> opRes)
{
    if (ExtrinsicCalibrationBase<CameraDataProcessor, CameraDataProcessor>::onRequestRemoveObservation(ipReq, opRes))
        sensorExtrinsics_.pop_back();
    else
        return false;

    return true;
}

//==================================================================================================
bool ExtrinsicCameraCameraCalibration::initializeCameraIntrinsics(
  CameraDataProcessor* iopCamProcessor,
  sensor_msgs::msg::CameraInfo& cameraInfo,
  EImageState imageState,
  std::string cameraInfoTopic)
{
    if (cameraInfo.width != 0)
    {
        lib3d::Intrinsics cameraIntr;
        utils::setCameraIntrinsicsFromCameraInfo(cameraInfo,
                                                 cameraIntr,
                                                 imageState);
        iopCamProcessor->setCameraIntrinsics(cameraIntr);

        return true;
    }
    else
    {
        RCLCPP_ERROR(CalibrationBase::logger_,
                     "Wait for message of 'camera_info' topic has timed out. "
                     "\n'camera_info' topic: %s"
                     "\nWaiting for next data package.",
                     cameraInfoTopic.c_str());
        return false;
    }
}
//==================================================================================================

void ExtrinsicCameraCameraCalibration::onSensorDataReceived(
  const InputImage_Message_T::ConstSharedPtr& ipSrcImgMsg,
  const InputImage_Message_T::ConstSharedPtr& ipRefImgMsg)
{
    //--- check if node is initialized
    if (!isInitialized_)
    {
        RCLCPP_ERROR(logger_, "Node is not initialized.");
        return;
    }
    if (pSrcDataProcessor_ == nullptr)
    {
        RCLCPP_ERROR(logger_, "Camera data processor is not initialized.");
        return;
    }
    if (pRefDataProcessor_ == nullptr)
    {
        RCLCPP_ERROR(logger_, "Lidar data processor is not initialized.");
        return;
    }

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(dataProcessingMutex_);

    //--- convert input messages to data
    bool isConversionSuccessful = true;

    // camera image
    cv::Mat srcCameraImage;
    isConversionSuccessful &= pSrcDataProcessor_->getSensorDataFromMsg(ipSrcImgMsg, srcCameraImage);

    cv::Mat refCameraImage;
    isConversionSuccessful &= pSrcDataProcessor_->getSensorDataFromMsg(ipRefImgMsg, refCameraImage);

    if (!isConversionSuccessful)
    {
        RCLCPP_ERROR(logger_,
                     "Something went wrong in getting the sensor data from the input messages.");
        return;
    }

    //--- camera intrinsics is not set to camera data processor,
    //--- wait for camera_info message and set intrinsics
    if (!pSrcDataProcessor_->isCameraIntrinsicsSet())
    {
        if (!initializeCameraIntrinsics(pSrcDataProcessor_.get(), srcCameraInfo_, srcImageState_, srcCameraInfoTopic_))
            return;
    }
    if (!pRefDataProcessor_->isCameraIntrinsicsSet())
    {
        if (!initializeCameraIntrinsics(pRefDataProcessor_.get(), refCameraInfo_, refImageState_, refCameraInfoTopic_))
            return;
    }

    //--- set frame ids and initialize sensor extrinsics if applicable
    if (srcFrameId_ != ipSrcImgMsg->header.frame_id ||
        refFrameId_ != ipRefImgMsg->header.frame_id)
    {
        srcFrameId_ = ipSrcImgMsg->header.frame_id;
        refFrameId_ = ipRefImgMsg->header.frame_id;

        // //--- if base frame id is not empty and unequal to refCloudFrameId use baseFrameID as
        // //--- reference frame id.
        // std::string tmpRefFrameId = refFrameId_;
        // if (!baseFrameId_.empty() && baseFrameId_ != refFrameId_)
        // {
        //     tmpRefFrameId = baseFrameId_;

        //     if (tfBuffer_->_frameExists(baseFrameId_))
        //     {
        //         /* @TODO */
        //     }
        //     else
        //     {
        //         RCLCPP_WARN(logger_,
        //                     "Base Frame '%s' does not exists! "
        //                     "Removing base frame and calibrating relative to reference cloud.",
        //                     baseFrameId_.c_str());
        //         baseFrameId_ = "";
        //     }
        // }

        // //--- set sensor extrinsics from either cloud or base frame id and apply frustum culling
        // if (useTfTreeAsInitialGuess_ &&
        //     setSensorExtrinsicsFromFrameIds(srcFrameId_, tmpRefFrameId))
        // {
        //     /* @TODO */
        // }
        // else
        // {
        //     /* @TODO */
        // }
    }

    /* @TODO */

    // Level at which to do the processing
    CameraDataProcessor::EProcessingLevel procLevel = (captureCalibrationTarget_)
                                                        ? CameraDataProcessor::TARGET_DETECTION
                                                        : CameraDataProcessor::PREVIEW;

    //--- process camera data asynchronously
    std::future<CameraDataProcessor::EProcessingResult> camProcFuture =
      std::async(&CameraDataProcessor::processData,
                 pSrcDataProcessor_,
                 srcCameraImage,
                 procLevel);

    //--- process lidar data asynchronously
    std::future<CameraDataProcessor::EProcessingResult> lidarProcFuture =
      std::async(&CameraDataProcessor::processData,
                 pRefDataProcessor_,
                 refCameraImage,
                 static_cast<CameraDataProcessor::EProcessingLevel>(procLevel));

    //--- wait for processing to return
    CameraDataProcessor::EProcessingResult srcCamProcResult   = camProcFuture.get();
    CameraDataProcessor::EProcessingResult refCamProcResult = lidarProcFuture.get();

    //--- if processing level is set to preview, publish preview from each sensor if successful
    if (procLevel == CameraDataProcessor::PREVIEW)
    {
        if (srcCamProcResult == CameraDataProcessor::SUCCESS)
            pSrcDataProcessor_->publishPreview(ipSrcImgMsg->header);
        if (refCamProcResult == CameraDataProcessor::SUCCESS)
            pRefDataProcessor_->publishPreview(ipRefImgMsg->header);
    }
    //--- else if, processing level is target_detection,
    //--- calibrate only if processing for both sensors is successful
    else if (procLevel == CameraDataProcessor::TARGET_DETECTION)
    {
        if (srcCamProcResult == CameraDataProcessor::SUCCESS &&
            refCamProcResult == CameraDataProcessor::SUCCESS)
        {
            //--- publish detections
            pSrcDataProcessor_->publishLastTargetDetection(ipSrcImgMsg->header);
            pRefDataProcessor_->publishLastTargetDetection(ipRefImgMsg->header);

            //--- do calibration
            calibrateLastObservation();
        }
        else
        {
            if (srcCamProcResult != CameraDataProcessor::SUCCESS &&
                refCamProcResult == CameraDataProcessor::SUCCESS)
                pRefDataProcessor_->removeCalibIteration(calibrationItrCnt_);

            if (srcCamProcResult == CameraDataProcessor::SUCCESS &&
                refCamProcResult != CameraDataProcessor::SUCCESS)
                pSrcDataProcessor_->removeCalibIteration(calibrationItrCnt_);

            interf::msg::CalibrationResult calibResultMsg;
            calibResultMsg.is_successful = false;
            pCalibResultPub_->publish(calibResultMsg);
        }
    }

    //--- if this point is reached, the target detection was not successful.
    //--- thus, if data processor is not pending for more data, set capturing flag to false.
    if (srcCamProcResult != CameraDataProcessor::PENDING &&
        refCamProcResult != CameraDataProcessor::PENDING)
    {
        captureCalibrationTarget_ = false;
    }
    return;
}

//==================================================================================================
bool ExtrinsicCameraCameraCalibration::saveCalibrationSettingsToWorkspace()
{
    if (!ExtrinsicCalibrationBase::saveCalibrationSettingsToWorkspace())
        return false;

    QSettings* pCalibSettings = pCalibrationWs_->settingsPtr();
    if (!pCalibSettings)
        return false;

    pCalibSettings->setValue("source_camera/sensor_name",
                             QString::fromStdString(srcSensorName_));

    pCalibSettings->setValue("source_camera/image_topic",
                             QString::fromStdString(srcTopicName_));

    pCalibSettings->setValue("source_camera/info_topic",
                             QString::fromStdString(srcCameraInfoTopic_));

    pCalibSettings->setValue("reference_camera/sensor_name",
                             QString::fromStdString(refSensorName_));

    pCalibSettings->setValue("reference_camera/image_topic",
                             QString::fromStdString(refTopicName_));

    pCalibSettings->setValue("reference_camera/info_topic",
                             QString::fromStdString(refCameraInfoTopic_));

    pCalibSettings->setValue("misc/sync_queue_size",
                             QVariant::fromValue(syncQueueSize_));

    pCalibSettings->setValue("misc/use_exact_sync",
                             QVariant::fromValue(useExactSync_));

    pCalibSettings->setValue("misc/intrinsic_calibration",
                             QVariant::fromValue(useExactSync_));

    pCalibSettings->sync();

    return true;
}

//==================================================================================================

void ExtrinsicCameraCameraCalibration::setupLaunchParameters(rclcpp::Node* ipNode) const
{
    ExtrinsicCalibrationBase::setupLaunchParameters(ipNode);

    /* ---- SRC Camera ---- */
    auto srcSensorNameDesc = rcl_interfaces::msg::ParameterDescriptor{};
    srcSensorNameDesc.description =
      "Name of the camera sensor that is to be calibrated.\n"
      "Default: \"camera\"";
    srcSensorNameDesc.read_only = true;
    ipNode->declare_parameter<std::string>("src_camera_sensor_name", DEFAULT_CAMERA_SENSOR_NAME,
                                           srcSensorNameDesc);

    auto srcSensorTopicDesc = rcl_interfaces::msg::ParameterDescriptor{};
    srcSensorTopicDesc.description =
      "Topic name of the corresponding camera images.\n"
      "Default: \"/camera/image_color\"";
    srcSensorTopicDesc.read_only = true;
    ipNode->declare_parameter<std::string>("src_camera_image_topic", DEFAULT_CAMERA_IMAGE_TOPIC,
                                           srcSensorTopicDesc);

    auto srcCameraInfoTopicDesc = rcl_interfaces::msg::ParameterDescriptor{};
    srcCameraInfoTopicDesc.description =
      "Name of the camera info topic. If this parameter is left empty the camera info topic name is "
      "constructed from the specified ```camera_image_topic```.\n "
      "Default: \"\"";
    srcCameraInfoTopicDesc.read_only = true;
    ipNode->declare_parameter<std::string>("src_camera_info_topic", "",
                                           srcCameraInfoTopicDesc);

    auto srcImageStateDesc = rcl_interfaces::msg::ParameterDescriptor{};
    srcImageStateDesc.description =
      "State of the camera images used.\n"
      "Default: \"DISTORTED\"";
    srcImageStateDesc.read_only = true;
    ipNode->declare_parameter<std::string>("src_image_state", DEFAULT_IMG_STATE_STR,
                                           srcImageStateDesc);

    /* ---- REF Camera ---- */
    auto refSensorNameDesc = rcl_interfaces::msg::ParameterDescriptor{};
    refSensorNameDesc.description =
      "Name of the camera sensor that is to be calibrated.\n"
      "Default: \"camera\"";
    refSensorNameDesc.read_only = true;
    ipNode->declare_parameter<std::string>("ref_camera_sensor_name", DEFAULT_CAMERA_SENSOR_NAME,
                                           refSensorNameDesc);

    auto refSensorTopicDesc = rcl_interfaces::msg::ParameterDescriptor{};
    refSensorTopicDesc.description =
      "Topic name of the corresponding camera images.\n"
      "Default: \"/camera/image_color\"";
    refSensorTopicDesc.read_only = true;
    ipNode->declare_parameter<std::string>("ref_camera_image_topic", DEFAULT_CAMERA_IMAGE_TOPIC,
                                           refSensorTopicDesc);

    auto refCameraInfoTopicDesc = rcl_interfaces::msg::ParameterDescriptor{};
    refCameraInfoTopicDesc.description =
      "Name of the camera info topic. If this parameter is left empty the camera info topic name is "
      "constructed from the specified ```camera_image_topic```.\n "
      "Default: \"\"";
    refCameraInfoTopicDesc.read_only = true;
    ipNode->declare_parameter<std::string>("ref_camera_info_topic", "",
                                           refCameraInfoTopicDesc);

    auto refImageStateDesc = rcl_interfaces::msg::ParameterDescriptor{};
    refImageStateDesc.description =
      "State of the camera images used.\n"
      "Default: \"DISTORTED\"";
    refImageStateDesc.read_only = true;
    ipNode->declare_parameter<std::string>("ref_image_state", DEFAULT_IMG_STATE_STR,
                                           refImageStateDesc);

    //--- sync queue
    auto syncQueueDesc = rcl_interfaces::msg::ParameterDescriptor{};
    syncQueueDesc.description =
      "Queue size used for the synchronization between the messages of the camera images\n "
      "Default: 100";
    syncQueueDesc.read_only = true;
    ipNode->declare_parameter<int>("sync_queue_size", DEFAULT_SYNC_QUEUE_SIZE,
                                   syncQueueDesc);

    //--- exact sync
    auto exactSyncDesc = rcl_interfaces::msg::ParameterDescriptor{};
    exactSyncDesc.description =
      "Set to true if an exact time synchronization between the camera image messages\n"
      "Default: false";
    syncQueueDesc.read_only = true;
    ipNode->declare_parameter<bool>("use_exact_sync", false, exactSyncDesc);
}

//==================================================================================================

void ExtrinsicCameraCameraCalibration::setupDynamicParameters(rclcpp::Node* ipNode) const
{
    registrationParams_.declareDynamic(ipNode);
}
//==================================================================================================

bool ExtrinsicCameraCameraCalibration::readLaunchParameters(const rclcpp::Node* ipNode)
{
    if (!ExtrinsicCalibrationBase::readLaunchParameters(ipNode))
        return false;

    srcSensorName_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "src_camera_sensor_name", DEFAULT_CAMERA_SENSOR_NAME);

    srcTopicName_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "src_camera_image_topic", DEFAULT_CAMERA_IMAGE_TOPIC);

    refSensorName_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "ref_camera_sensor_name", DEFAULT_CAMERA_SENSOR_NAME);

    refTopicName_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "ref_camera_image_topic", DEFAULT_CAMERA_IMAGE_TOPIC);

    //--- camera_info_topic
    srcCameraInfoTopic_ = ipNode->get_parameter("src_camera_info_topic").as_string();
    if (srcCameraInfoTopic_.empty())
    {
        srcCameraInfoTopic_ =
          srcTopicName_.substr(0, srcTopicName_.find_last_of('/')) + "/camera_info";
    }

    refCameraInfoTopic_ = ipNode->get_parameter("ref_camera_info_topic").as_string();
    if (refCameraInfoTopic_.empty())
    {
        refCameraInfoTopic_ =
          srcTopicName_.substr(0, srcTopicName_.find_last_of('/')) + "/camera_info";
    }

    //--- image state
    std::string srcImageState_ =
      CalibrationBase::readStringLaunchParameter(ipNode, "src_image_state",
                                                 DEFAULT_IMG_STATE_STR);
    auto findItr = STR_2_IMG_STATE.find(srcImageState_);
    if (findItr != STR_2_IMG_STATE.end())
        srcImageState_ = std::to_string(findItr->second);
    else
        RCLCPP_WARN(CalibrationBase::logger_, "String passed to 'src_image_state' is not valid. "
                                              "\nSetting 'src_image_state' to default: %s",
                    DEFAULT_IMG_STATE_STR.c_str());

    std::string refImageState_ =
      CalibrationBase::readStringLaunchParameter(ipNode, "ref_image_state",
                                                 DEFAULT_IMG_STATE_STR);
    findItr = STR_2_IMG_STATE.find(refImageState_);
    if (findItr != STR_2_IMG_STATE.end())
        refImageState_ = std::to_string(findItr->second);
    else
        RCLCPP_WARN(CalibrationBase::logger_, "String passed to 'ref_image_state' is not valid. "
                                              "\nSetting 'ref_image_state' to default: %s",
                    DEFAULT_IMG_STATE_STR.c_str());

    //--- sync queue
    syncQueueSize_ = CalibrationBase::readNumericLaunchParameter<int>(
      ipNode, "sync_queue_size", DEFAULT_SYNC_QUEUE_SIZE, 1, INT_MAX);

    //--- exact sync
    useExactSync_ = ipNode->get_parameter("use_exact_sync").as_bool();

    return true;
}

//==================================================================================================
bool ExtrinsicCameraCameraCalibration::setDynamicParameter(const rclcpp::Parameter& iParameter)
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

void ExtrinsicCameraCameraCalibration::reset()
{
    ExtrinsicCalibrationBase::reset();

    pSrcDataProcessor_->reset();
    pRefDataProcessor_->reset();
}

//==================================================================================================
bool ExtrinsicCameraCameraCalibration::shutdownSubscribers()
{
    //--- check if node is initialized
    if (!CalibrationBase::isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(CalibrationBase::dataProcessingMutex_);

    //--- unsubscribe subscribers
    srcImageSubsc_.unsubscribe();
    refImageSubsc_.unsubscribe();

    return true;
}

//==================================================================================================
} // namespace multisensor_calibration

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(multisensor_calibration::ExtrinsicCameraCameraCalibration)
