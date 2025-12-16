/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/calibration/ExtrinsicCameraReferenceCalibration.h"

// Std
#include <cstring>
#include <functional>
#include <future>

// Qt
#include <QFile>

// ROS
#include <memory>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2/LinearMath/Transform.hpp>

// PCL
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>

// Eigen
#include <Eigen/Eigenvalues>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"

namespace multisensor_calibration
{

//==================================================================================================
ExtrinsicCameraReferenceCalibration::
  ExtrinsicCameraReferenceCalibration(const std::string& nodeName,
                                      const rclcpp::NodeOptions& options) :
  Extrinsic2d3dCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>(
    EXTRINSIC_CAMERA_REFERENCE_CALIBRATION),
  rclcpp::Node(nodeName, options)
{
    CalibrationBase::init(this);
}

//==================================================================================================
ExtrinsicCameraReferenceCalibration::ExtrinsicCameraReferenceCalibration(const rclcpp::NodeOptions& options) :
  ExtrinsicCameraReferenceCalibration(
    CALIB_TYPE_2_NODE_NAME.at(EXTRINSIC_CAMERA_REFERENCE_CALIBRATION),
    options)
{
}

//==================================================================================================
ExtrinsicCameraReferenceCalibration::~ExtrinsicCameraReferenceCalibration()
{
    //--- reset pointers message filters before sensor processors in order to avoid seg fault during
    //--- disconnection of callbacks.
    pSrcDataProcessor_.reset();
    pRefDataProcessor_.reset();
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibration::finalizeCalibration()
{
    if (!pSrcDataProcessor_->isCameraIntrinsicsSet())
    {
        RCLCPP_ERROR(logger_, "Could not finalize calibration. "
                              "Camera intrinsics are not set");
        return false;
    }

    //--- get all observations from data processors
    std::set<uint> cameraObservationIds;
    std::vector<cv::Point2f> cameraCornerObservations;
    pSrcDataProcessor_->getOrderedObservations(cameraObservationIds, cameraCornerObservations);

    std::set<uint> refObservationIds;
    std::vector<cv::Point3f> refCornerObservations;
    pRefDataProcessor_->getOrderedObservations(refObservationIds, refCornerObservations);

    //--- remove observations that do not have a correspondence in the other list
    removeCornerObservationsWithoutCorrespondence(cameraObservationIds,
                                                  refObservationIds,
                                                  refCornerObservations);
    removeCornerObservationsWithoutCorrespondence(refObservationIds,
                                                  cameraObservationIds,
                                                  cameraCornerObservations);

    if (cameraObservationIds.empty() || refObservationIds.empty())
    {
        RCLCPP_ERROR(logger_, "Could not finalize calibration. "
                              "No common observations available.");
        return false;
    }

    //--- do pnp calibration using all observed poses of calibration target
    lib3d::Extrinsics finalSensorExtrinsics; //  sensor extrinsics after pnp calibration using all targets
    std::pair<double, int> pnpRetVal = runPnp(
      cameraCornerObservations.cbegin(),
      cameraCornerObservations.cend(),
      refCornerObservations.cbegin(),
      refCornerObservations.cend(),
      pSrcDataProcessor_->cameraIntrinsics(),
      static_cast<float>(registrationParams_.pnp_inlier_rpj_error_limit.value),
      false,
      finalSensorExtrinsics);

    ExtrinsicCalibrationBase::updateCalibrationResult(std::make_pair("Mean Reprojection Error (in pixel)", pnpRetVal.first), static_cast<int>(pRefDataProcessor_->getNumCalibIterations()));

    //--- calculate additional sensor calibrations if camera is to be calibrated as stereo camera
    if (isStereoCamera_ && rightCameraInfo_.width != 0)
        calculateAdditionalStereoCalibrations();
    else if (isStereoCamera_ && rightCameraInfo_.width == 0)
        RCLCPP_ERROR(logger_, "Could not calculate additional sensor calibrations in stereo case "
                              "because 'camera info' data of right camera is not available.");

    //--- publish last sensor extrinsics
    ExtrinsicCalibrationBase::publishLastCalibrationResult();

    return true;
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibration::initializeDataProcessors()
{
    //--- initialize camera data processor
    pSrcDataProcessor_ = std::make_shared<CameraDataProcessor>(
      logger_.get_name(), cameraSensorName_, calibTargetFilePath_);

    //--- initialize reference data processor
    pRefDataProcessor_ = std::make_shared<ReferenceDataProcessor3d>(
      logger_.get_name(), refSensorName_, calibTargetFilePath_);

    //--- if either of the two data processors are not initialized, return false.
    if (!pSrcDataProcessor_ || !pRefDataProcessor_)
        return false;

    //--- set data to camera data processor
    pSrcDataProcessor_->setImageState(imageState_);
    pSrcDataProcessor_->initializeServices(this);
    pSrcDataProcessor_->initializePublishers(this);

    //--- set data to reference data processor
    pRefDataProcessor_->initializeServices(this);
    pRefDataProcessor_->initializePublishers(this);

    return true;
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibration::initializeServices(rclcpp::Node* ipNode)
{
    if (!ExtrinsicCalibrationBase::initializeServices(ipNode))
        return false;

    //--- service to get camera intrinsics
    pCameraIntrSrv_ = ipNode->create_service<interf::srv::CameraIntrinsics>(
      "~/" + REQUEST_CAM_INTRINSICS_SRV_NAME,
      std::bind(&ExtrinsicCameraReferenceCalibration::onRequestCameraIntrinsics, this,
                std::placeholders::_1, std::placeholders::_2));

    return true;
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibration::initializeSubscribers(rclcpp::Node* ipNode)
{
    if (!Extrinsic2d3dCalibrationBase::initializeSubscribers(ipNode))
        return false;

    //--- subscribe to image topics
    pImageSubsc_ = ipNode->create_subscription<sensor_msgs::msg::Image>(
      cameraImageTopic_,
      1,
      std::bind(&ExtrinsicCameraReferenceCalibration::onSensorDataReceived, this, std::placeholders::_1));

    return true;
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibration::initializeWorkspaceObjects()
{
    bool retVal = true;

    retVal &= CalibrationBase::initializeWorkspaceObjects();

    //--- initialize calibration workspace
    fs::path calibWsPath = robotWsPath_;
    calibWsPath /=
      std::string(cameraSensorName_ + "_" + refSensorName_ + "_extrinsic_calibration");
    pCalibrationWs_ =
      std::make_shared<ExtrinsicCameraReferenceCalibWorkspace>(calibWsPath, logger_);
    retVal &= (pCalibrationWs_ != nullptr);

    if (pCalibrationWs_)
    {
        //--- create backup
        retVal &= pCalibrationWs_->createBackup();
    }

    return retVal;
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibration::onRequestCameraIntrinsics(
  const std::shared_ptr<interf::srv::CameraIntrinsics::Request> ipReq,
  std::shared_ptr<interf::srv::CameraIntrinsics::Response> opRes)
{
    UNUSED_VAR(ipReq);

    lib3d::Intrinsics cameraIntr;
    if (!isInitialized_ || pSrcDataProcessor_ == nullptr)
        cameraIntr = lib3d::Intrinsics();
    else
        cameraIntr = pSrcDataProcessor_->getCameraIntrinsics();

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

    //--- image state
    opRes->image_state = IMG_STATE_2_STR.find(imageState_)->second;

    return true;
}

//==================================================================================================
void ExtrinsicCameraReferenceCalibration::onSensorDataReceived(
  const InputImage_Message_T::ConstSharedPtr& ipImgMsg)
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
        RCLCPP_ERROR(logger_, "Reference data processor is not initialized.");
        return;
    }

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(dataProcessingMutex_);

    //--- convert input messages to data
    bool isConversionSuccessful = true;

    // camera image
    cv::Mat cameraImage;
    isConversionSuccessful &= pSrcDataProcessor_->getSensorDataFromMsg(ipImgMsg, cameraImage);

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
        if (!initializeCameraIntrinsics(pSrcDataProcessor_.get()))
            return;
    }

    //--- set frame ids and initialize sensor extrinsics if applicable
    if (imageFrameId_ != ipImgMsg->header.frame_id)
    {
        imageFrameId_ = ipImgMsg->header.frame_id;

        //--- compute sensor extrinsics between source frame id and ref or base frame id
        //--- if base frame id is not empty and unequal to refCloudFrameId also get transform
        //--- between refFrameID and baseFrameID and pass to refLidarProcessor.
        if (!baseFrameId_.empty() && baseFrameId_ != refFrameId_)
        {
            if (useTfTreeAsInitialGuess_)
                setSensorExtrinsicsFromFrameIds(imageFrameId_, baseFrameId_);

            if (tfBuffer_->_frameExists(baseFrameId_))
            {
                try
                {
                    auto t = tfBuffer_->lookupTransform(baseFrameId_, refFrameId_,
                                                        tf2::TimePointZero);
                    pRefDataProcessor_->setDataTransform(
                      std::make_shared<tf2::Transform>(tf2::Quaternion(t.transform.rotation.x,
                                                                       t.transform.rotation.y,
                                                                       t.transform.rotation.z,
                                                                       t.transform.rotation.w),
                                                       tf2::Vector3(t.transform.translation.x,
                                                                    t.transform.translation.y,
                                                                    t.transform.translation.z)));
                }
                catch (tf2::TransformException& ex)
                {
                    RCLCPP_ERROR(logger_,
                                 "tf2::TransformException in trying to set data transform "
                                 "to reference data processor: %s ",
                                 ex.what());
                }
            }
            else
            {
                RCLCPP_WARN(logger_,
                            "Base Frame '%s' does not exists! "
                            "Removing base frame and calibrating relative to reference cloud.",
                            baseFrameId_.c_str());
                baseFrameId_ = "";
                pRefDataProcessor_->setDataTransform(nullptr);
            }
        }
        else
        {
            if (useTfTreeAsInitialGuess_)
                setSensorExtrinsicsFromFrameIds(imageFrameId_, refFrameId_);
        }
    }

    // Level at which to do the processing
    CameraDataProcessor::EProcessingLevel procLevel = (captureCalibrationTarget_)
                                                        ? CameraDataProcessor::TARGET_DETECTION
                                                        : CameraDataProcessor::PREVIEW;

    //--- process camera data asynchronously
    std::future<CameraDataProcessor::EProcessingResult> camProcFuture =
      std::async(&CameraDataProcessor::processData,
                 pSrcDataProcessor_,
                 cameraImage,
                 procLevel);

    //--- wait for processing to return
    CameraDataProcessor::EProcessingResult camProcResult = camProcFuture.get();

    //--- if processing level is set to preview, publish preview from each sensor if successful
    if (procLevel == CameraDataProcessor::PREVIEW)
    {
        if (camProcResult == CameraDataProcessor::SUCCESS)
            pSrcDataProcessor_->publishPreview(ipImgMsg->header);
    }
    //--- else if, processing level is target_detection,
    //--- calibrate only if processing for both sensors is successful
    else if (procLevel == CameraDataProcessor::TARGET_DETECTION)
    {
        interf::msg::CalibrationResult calibResultMsg;

        if (camProcResult == CameraDataProcessor::SUCCESS)
        {
            //--- publish detections
            pSrcDataProcessor_->publishLastTargetDetection(ipImgMsg->header);

            //--- Other extrinsic calibration routines, i.e. camera-lidar or lidar-lidar, run
            //--- an intermediate calibration at this point. This will increase the calibration
            //--- iteration counter (calibrationItrCnt_) which, in turn, is evaluated to be larger
            //--- than 1 before finalizing the calibration. The calibration iteration counter
            //--- somehow also serves for an internal counter on how many common observations there
            //--- are. Since for the sensor-reference calibration, only a calibration is done at the
            //--- end, increase the counter when a observation in the source data was detected.
            calibrationItrCnt_++;

            calibResultMsg.is_successful = true;
        }
        else
        {
            calibResultMsg.is_successful = false;
        }

        pCalibResultPub_->publish(calibResultMsg);
    }

    //--- if this point is reachted, the target detection was not successful.
    //--- thus, if data processor is not pending for more data, set capturing flag to false.
    if (camProcResult != CameraDataProcessor::PENDING)
    {
        captureCalibrationTarget_ = false;
    }
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibration::saveCalibrationSettingsToWorkspace()
{
    if (!Extrinsic2d3dCalibrationBase::saveCalibrationSettingsToWorkspace())
        return false;

    QSettings* pCalibSettings = pCalibrationWs_->settingsPtr();
    if (!pCalibSettings)
        return false;

    //--- reference name
    pCalibSettings->setValue("reference/name",
                             QString::fromStdString(refSensorName_));

    //--- reference frame id
    pCalibSettings->setValue("reference/frame_id",
                             QString::fromStdString(refFrameId_));

    //--- sync settings file
    pCalibSettings->sync();

    return true;
}

//==================================================================================================
void ExtrinsicCameraReferenceCalibration::setupLaunchParameters(rclcpp::Node* ipNode) const
{
    Extrinsic2d3dCalibrationBase::setupLaunchParameters(ipNode);

    //--- reference_name
    auto refNameDesc = rcl_interfaces::msg::ParameterDescriptor{};
    refNameDesc.description =
      "Name of the reference with respect to which the source LiDAR sensor "
      "is to be calibrated.\n "
      "Default: \"reference\"";
    refNameDesc.read_only = true;
    ipNode->declare_parameter<std::string>("reference_name", "reference",
                                           refNameDesc);

    //--- reference_frame_id
    auto refFrameIdDesc = rcl_interfaces::msg::ParameterDescriptor{};
    refFrameIdDesc.description =
      "Frame ID to which the reference data is associated.\n"
      "Default: \"reference\"";
    refFrameIdDesc.read_only = true;
    ipNode->declare_parameter<std::string>("reference_frame_id", "reference",
                                           refFrameIdDesc);
}

//==================================================================================================
void ExtrinsicCameraReferenceCalibration::setupDynamicParameters(rclcpp::Node* ipNode) const
{
    registrationParams_.declareDynamic(ipNode);
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibration::readLaunchParameters(const rclcpp::Node* ipNode)
{
    if (!Extrinsic2d3dCalibrationBase::readLaunchParameters(ipNode))
        return false;

    //--- reference_name
    refSensorName_ =
      readStringLaunchParameter(ipNode, "reference_name", "reference");

    //--- ref_lidar_sensor_name
    refFrameId_ =
      readStringLaunchParameter(ipNode, "reference_frame_id", "reference");

    //--- need to set ref in order for the calibration meta data to be complete. However,
    //--- this is not evaluated
    refTopicName_ = "/cloud";

    //--- initial guess is not supported for sensor-reference calibration
    useTfTreeAsInitialGuess_ = false;

    return true;
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibration::setDynamicParameter(const rclcpp::Parameter& iParameter)
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
void ExtrinsicCameraReferenceCalibration::reset()
{
    ExtrinsicCalibrationBase::reset();

    pSrcDataProcessor_->reset();
    pRefDataProcessor_->reset();
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibration::shutdownSubscribers()
{
    //--- check if node is initialized
    if (!isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(dataProcessingMutex_);

    //--- unsubscribe subscribers
    pImageSubsc_.reset();

    return true;
}

} // namespace multisensor_calibration

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(multisensor_calibration::ExtrinsicCameraReferenceCalibration)
