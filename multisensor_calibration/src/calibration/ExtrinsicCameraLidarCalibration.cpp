/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/calibration/ExtrinsicCameraLidarCalibration.h"

// Std
#include <cstring>
#include <functional>
#include <future>

// Qt
#include <QFile>

// ROS
#include <sensor_msgs/msg/camera_info.hpp>

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
ExtrinsicCameraLidarCalibration::
  ExtrinsicCameraLidarCalibration(const std::string& nodeName,
                                  const rclcpp::NodeOptions& options) :
  Extrinsic2d3dCalibrationBase<CameraDataProcessor, LidarDataProcessor>(
    EXTRINSIC_CAMERA_LIDAR_CALIBRATION),
  rclcpp::Node(nodeName, options),
  pImgCloudApproxSync_(nullptr),
  pImgCloudExactSync_(nullptr),
  syncQueueSize_(DEFAULT_SYNC_QUEUE_SIZE),
  useExactSync_(false),
  pFrustumCullingFilters_()
{
    CalibrationBase::init(this);
}

//==================================================================================================
ExtrinsicCameraLidarCalibration::ExtrinsicCameraLidarCalibration(const rclcpp::NodeOptions& options) :
  ExtrinsicCameraLidarCalibration(
    CALIB_TYPE_2_NODE_NAME.at(EXTRINSIC_CAMERA_LIDAR_CALIBRATION),
    options)
{
}

//==================================================================================================
ExtrinsicCameraLidarCalibration::~ExtrinsicCameraLidarCalibration()
{
    //--- reset pointers message filters before sensor processors in order to avoid seg fault during
    //--- disconnection of callbacks.
    pImgCloudApproxSync_.reset();
    pImgCloudExactSync_.reset();
    pSrcDataProcessor_.reset();
    pRefDataProcessor_.reset();
}

//==================================================================================================
void ExtrinsicCameraLidarCalibration::calibrateLastObservation()
{
    if (!pSrcDataProcessor_->isCameraIntrinsicsSet())
    {
        RCLCPP_ERROR(logger_, "Could not calibrate last observation. Camera intrinsics are not set");
        return;
    }

    //--- check that for this calibration iteration, both camera and lidar observations are available
    //--- i.e. if num of captured observations is smaller than current calibration iteration, return
    if (pSrcDataProcessor_->getNumCalibIterations() < calibrationItrCnt_ ||
        pRefDataProcessor_->getNumCalibIterations() < calibrationItrCnt_)
        return;

    //--- get last observations from camera
    std::set<uint> cameraObservationIds;
    std::vector<cv::Point2f> cameraCornerObservations;
    pSrcDataProcessor_->getOrderedObservations(cameraObservationIds, cameraCornerObservations,
                                               calibrationItrCnt_, 1);

    //--- get last observations from LiDAR
    std::set<uint> lidarObservationIds;
    std::vector<cv::Point3f> lidarCornerObservations;
    pRefDataProcessor_->getOrderedObservations(lidarObservationIds, lidarCornerObservations,
                                                 calibrationItrCnt_, 1);

    //--- remove observations that do not have a correspondence in the other list
    removeCornerObservationsWithoutCorrespondence(cameraObservationIds,
                                                  lidarObservationIds,
                                                  lidarCornerObservations);
    removeCornerObservationsWithoutCorrespondence(lidarObservationIds,
                                                  cameraObservationIds,
                                                  cameraCornerObservations);

    //--- do pnp calibration for a single pose of calibration target.
    //--- use pose guess if there has already been a calibration, to enforce consistency between
    //--- calibrations.
    lib3d::Extrinsics newExtrinsics; // new (temporary) sensor extrinsics after pnp calibration
    std::pair<double, int> pnpRetVal = runPnp(
      cameraCornerObservations.cbegin(), cameraCornerObservations.cend(),
      lidarCornerObservations.cbegin(), lidarCornerObservations.cend(),
      pSrcDataProcessor_->cameraIntrinsics(),
      static_cast<float>(registrationParams_.pnp_inlier_rpj_error_limit.value),
      (calibrationItrCnt_ > 1),
      newExtrinsics);

    //--- check if calibration is to be kept
    if (((registrationParams_.limit_single_board_rpj_error.value &&
          pnpRetVal.first <= registrationParams_.single_board_max_rpj_error.value) ||
         (registrationParams_.limit_single_board_rpj_error.value == false)) &&
        pnpRetVal.second >= registrationParams_.single_board_min_inliers.value)
    {
        RCLCPP_INFO(logger_, "Calibration accepted!"
                             "\nMean Reprojection Error: %f px"
                             "\nInliers: %i pnts",
                    pnpRetVal.first, pnpRetVal.second);

        //--- store into member variable, and configure and apply frustum culling
        sensorExtrinsics_.push_back(newExtrinsics);
        configureAndApplyFrustumCulling();

        //--- increment iteration count
        calibrationItrCnt_++;

        //--- publish last sensor extrinsics
        ExtrinsicCalibrationBase::publishLastCalibrationResult();
    }
    else
    {
        RCLCPP_WARN(logger_, "Calibration rejected!"
                             "\nMean Reprojection Error: %f px (max. threshold: %f px)"
                             "\nInliers: %i pnts (min. threshold: %i pnts)"
                             "\nRemoving latest observations.",
                    pnpRetVal.first, registrationParams_.single_board_max_rpj_error.value,
                    pnpRetVal.second, registrationParams_.single_board_min_inliers.value);

        //--- remove all observations from this calibration iteration
        pSrcDataProcessor_->removeCalibIteration(calibrationItrCnt_);
        pRefDataProcessor_->removeCalibIteration(calibrationItrCnt_);

        //--- publish message on faled calibration sensor extrinsics
        // message publishing calibration result
        interf::msg::CalibrationResult calibResultMsg;
        calibResultMsg.is_successful = false;
        pCalibResultPub_->publish(calibResultMsg);
    }
}

//==================================================================================================
void ExtrinsicCameraLidarCalibration::configureAndApplyFrustumCulling()
{
    //--- construct frustum culling pose
    Eigen::Matrix4f RTmatrix;
    cv::cv2eigen(cv::Mat(sensorExtrinsics_.back().getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF)),
                 RTmatrix);
    Eigen::Matrix4f cam2robot;
    cam2robot << 0, 0, 1, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    Eigen::Matrix4f frustumCullingCamPose = RTmatrix * cam2robot;

    //--- configure filter
    pcl::FrustumCulling<InputPointType>::Ptr pFilter =
      pcl::FrustumCulling<InputPointType>::Ptr(new pcl::FrustumCulling<InputPointType>());
    pFilter->setHorizontalFOV(
      static_cast<float>(pSrcDataProcessor_->getCameraIntrinsics().getHFov()) + 10.f);
    pFilter->setVerticalFOV(
      static_cast<float>(pSrcDataProcessor_->getCameraIntrinsics().getVFov()) + 10.f);
    pFilter->setCameraPose(frustumCullingCamPose);
    pFilter->setNearPlaneDistance(0.01f);
    pFilter->setFarPlaneDistance(10.f);

    //--- push back filter into list
    pFrustumCullingFilters_.push_back(pFilter);

    //--- set filter
    if (pRefDataProcessor_)
    {
        pRefDataProcessor_->setPreprocFilter(pFilter);
    }
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibration::finalizeCalibration()
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

    std::set<uint> lidarObservationIds;
    std::vector<cv::Point3f> lidarCornerObservations;
    pRefDataProcessor_->getOrderedObservations(lidarObservationIds, lidarCornerObservations);

    //--- remove observations that do not have a correspondence in the other list
    removeCornerObservationsWithoutCorrespondence(cameraObservationIds,
                                                  lidarObservationIds,
                                                  lidarCornerObservations);
    removeCornerObservationsWithoutCorrespondence(lidarObservationIds,
                                                  cameraObservationIds,
                                                  cameraCornerObservations);

    if (cameraObservationIds.empty() || lidarObservationIds.empty())
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
      lidarCornerObservations.cbegin(),
      lidarCornerObservations.cend(),
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
bool ExtrinsicCameraLidarCalibration::initializeDataProcessors()
{
    //--- initialize camera data processor
    pSrcDataProcessor_.reset(
      new CameraDataProcessor(logger_.get_name(), cameraSensorName_, calibTargetFilePath_));

    //--- initialize lidar data processor
    pRefDataProcessor_.reset(
      new LidarDataProcessor(logger_.get_name(), refSensorName_, calibTargetFilePath_));

    //--- if either of the two data processors are not initialized, return false.
    if (!pSrcDataProcessor_ || !pRefDataProcessor_)
        return false;

    //--- set data to camera data processor
    pSrcDataProcessor_->setImageState(imageState_);
    pSrcDataProcessor_->initializeServices(this);
    pSrcDataProcessor_->initializePublishers(this);

    //--- set data to lidar data processor
    pRefDataProcessor_->initializeServices(this);
    pRefDataProcessor_->initializePublishers(this);
    pRefDataProcessor_->setParameters(lidarTargetDetectionParams_);

    return true;
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibration::initializeServices(rclcpp::Node* ipNode)
{
    if (!ExtrinsicCalibrationBase::initializeServices(ipNode))
        return false;

    //--- service to get camera intrinsics
    pCameraIntrSrv_ = ipNode->create_service<interf::srv::CameraIntrinsics>(
      "~/" + REQUEST_CAM_INTRINSICS_SRV_NAME,
      std::bind(&ExtrinsicCameraLidarCalibration::onRequestCameraIntrinsics, this,
                std::placeholders::_1, std::placeholders::_2));

    return true;
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibration::initializeSubscribers(rclcpp::Node* ipNode)
{
    if (!Extrinsic2d3dCalibrationBase<CameraDataProcessor, LidarDataProcessor>::
          initializeSubscribers(ipNode))
        return false;

    //--- subscribe to topics
    imageSubsc_.subscribe(ipNode, cameraImageTopic_, "raw");
    cloudSubsc_.subscribe(ipNode, refTopicName_);

    //--- initialize synchronizers
    if (useExactSync_)
    {
        pImgCloudExactSync_ =
          std::make_shared<message_filters::Synchronizer<ImgCloudExactSync>>(
            ImgCloudExactSync(10), imageSubsc_, cloudSubsc_);
        pImgCloudExactSync_->registerCallback(
          std::bind(&ExtrinsicCameraLidarCalibration::onSensorDataReceived, this,
                    std::placeholders::_1, std::placeholders::_2));
    }
    else
    {
        pImgCloudApproxSync_ =
          std::make_shared<message_filters::Synchronizer<ImgCloudApproxSync>>(
            ImgCloudApproxSync(syncQueueSize_), imageSubsc_, cloudSubsc_);
        pImgCloudApproxSync_->registerCallback(
          std::bind(&ExtrinsicCameraLidarCalibration::onSensorDataReceived, this,
                    std::placeholders::_1, std::placeholders::_2));
    }

    return true;
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibration::initializeWorkspaceObjects()
{
    bool retVal = true;

    retVal &= CalibrationBase::initializeWorkspaceObjects();

    //--- initialize calibration workspace
    fs::path calibWsPath = robotWsPath_;
    calibWsPath /=
      std::string(cameraSensorName_ + "_" + refSensorName_ + "_extrinsic_calibration");
    pCalibrationWs_ =
      std::make_shared<ExtrinsicCameraLidarCalibWorkspace>(calibWsPath, logger_);
    retVal &= (pCalibrationWs_ != nullptr);

    if (pCalibrationWs_)
    {
        //--- create backup
        retVal &= pCalibrationWs_->createBackup();
    }

    return retVal;
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibration::onRequestCameraIntrinsics(
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
bool ExtrinsicCameraLidarCalibration::onRequestRemoveObservation(
  const std::shared_ptr<interf::srv::RemoveLastObservation::Request> ipReq,
  std::shared_ptr<interf::srv::RemoveLastObservation::Response> opRes)
{

    if (ExtrinsicCalibrationBase<CameraDataProcessor, LidarDataProcessor>::onRequestRemoveObservation(ipReq, opRes))
    {
        sensorExtrinsics_.pop_back();
        //--- pop last frustum culling filter from stack and set previous filter
        pFrustumCullingFilters_.pop_back();
        if (pFrustumCullingFilters_.size() > 0 && pRefDataProcessor_)
            pRefDataProcessor_->setPreprocFilter(pFrustumCullingFilters_.back());
        else if (pFrustumCullingFilters_.size() == 0 && pRefDataProcessor_)
            pRefDataProcessor_->setPreprocFilter(nullptr);
    }
    else
        return false;

    return true;
}

//==================================================================================================
void ExtrinsicCameraLidarCalibration::onSensorDataReceived(
  const InputImage_Message_T::ConstSharedPtr& ipImgMsg,
  const InputCloud_Message_T::ConstSharedPtr& ipCloudMsg)
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
    cv::Mat cameraImage;
    isConversionSuccessful &= pSrcDataProcessor_->getSensorDataFromMsg(ipImgMsg, cameraImage);

    // point cloud
    pcl::PointCloud<InputPointType> pointCloud;
    isConversionSuccessful &= pRefDataProcessor_->getSensorDataFromMsg(ipCloudMsg, pointCloud);

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
    if (imageFrameId_ != ipImgMsg->header.frame_id ||
        refFrameId_ != ipCloudMsg->header.frame_id)
    {
        imageFrameId_ = ipImgMsg->header.frame_id;
        refFrameId_ = ipCloudMsg->header.frame_id;

        //--- if base frame id is not empty and unequal to refCloudFrameId use baseFrameID as
        //--- reference frame id.
        std::string tmpRefFrameId = refFrameId_;
        if (!baseFrameId_.empty() && baseFrameId_ != refFrameId_)
        {
            tmpRefFrameId = baseFrameId_;

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
                                 "to LiDAR data processor: %s ",
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

        //--- set sensor extrinsics from either cloud or base frame id and apply frustum culling
        if (useTfTreeAsInitialGuess_ &&
            setSensorExtrinsicsFromFrameIds(imageFrameId_, tmpRefFrameId))
        {
            //--- update preprocessing filter with current sensorExtrinsics_
            configureAndApplyFrustumCulling();
        }
        else
        {
            //--- reset preprocessing filter
            pRefDataProcessor_->setPreprocFilter(nullptr);
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

    //--- process lidar data asynchronously
    std::future<LidarDataProcessor::EProcessingResult> lidarProcFuture =
      std::async(&LidarDataProcessor::processData,
                 pRefDataProcessor_,
                 pointCloud,
                 static_cast<LidarDataProcessor::EProcessingLevel>(procLevel));

    //--- wait for processing to return
    CameraDataProcessor::EProcessingResult camProcResult  = camProcFuture.get();
    LidarDataProcessor::EProcessingResult lidarProcResult = lidarProcFuture.get();

    //--- if processing level is set to preview, publish preview from each sensor if successful
    if (procLevel == CameraDataProcessor::PREVIEW)
    {
        if (camProcResult == CameraDataProcessor::SUCCESS)
            pSrcDataProcessor_->publishPreview(ipImgMsg->header);
        if (lidarProcResult == LidarDataProcessor::SUCCESS)
            pRefDataProcessor_->publishPreview(ipCloudMsg->header.stamp,
                                                 (baseFrameId_.empty())
                                                   ? refFrameId_
                                                   : baseFrameId_);
    }
    //--- else if, processing level is target_detection,
    //--- calibrate only if processing for both sensors is successful
    else if (procLevel == CameraDataProcessor::TARGET_DETECTION)
    {
        if (camProcResult == CameraDataProcessor::SUCCESS &&
            lidarProcResult == LidarDataProcessor::SUCCESS)
        {
            //--- publish detections
            pSrcDataProcessor_->publishLastTargetDetection(ipImgMsg->header);
            pRefDataProcessor_->publishLastTargetDetection(ipCloudMsg->header.stamp,
                                                             (baseFrameId_.empty())
                                                               ? refFrameId_
                                                               : baseFrameId_);

            //--- do calibration
            calibrateLastObservation();
        }
        else
        {
            if (camProcResult != CameraDataProcessor::SUCCESS &&
                lidarProcResult == LidarDataProcessor::SUCCESS)
                pRefDataProcessor_->removeCalibIteration(calibrationItrCnt_);

            if (camProcResult == CameraDataProcessor::SUCCESS &&
                lidarProcResult != LidarDataProcessor::SUCCESS)
                pSrcDataProcessor_->removeCalibIteration(calibrationItrCnt_);

            interf::msg::CalibrationResult calibResultMsg;
            calibResultMsg.is_successful = false;
            pCalibResultPub_->publish(calibResultMsg);
        }
    }

    //--- if this point is reachted, the target detection was not successful.
    //--- thus, if data processor is not pending for more data, set capturing flag to false.
    if (camProcResult != CameraDataProcessor::PENDING &&
        lidarProcResult != LidarDataProcessor::PENDING)
    {
        captureCalibrationTarget_ = false;
    }
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibration::saveCalibrationSettingsToWorkspace()
{
    if (!Extrinsic2d3dCalibrationBase::saveCalibrationSettingsToWorkspace())
        return false;

    QSettings* pCalibSettings = pCalibrationWs_->settingsPtr();
    if (!pCalibSettings)
        return false;

    //--- lidar sensor name
    pCalibSettings->setValue("lidar/sensor_name",
                             QString::fromStdString(refSensorName_));

    //--- camera image topic
    pCalibSettings->setValue("lidar/cloud_topic",
                             QString::fromStdString(refTopicName_));

    //--- sync queue
    pCalibSettings->setValue("misc/sync_queue_size",
                             QVariant::fromValue(syncQueueSize_));

    //--- exact sync
    pCalibSettings->setValue("misc/use_exact_sync",
                             QVariant::fromValue(useExactSync_));

    //--- sync settings file
    pCalibSettings->sync();

    return true;
}

//==================================================================================================
void ExtrinsicCameraLidarCalibration::setupLaunchParameters(rclcpp::Node* ipNode) const
{
    Extrinsic2d3dCalibrationBase::setupLaunchParameters(ipNode);

    //--- lidar_sensor_name
    auto lidarSensorNameDesc = rcl_interfaces::msg::ParameterDescriptor{};
    lidarSensorNameDesc.description =
      "Name of the LiDAR sensor with respect to which the camera is to be calibrated.\n "
      "Default: \"lidar\"";
    lidarSensorNameDesc.read_only = true;
    ipNode->declare_parameter<std::string>("lidar_sensor_name", DEFAULT_LIDAR_SENSOR_NAME,
                                           lidarSensorNameDesc);

    //--- lidar_cloud_topic
    auto lidarCloudTopicDesc = rcl_interfaces::msg::ParameterDescriptor{};
    lidarCloudTopicDesc.description =
      "Topic name of the corresponding LiDAR cloud.\n"
      "Default: \"/lidar/cloud\"";
    lidarCloudTopicDesc.read_only = true;
    ipNode->declare_parameter<std::string>("lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC,
                                           lidarCloudTopicDesc);

    //--- sync queue
    auto syncQueueDesc = rcl_interfaces::msg::ParameterDescriptor{};
    syncQueueDesc.description =
      "Queue size used for the synchronization between the messages of the camera images and "
      "the LiDAR clouds.\n "
      "Default: 100";
    syncQueueDesc.read_only = true;
    ipNode->declare_parameter<int>("sync_queue_size", DEFAULT_SYNC_QUEUE_SIZE,
                                   syncQueueDesc);

    //--- exact sync
    auto exactSyncDesc = rcl_interfaces::msg::ParameterDescriptor{};
    exactSyncDesc.description =
      "Set to true if an exact time synchronization between the camera image messages and the "
      "LiDAR cloud messages.\n"
      "Default: false";
    syncQueueDesc.read_only = true;
    ipNode->declare_parameter<bool>("use_exact_sync", false, exactSyncDesc);
}

//==================================================================================================
void ExtrinsicCameraLidarCalibration::setupDynamicParameters(rclcpp::Node* ipNode) const
{
    registrationParams_.declareDynamic(ipNode);
    lidarTargetDetectionParams_.declareDynamic(ipNode);
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibration::readLaunchParameters(const rclcpp::Node* ipNode)
{
    if (!Extrinsic2d3dCalibrationBase::readLaunchParameters(ipNode))
        return false;

    //--- lidar_sensor_name
    refSensorName_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "lidar_sensor_name", DEFAULT_LIDAR_SENSOR_NAME);

    //--- lidar_cloud_topic
    refTopicName_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC);

    //--- sync queue
    syncQueueSize_ = CalibrationBase::readNumericLaunchParameter<int>(
      ipNode, "sync_queue_size", DEFAULT_SYNC_QUEUE_SIZE, 1, INT_MAX);

    //--- exact sync
    useExactSync_ = ipNode->get_parameter("use_exact_sync").as_bool();

    return true;
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibration::setDynamicParameter(const rclcpp::Parameter& iParameter)
{
    if (CalibrationBase::setDynamicParameter(iParameter))
    {
        return true;
    }
    if (registrationParams_.tryToSetParameter(iParameter))
    {
        return true;
    }
    else if (lidarTargetDetectionParams_.tryToSetParameter(iParameter))
    {
        pRefDataProcessor_->setParameters(lidarTargetDetectionParams_);
        return true;
    }
    else
    {
        return false;
    }
}

//==================================================================================================
void ExtrinsicCameraLidarCalibration::reset()
{
    ExtrinsicCalibrationBase::reset();

    pSrcDataProcessor_->reset();
    pRefDataProcessor_->reset();
    pRefDataProcessor_->setPreprocFilter(nullptr);
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibration::shutdownSubscribers()
{
    //--- check if node is initialized
    if (!CalibrationBase::isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(dataProcessingMutex_);

    //--- unsubscribe subscribers
    imageSubsc_.unsubscribe();
    cloudSubsc_.unsubscribe();

    return true;
}

} // namespace multisensor_calibration

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(multisensor_calibration::ExtrinsicCameraLidarCalibration)
