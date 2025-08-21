/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/calibration/Extrinsic2d3dCalibrationBase.h"

// Std
#include <thread>
#include <vector>

// ROS
#include <tf2/LinearMath/Transform.h>

// multisensor_calibration
#include "multisensor_calibration/common/utils.hpp"
#include "multisensor_calibration/sensor_data_processing/LidarDataProcessor.h"
#include "multisensor_calibration/sensor_data_processing/ReferenceDataProcessor3d.h"

namespace multisensor_calibration
{

using namespace utils;

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  Extrinsic2d3dCalibrationBase(ECalibrationType type) :
  ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>(type),
  cameraInfoTopic_(""),
  imageState_(STR_2_IMG_STATE.at(DEFAULT_IMG_STATE_STR)),
  isStereoCamera_(false),
  rightCameraSensorName_(DEFAULT_CAMERA_SENSOR_NAME),
  rightCameraInfoTopic_(""),
  rightImageFrameId_(""),
  rectSuffix_("_rect"),
  leftCameraInfo_(),
  rightCameraInfo_()
{
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::~Extrinsic2d3dCalibrationBase()
{
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
void Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  calculateAdditionalStereoCalibrations()
{
    calibResult_.calibrations.resize(4);

    tf2::Matrix3x3 leftRectMat =
      tf2::Matrix3x3(leftCameraInfo_.r[0], leftCameraInfo_.r[1], leftCameraInfo_.r[2],
                     leftCameraInfo_.r[3], leftCameraInfo_.r[4], leftCameraInfo_.r[5],
                     leftCameraInfo_.r[6], leftCameraInfo_.r[7], leftCameraInfo_.r[8]);

    tf2::Matrix3x3 rightRectMat =
      tf2::Matrix3x3(rightCameraInfo_.r[0], rightCameraInfo_.r[1], rightCameraInfo_.r[2],
                     rightCameraInfo_.r[3], rightCameraInfo_.r[4], rightCameraInfo_.r[5],
                     rightCameraInfo_.r[6], rightCameraInfo_.r[7], rightCameraInfo_.r[8]);

    double roll, pitch, yaw;

    //--- if image state is DISTORTED or UNDISTORTED, calculate position of frame of left
    //--- rectified image
    //--- the rectified frame is at the same position as the distorted frame, but rotated by the
    //--- transposed (inverse) rectification matrix provided in the camera_info
    if (imageState_ == DISTORTED ||
        imageState_ == UNDISTORTED)
    {
        calibResult_.calibrations[1].srcSensorName =
          calibResult_.calibrations[0].srcSensorName + rectSuffix_;
        calibResult_.calibrations[1].srcFrameId =
          calibResult_.calibrations[0].srcFrameId + rectSuffix_;
        calibResult_.calibrations[1].refSensorName =
          calibResult_.calibrations[0].srcSensorName;
        calibResult_.calibrations[1].refFrameId =
          calibResult_.calibrations[0].srcFrameId;
        calibResult_.calibrations[1].baseFrameId = "";

        calibResult_.calibrations[1].XYZ = tf2::Vector3(0.f, 0.f, 0.f);

        leftRectMat.transpose().getRPY(roll, pitch, yaw);
        calibResult_.calibrations[1].RPY = tf2::Vector3(roll, pitch, yaw);
    }
    //--- if image state is STEREO_RECTIFIED, calculate position of frame of left distorted image
    //--- the distorted frame is at the same position as the rectified frame, but rotated by the
    //--- rectification matrix provided in the camera_info
    else
    {
        //--- copy rectified calibration to second position to preserve order
        calibResult_.calibrations[1] = calibResult_.calibrations[0];

        auto suffixPos = calibResult_.calibrations[0].srcSensorName.rfind(rectSuffix_);
        if (suffixPos != std::string::npos)
            calibResult_.calibrations[0].srcSensorName.replace(
              suffixPos, rectSuffix_.length(), "");

        suffixPos = calibResult_.calibrations[0].srcFrameId.rfind(rectSuffix_);
        if (suffixPos != std::string::npos)
            calibResult_.calibrations[0].srcFrameId.replace(
              suffixPos, rectSuffix_.length(), "");
        calibResult_.calibrations[0].baseFrameId = "";
        calibResult_.calibrations[0].refSensorName =
          calibResult_.calibrations[1].srcSensorName;
        calibResult_.calibrations[0].refFrameId =
          calibResult_.calibrations[1].srcFrameId;

        calibResult_.calibrations[0].XYZ = tf2::Vector3(0.f, 0.f, 0.f);

        leftRectMat.getRPY(roll, pitch, yaw);
        calibResult_.calibrations[0].RPY = tf2::Vector3(roll, pitch, yaw);
    }

    //--- calculate pose of right rectified camera from left rectified camera
    //--- the pose of the right rectified frame is moved by the baseline of the stereo pair along
    //--- the x axis of the left rectified frame
    //--- the baseline is computed from P(0,3)/-P(0,0) stored inside the right camera info
    calibResult_.calibrations[3].srcSensorName = rightCameraSensorName_;
    calibResult_.calibrations[3].srcFrameId    = rightCameraInfo_.header.frame_id + rectSuffix_;
    if (imageState_ == DISTORTED ||
        imageState_ == UNDISTORTED)
    {
        calibResult_.calibrations[3].srcSensorName += rectSuffix_;
    }
    calibResult_.calibrations[3].refSensorName =
      calibResult_.calibrations[1].srcSensorName;
    calibResult_.calibrations[3].refFrameId =
      calibResult_.calibrations[1].srcFrameId;
    calibResult_.calibrations[3].baseFrameId = "";

    calibResult_.calibrations[3].XYZ =
      tf2::Vector3((-rightCameraInfo_.p[3] / rightCameraInfo_.p[0]), 0.f, 0.f);

    calibResult_.calibrations[3].RPY = tf2::Vector3(0.f, 0.f, 0.f);

    //--- calculate pose of right undistorted camera from right rectified camera
    //--- the distorted frame is at the same position as the rectified frame, but rotated by the
    //--- rectification matrix provided in the camera_info
    calibResult_.calibrations[2].srcSensorName = rightCameraSensorName_;
    calibResult_.calibrations[2].srcFrameId    = rightCameraInfo_.header.frame_id;
    if (imageState_ == STEREO_RECTIFIED)
    {
        auto suffixPos = calibResult_.calibrations[2].srcSensorName.rfind(rectSuffix_);
        if (suffixPos != std::string::npos)
            calibResult_.calibrations[2].srcSensorName.replace(
              suffixPos, rectSuffix_.length(), "");
    }
    calibResult_.calibrations[2].refSensorName =
      calibResult_.calibrations[3].srcSensorName;
    calibResult_.calibrations[2].refFrameId =
      calibResult_.calibrations[3].srcFrameId;
    calibResult_.calibrations[2].baseFrameId = "";

    calibResult_.calibrations[2].XYZ = tf2::Vector3(0.f, 0.f, 0.f);

    rightRectMat.getRPY(roll, pitch, yaw);
    calibResult_.calibrations[2].RPY = tf2::Vector3(roll, pitch, yaw);
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
std::pair<double, int> Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::runPnp(
  const std::vector<cv::Point2f>::const_iterator& iCamObsStart,
  const std::vector<cv::Point2f>::const_iterator& iCamObsEnd,
  const std::vector<cv::Point3f>::const_iterator& iLidarObsStart,
  const std::vector<cv::Point3f>::const_iterator& iLidarObsEnd,
  const lib3d::Intrinsics& iCameraIntrinsics,
  const float& iInlierMaxRpjError,
  const bool& iUsePoseGuess,
  lib3d::Extrinsics& oNewSensorExtrinsics,
  const std::vector<uint>& iIndices) const
{
    std::vector<cv::Point3f> lidarCornerObs; // corner observations in lidar data
    std::vector<cv::Point2f> camCornerObs;   // corner observations in camera data

    //--- initialize 3d and 2d corner observations
    //--- if index list is empty copy all points, if not iterate over indices and copy only
    //--- indexed observations
    if (iIndices.empty())
    {
        std::copy(iLidarObsStart, iLidarObsEnd, std::back_inserter(lidarCornerObs));
        std::copy(iCamObsStart, iCamObsEnd, std::back_inserter(camCornerObs));
    }
    else
    {
        const int LIDAR_OBS_END_IDX = std::distance(iCamObsStart, iCamObsEnd);
        const int CAM_OBS_END_IDX   = std::distance(iCamObsStart, iCamObsEnd);
        for (uint idx : iIndices)
        {
            if (idx < static_cast<uint>(LIDAR_OBS_END_IDX))
                lidarCornerObs.push_back(*(iLidarObsStart + idx));
            if (idx < static_cast<uint>(CAM_OBS_END_IDX))
                camCornerObs.push_back(*(iCamObsStart + idx));
        }
    }

    //--- initialize extrinsic guess
    lib3d::Extrinsics prevExtrinsics =
      ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::sensorExtrinsics_.back();
    prevExtrinsics.setTransfDirection(lib3d::Extrinsics::REF_2_LOCAL);
    cv::Vec3d cameraRVec = prevExtrinsics.getRotationVec();
    cv::Vec3d cameraTVec = prevExtrinsics.getTranslationVec();

    //--- run solvePnP
    std::vector<int> inliers;
    cv::solvePnPRansac(lidarCornerObs, camCornerObs,
                       iCameraIntrinsics.getK_as3x3(), iCameraIntrinsics.getDistortionCoeffs(),
                       cameraRVec, cameraTVec, iUsePoseGuess, 100,
                       iInlierMaxRpjError,
                       0.99, inliers);

    //--- calculate reprojection error
    double rpjError = utils::calculateMeanReprojectionError(camCornerObs,
                                                            lidarCornerObs,
                                                            iCameraIntrinsics.getK_as3x3(),
                                                            iCameraIntrinsics.getDistortionCoeffs(),
                                                            cameraRVec, cameraTVec,
                                                            inliers);

    //--- set new extrinsics
    oNewSensorExtrinsics.setTransfDirection(lib3d::Extrinsics::REF_2_LOCAL);
    oNewSensorExtrinsics.setRotationVec(cameraRVec);
    oNewSensorExtrinsics.setTranslationVec(cameraTVec);

    return std::make_pair(rpjError, static_cast<int>(inliers.size()));
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  initializeCameraIntrinsics(CameraDataProcessor* iopCamProcessor)
{
    if (leftCameraInfo_.width != 0)
    {
        lib3d::Intrinsics cameraIntr;
        setCameraIntrinsicsFromCameraInfo(leftCameraInfo_,
                                          cameraIntr,
                                          imageState_);
        iopCamProcessor->setCameraIntrinsics(cameraIntr);

        return true;
    }
    else
    {
        RCLCPP_ERROR(CalibrationBase::logger_,
                     "Wait for message of 'camera_info' topic has timed out. "
                     "\n'camera_info' topic: %s"
                     "\nWaiting for next data package.",
                     cameraInfoTopic_.c_str());
        return false;
    }
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  initializeSubscribers(rclcpp::Node* ipNode)
{
    //--- left camera info message
    pLeftCamInfoSubsc_ = ipNode->create_subscription<sensor_msgs::msg::CameraInfo>(
      cameraInfoTopic_, 1,
      std::bind(&Extrinsic2d3dCalibrationBase::onLeftCameraInfoReceived, this,
                std::placeholders::_1));

    //--- left camera info message
    if (!rightCameraInfoTopic_.empty())
    {
        pRightCamInfoSubsc_ = ipNode->create_subscription<sensor_msgs::msg::CameraInfo>(
          rightCameraInfoTopic_, 1,
          std::bind(&Extrinsic2d3dCalibrationBase::onRightCameraInfoReceived, this,
                    std::placeholders::_1));
    }

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
void Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  onLeftCameraInfoReceived(const sensor_msgs::msg::CameraInfo::SharedPtr pCamInfo)
{
    if (leftCameraInfo_.width != pCamInfo->width)
        leftCameraInfo_ = *pCamInfo;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
void Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  onRightCameraInfoReceived(const sensor_msgs::msg::CameraInfo::SharedPtr pCamInfo)
{
    if (rightCameraInfo_.width != pCamInfo->width)
        rightCameraInfo_ = *pCamInfo;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  saveCalibrationSettingsToWorkspace()
{
    if (!ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
          saveCalibrationSettingsToWorkspace())
        return false;

    QSettings* pCalibSettings = CalibrationBase::pCalibrationWs_->settingsPtr();
    if (!pCalibSettings)
        return false;

    //--- camera sensor name
    pCalibSettings->setValue("camera/sensor_name",
                             QString::fromStdString(cameraSensorName_));

    //--- camera image topic
    pCalibSettings->setValue("camera/image_topic",
                             QString::fromStdString(cameraImageTopic_));

    //--- camera info topic
    pCalibSettings->setValue("camera/info_topic",
                             QString::fromStdString(cameraInfoTopic_));

    //--- camera image state
    pCalibSettings->setValue("camera/image_state",
                             QVariant::fromValue(static_cast<int>(imageState_)));

    //--- is stereo camera
    pCalibSettings->setValue("camera/is_stereo_camera",
                             QVariant::fromValue(isStereoCamera_));

    //--- right camera sensor name
    pCalibSettings->setValue("camera/right_sensor_name",
                             QString::fromStdString(rightCameraSensorName_));

    //--- right camera info topic
    pCalibSettings->setValue("camera/right_info_topic",
                             QString::fromStdString(rightCameraInfoTopic_));

    //--- rect frame id suffix
    pCalibSettings->setValue("camera/rect_suffix",
                             QString::fromStdString(rectSuffix_));

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
void Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::setupLaunchParameters(
  rclcpp::Node* ipNode) const
{
    ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::setupLaunchParameters(ipNode);

    //--- camera sensor name
    auto cameraSensorNameDesc = rcl_interfaces::msg::ParameterDescriptor{};
    cameraSensorNameDesc.description =
      "Name of the camera sensor that is to be calibrated.\n"
      "Default: \"camera\"";
    cameraSensorNameDesc.read_only = true;
    ipNode->declare_parameter<std::string>("camera_sensor_name", DEFAULT_CAMERA_SENSOR_NAME,
                                           cameraSensorNameDesc);

    //--- camera image topic
    auto cameraImageTopicDesc = rcl_interfaces::msg::ParameterDescriptor{};
    cameraImageTopicDesc.description =
      "Topic name of the corresponding camera images.\n"
      "Default: \"/camera/image_color\"";
    cameraImageTopicDesc.read_only = true;
    ipNode->declare_parameter<std::string>("camera_image_topic", DEFAULT_CAMERA_IMAGE_TOPIC,
                                           cameraImageTopicDesc);

    //--- camera info topic
    auto cameraInfoTopicDesc = rcl_interfaces::msg::ParameterDescriptor{};
    cameraInfoTopicDesc.description =
      "Name of the camera info topic. If this parameter is left empty the camera info topic name is "
      "constructed from the specified ```camera_image_topic```.\n "
      "Default: \"\"";
    cameraInfoTopicDesc.read_only = true;
    ipNode->declare_parameter<std::string>("camera_info_topic", "",
                                           cameraInfoTopicDesc);

    //--- image state
    auto imageStateDesc = rcl_interfaces::msg::ParameterDescriptor{};
    imageStateDesc.description =
      "State of the camera images used.\n"
      "Default: \"DISTORTED\"";
    imageStateDesc.read_only = true;
    ipNode->declare_parameter<std::string>("image_state", DEFAULT_IMG_STATE_STR,
                                           imageStateDesc);

    //--- is stereo camera
    auto isStereoCameraDesc = rcl_interfaces::msg::ParameterDescriptor{};
    isStereoCameraDesc.description =
      "Set to true, if camera is to be calibrated as stereo camera. "
      "If set to true, ```right_camera_sensor_name``` and ```right_camera_info_topic``` "
      "also need to be set.\n"
      "Default: false";
    isStereoCameraDesc.read_only = true;
    ipNode->declare_parameter<bool>("is_stereo_camera", false, isStereoCameraDesc);

    //--- right camera sensor name
    auto rightCameraNameDesc = rcl_interfaces::msg::ParameterDescriptor{};
    rightCameraNameDesc.description =
      "Name of the right camera sensor when the camera is to be "
      "calibrated as a stereo camera system. Required if ```is_stereo_camera == true```.\n"
      "Default: \"\"";
    rightCameraNameDesc.read_only = true;
    ipNode->declare_parameter<std::string>("right_camera_sensor_name", "", rightCameraNameDesc);

    //--- right camera info topic
    auto rightCameraInfoDesc = rcl_interfaces::msg::ParameterDescriptor{};
    rightCameraInfoDesc.description =
      "Topic name of the camera info corresponding to the right camera. "
      "This is needed when the camera is to be calibrated as a stereo camera system. "
      "Required if ```is_stereo_camera == true```.\n"
      "Default: \"\"";
    rightCameraInfoDesc.read_only = true;
    ipNode->declare_parameter<std::string>("right_camera_info_topic", "", rightCameraInfoDesc);

    //--- rect suffix
    auto rectSuffixDesc = rcl_interfaces::msg::ParameterDescriptor{};
    rectSuffixDesc.description =
      "Suffix of the of the right sensor name as well as the frame id for the "
      "rectified images. If the ```image_state``` of the input images is DISTORTED or UNDISTORTED "
      "this is added to the rectified frame id. If the imageState_ is STEREO_RECTIFIED this is "
      "removed from the frame id. "
      "Default: \"_rect\"";
    rectSuffixDesc.read_only = true;
    ipNode->declare_parameter<std::string>("rect_suffix", "_rect", rectSuffixDesc);
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  readLaunchParameters(const rclcpp::Node* ipNode)
{
    if (!ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
          readLaunchParameters(ipNode))
        return false;

    //--- camera_sensor_name
    cameraSensorName_ =
      CalibrationBase::readStringLaunchParameter(ipNode, "camera_sensor_name",
                                                 DEFAULT_CAMERA_SENSOR_NAME);

    //--- camera_image_topic
    cameraImageTopic_ =
      CalibrationBase::readStringLaunchParameter(ipNode, "camera_image_topic",
                                                 DEFAULT_CAMERA_IMAGE_TOPIC);

    //--- camera_info_topic
    cameraInfoTopic_ = ipNode->get_parameter("camera_info_topic").as_string();
    if (cameraInfoTopic_.empty())
    {
        cameraInfoTopic_ =
          cameraImageTopic_.substr(0, cameraImageTopic_.find_last_of('/')) + "/camera_info";
    }

    //--- image state
    std::string imageStateStr =
      CalibrationBase::readStringLaunchParameter(ipNode, "image_state",
                                                 DEFAULT_IMG_STATE_STR);
    auto findItr = STR_2_IMG_STATE.find(imageStateStr);
    if (findItr != STR_2_IMG_STATE.end())
        imageState_ = findItr->second;
    else
        RCLCPP_WARN(CalibrationBase::logger_, "String passed to 'image_state' is not valid. "
                                              "\nSetting 'image_state' to default: %s",
                    DEFAULT_IMG_STATE_STR.c_str());

    //--- is_stereo_camera
    isStereoCamera_ = ipNode->get_parameter("is_stereo_camera").as_bool();

    //--- right_camera_sensor_name
    rightCameraSensorName_ = ipNode->get_parameter("right_camera_sensor_name").as_string();

    //--- right_camera_info_topic
    rightCameraInfoTopic_ = ipNode->get_parameter("right_camera_info_topic").as_string();

    //--- rect_suffix
    rectSuffix_ = CalibrationBase::readStringLaunchParameter(ipNode, "rect_suffix",
                                                             "_rect");

    return true;
}

template class Extrinsic2d3dCalibrationBase<CameraDataProcessor, LidarDataProcessor>;
template class Extrinsic2d3dCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>;

} // namespace multisensor_calibration