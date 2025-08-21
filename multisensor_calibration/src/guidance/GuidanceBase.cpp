/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/guidance/GuidanceBase.h"

// ROS
#include <chrono>
#include <functional>
#include <tf2_ros/transform_listener.h>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/common/utils.hpp"
#include <multisensor_calibration_interface/srv/sensor_extrinsics.hpp>
namespace multisensor_calibration
{

using namespace utils;
;

//==================================================================================================
GuidanceBase::GuidanceBase(rclcpp::Node* pNode) :
  isInitialized_(true),
  initialPoseReceived_(false),
  pNode_(pNode),
  pExecutor_(nullptr),
  pCalibMetaDataTimer_(nullptr),
  pCalibResultSubsc_(nullptr),
  pResetSrv_(nullptr),
  extrinsicSensorPose_(lib3d::Extrinsics()),
  nextTargetPose_(lib3d::Extrinsics()),
  axes_({Eigen::Vector3d(1.0, 0.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0),
         Eigen::Vector3d(0.0, 0.0, 1.0)})
{
}

//==================================================================================================
GuidanceBase::~GuidanceBase()
{
}

//==================================================================================================
float GuidanceBase::computeAxisBound(const Eigen::Vector3d& iPnt, const Eigen::Vector3d& iVec,
                                     const Eigen::Vector4d& iPlane) const
{
    double lambda = (-iPlane(3) - iPlane.head<3>().dot(iPnt)) / (iPlane.head<3>().dot(iVec));

    //--- if lambda is negative, i.e. if it intersects the plane at the back, return infinity
    if (lambda <= 0)
        lambda = INFINITY;

    return static_cast<float>(lambda);
}

//==================================================================================================
bool GuidanceBase::initializeServices()
{
    //--- reset service
    pResetSrv_ =
      pNode_->create_service<multisensor_calibration_interface::srv::ResetCalibration>(RESET_SRV_NAME,
                                                                                       std::bind(&GuidanceBase::onReset, this, std::placeholders::_1, std::placeholders::_2));

    return pResetSrv_ != nullptr;
}

//==================================================================================================
bool GuidanceBase::initializeSubscribers()
{
    //--- subscriber to calib result
    pCalibResultSubsc_ = pNode_->create_subscription<CalibrationResult_Message_T>(
      calibratorNodeName_ + "/" + CALIB_RESULT_TOPIC_NAME,
      1,
      std::bind(&GuidanceBase::onCalibrationResultReceived, this, std::placeholders::_1));

    return pCalibResultSubsc_ != nullptr;
}

//==================================================================================================
bool GuidanceBase::initializeTimers()
{
    //--- initialize trigger to call routine to get calibration meta data
    pCalibMetaDataTimer_ = pNode_->create_wall_timer(std::chrono::seconds(1), std::bind(&GuidanceBase::getCalibrationMetaData, this), nullptr, false);

    return pCalibMetaDataTimer_ != nullptr;
}

//==================================================================================================
bool GuidanceBase::isTargetPoseWithinBoundingPlanes(const lib3d::Extrinsics& iPose) const
{
    bool retVal = true;

    // 4x4 RT matrix of the pose transforming a local point into the reference coordinate system
    const cv::Matx44d POSE_RT = iPose.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF);

    auto isPntWithinBoundingPlane = [&](const cv::Vec4d& iLocalPnt, const Eigen::Vector4d& iPlane)
    {
        cv::Vec4d refPnt = POSE_RT * iLocalPnt;
        return ((refPnt(0) * iPlane(0) +
                 refPnt(1) * iPlane(1) +
                 refPnt(2) * iPlane(2) +
                 iPlane(3)) > 0);
    };

    for (Eigen::Vector4d plane : fovBoundingPlanes_)
    {
        // Top-Left
        retVal &= isPntWithinBoundingPlane(cv::Vec4d(-calibrationTarget_.boardSize.width / 2,
                                                     calibrationTarget_.boardSize.height / 2,
                                                     0.0,
                                                     1.0),
                                           plane);
        // Top-Right
        retVal &= isPntWithinBoundingPlane(cv::Vec4d(calibrationTarget_.boardSize.width / 2,
                                                     calibrationTarget_.boardSize.height / 2,
                                                     0.0,
                                                     1.0),
                                           plane);
        // Bottom-Right
        retVal &= isPntWithinBoundingPlane(cv::Vec4d(calibrationTarget_.boardSize.width / 2,
                                                     -calibrationTarget_.boardSize.height / 2,
                                                     0.0,
                                                     1.0),
                                           plane);
        // Bottom-Left
        retVal &= isPntWithinBoundingPlane(cv::Vec4d(-calibrationTarget_.boardSize.width / 2,
                                                     -calibrationTarget_.boardSize.height / 2,
                                                     0.0,
                                                     1.0),
                                           plane);
    }

    return retVal;
}

//==================================================================================================
void GuidanceBase::onCalibrationResultReceived(const CalibrationResult_Message_T::ConstSharedPtr& ipResultMsg)
{
    if (!ipResultMsg->is_successful)
        return;

    //--- construct transform from pose
    tf2::Transform ref2LocalTransform;
    utils::cvtGeometryTransform2TfTransform(ipResultMsg->sensor_extrinsics, ref2LocalTransform);

    //--- convert to extrinsic sensor pose
    setCameraExtrinsicsFromTfTransform(ref2LocalTransform,
                                       extrinsicSensorPose_);

    //--- compute overlapping Fov
    computeExtrinsicFovBoundingPlanes();

    //--- compute next target pose
    computeNextTargetPose();
}

//==================================================================================================
void GuidanceBase::onTargetPoseReceived(const TargetBoardPose_Message_T::ConstSharedPtr& ipPoseMsg)
{
    if (ipPoseMsg->is_detection)
    {
        //--- construct transform from pose
        tf2::Transform ref2LocalTransform;
        utils::cvtGeometryPose2TfTransform(ipPoseMsg->target_pose, ref2LocalTransform);

        //--- convert to extrinsic sensor pose
        detectedTargetPoses_.push_back(lib3d::Extrinsics());
        setCameraExtrinsicsFromTfTransform(ref2LocalTransform,
                                           detectedTargetPoses_.back());
    }
}

//==================================================================================================
bool GuidanceBase::readLaunchParameters()
{
    return true;
}

//==================================================================================================
void GuidanceBase::getCalibrationMetaData()
{
    //--- get calibration meta data
    pMetaDataClient_ =
      pNode_->create_client<multisensor_calibration_interface::srv::CalibrationMetaData>(calibratorNodeName_ +
                                                                                         "/" + REQUEST_META_DATA_SRV_NAME);

    bool isServiceAvailable = false;
    const int MAX_TRIES     = 10;
    int cntr                = 0;

    while (!isServiceAvailable && cntr < MAX_TRIES)
    {
        isServiceAvailable = pMetaDataClient_->wait_for_service(std::chrono::milliseconds(500));
        cntr++;
    }

    if (isServiceAvailable)
    {

        auto request = std::make_shared<multisensor_calibration_interface::srv::CalibrationMetaData::Request>();
        auto future  = pMetaDataClient_->async_send_request(
          request,
          [this](rclcpp::Client<CalibrationMetadataSrv>::SharedFuture response)
          {
              pCalibrationMetaData_ = response.get();

              //--- if calibration metadata is complete stop timer
              if (pCalibrationMetaData_->is_complete)
              {
                  pCalibMetaDataTimer_->cancel();
                  // read calibration target
                  calibrationTarget_.readFromYamlFile(pCalibrationMetaData_->calib_target_file_path);
                  isInitialized_ &= calibrationTarget_.isValid();

                  // initialize subscribers
                  isInitialized_ &= initializeSubscribers();

                  // get initial sensor pose
                  isInitialized_ &= getInitialSensorPose();
              }
              else
              {
                  RCLCPP_ERROR(pNode_->get_logger(),
                               "Failure in getting calibration meta data.\n"
                               "Check if calibration node is initialized!");
              }
          });
    }
    else
    {
        RCLCPP_ERROR(pNode_->get_logger(),
                     "Service to get calibration meta data is not available.\n"
                     "Check if calibration node is initialized!");
    }
}

//==================================================================================================
bool GuidanceBase::getInitialSensorPose()
{
    //--- get sensor extrinsics
    extrinsicsClient_ =
      pNode_->create_client<multisensor_calibration_interface::srv::SensorExtrinsics>(calibratorNodeName_ +
                                                                                      "/" + REQUEST_SENSOR_EXTRINSICS_SRV_NAME);

    bool isServiceAvailable = false;
    const int MAX_TRIES     = 10;
    int cntr                = 0;
    while (!isServiceAvailable && cntr < MAX_TRIES)
    {
        isServiceAvailable = extrinsicsClient_->wait_for_service(std::chrono::milliseconds(500));
        cntr++;
    }

    if (!isServiceAvailable)
    {
        RCLCPP_ERROR(pNode_->get_logger(),
                     "Service to get extrinsics is not available");
        return false;
    }

    auto request = std::make_shared<multisensor_calibration_interface::srv::SensorExtrinsics::Request>();
    auto future  = extrinsicsClient_->async_send_request(
      request, [this](rclcpp::Client<multisensor_calibration_interface::srv::SensorExtrinsics>::SharedFuture response)
      {
          auto& POSE = response.get()->extrinsics;
          //--- check if extrinsic pose is 0
          if (tf2::Vector3(POSE.position.x,
                           POSE.position.y,
                           POSE.position.z) == tf2::Vector3(0, 0, 0) &&
              tf2::Quaternion(POSE.orientation.x,
                              POSE.orientation.y,
                              POSE.orientation.z,
                              POSE.orientation.w) == tf2::Quaternion(0, 0, 0, 1))
          {
              RCLCPP_INFO(pNode_->get_logger(), "No initial sensor pose available. Please add first observation:"
                                                "\n\t> Place target in field-of-view of both sensors."
                                                "\n\t> Press button to add observation.");
          }
          else
          {
              //--- construct transform from pose
              tf2::Transform ref2LocalTransform;
              utils::cvtGeometryPose2TfTransform(POSE, ref2LocalTransform);

              //--- convert to extrinsic sensor pose
              setCameraExtrinsicsFromTfTransform(ref2LocalTransform,
                                                 extrinsicSensorPose_);
          }

          initialPoseReceived_ = true; });

    return true;
}

//==================================================================================================
bool GuidanceBase::onReset(multisensor_calibration_interface::srv::ResetCalibration::Request::SharedPtr iReq,
                           multisensor_calibration_interface::srv::ResetCalibration::Response::SharedPtr oRes)
{
    UNUSED_VAR(iReq);

    extrinsicSensorPose_ = lib3d::Extrinsics();
    detectedTargetPoses_.clear();
    resetNextTargetPose();

    oRes->is_accepted = true;
    oRes->msg         = "Guidance is reset.";

    return true;
}

} // namespace multisensor_calibration