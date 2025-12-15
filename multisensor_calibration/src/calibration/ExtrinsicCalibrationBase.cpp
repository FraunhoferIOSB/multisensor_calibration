/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/calibration/ExtrinsicCalibrationBase.h"

// Std
#include <fstream>

// ROS
#include <geometry_msgs/msg/transform_stamped.hpp>

// PCL
#include <pcl/common/io.h>

// multisensor_calibration
#include "multisensor_calibration/common/utils.hpp"
#include "multisensor_calibration/sensor_data_processing/CameraDataProcessor.h"
#include "multisensor_calibration/sensor_data_processing/LidarDataProcessor.h"
#include "multisensor_calibration/sensor_data_processing/ReferenceDataProcessor3d.h"

namespace multisensor_calibration
{

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  ExtrinsicCalibrationBase(ECalibrationType type) :
  CalibrationBase(type),
  pCalibResultPub_(nullptr),
  pRemoveObsSrv_(nullptr),
  pCalibMetaDataSrv_(nullptr),
  pSensorExtrinsicsSrv_(nullptr),
  pSrcDataProcessor_(nullptr),
  pRefDataProcessor_(nullptr),
  srcSensorName_(""),
  srcTopicName_(""),
  srcFrameId_(""),
  refSensorName_(""),
  refTopicName_(""),
  refFrameId_(""),
  sensorExtrinsics_(std::vector<lib3d::Extrinsics>(1, lib3d::Extrinsics())),
  calibResult_(),
  useTfTreeAsInitialGuess_(false)
{
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::~ExtrinsicCalibrationBase()
{
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::initializePublishers(
  rclcpp::Node* ipNode)
{
    pCalibResultPub_ = ipNode->create_publisher<CalibrationResult_Message_T>(
      "~/" + CALIB_RESULT_TOPIC_NAME, 10);

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::onRequestRemoveObservation(
  const std::shared_ptr<interf::srv::RemoveLastObservation::Request> ipReq,
  std::shared_ptr<interf::srv::RemoveLastObservation::Response> opRes)
{
    UNUSED_VAR(ipReq);

    //--- if there is a calibration to be removed, remove all observations from this iteration
    if (calibrationItrCnt_ > 1)
    {

        //--- get ownership of mutex
        std::lock_guard<std::mutex> guard(dataProcessingMutex_);

        calibrationItrCnt_--;

        pSrcDataProcessor_->removeCalibIteration(calibrationItrCnt_);
        pRefDataProcessor_->removeCalibIteration(calibrationItrCnt_);

        opRes->is_accepted = true;
        opRes->msg         = "Last observation successfully removed! "
                             "Remaining number of observations: " +
                     std::to_string(pSrcDataProcessor_->getNumCalibIterations()) + " (src), " +
                     std::to_string(pRefDataProcessor_->getNumCalibIterations()) + " (ref).";
    }
    else
    {
        opRes->is_accepted = false;
        opRes->msg         = "No observation available to be removed!";
    }

    RCLCPP_INFO(logger_, "%s", opRes->msg.c_str());

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::initializeServices(
  rclcpp::Node* ipNode)
{
    //--- call parent method
    bool isSuccessful = CalibrationBase::initializeServices(ipNode);

    //--- remove last observation
    pRemoveObsSrv_ = ipNode->create_service<interf::srv::RemoveLastObservation>(
      "~/" + REMOVE_OBSERVATION_SRV_NAME,
      std::bind(&ExtrinsicCalibrationBase::onRequestRemoveObservation, this,
                std::placeholders::_1, std::placeholders::_2));

    //--- calibration meta data
    pCalibMetaDataSrv_ = ipNode->create_service<interf::srv::CalibrationMetaData>(
      "~/" + REQUEST_META_DATA_SRV_NAME,
      std::bind(&ExtrinsicCalibrationBase::onRequestCalibrationMetaData, this,
                std::placeholders::_1, std::placeholders::_2));

    //--- sensor extrinsics
    pSensorExtrinsicsSrv_ = ipNode->create_service<interf::srv::SensorExtrinsics>(
      "~/" + REQUEST_SENSOR_EXTRINSICS_SRV_NAME,
      std::bind(&ExtrinsicCalibrationBase::onRequestSensorExtrinsics, this,
                std::placeholders::_1, std::placeholders::_2));

    return isSuccessful;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::onRequestCalibrationMetaData(
  const std::shared_ptr<interf::srv::CalibrationMetaData::Request> ipReq,
  std::shared_ptr<interf::srv::CalibrationMetaData::Response> opRes)
{
    UNUSED_VAR(ipReq);

    if (!isInitialized_)
        return false;

    opRes->calib_type = static_cast<int>(type_);

    opRes->robot_ws_path = (fs::exists(pRobotWs_->getPath()))
                             ? pRobotWs_->getPath().string()
                             : "";
    opRes->calib_ws_path = (fs::exists(pCalibrationWs_->getPath()))
                             ? pCalibrationWs_->getPath().string()
                             : "";

    opRes->calib_target_file_path = calibTargetFilePath_;

    opRes->src_sensor_name = srcSensorName_;
    opRes->src_topic_name  = srcTopicName_;
    opRes->src_frame_id    = srcFrameId_;

    opRes->ref_sensor_name = refSensorName_;
    opRes->ref_topic_name  = refTopicName_;
    opRes->ref_frame_id    = refFrameId_;

    opRes->base_frame_id = baseFrameId_;

    opRes->is_complete = (!opRes->robot_ws_path.empty() &&
                          !opRes->calib_ws_path.empty() &&
                          !opRes->calib_target_file_path.empty() &&
                          !opRes->src_sensor_name.empty() &&
                          !opRes->src_topic_name.empty() &&
                          !opRes->src_frame_id.empty() &&
                          !opRes->ref_sensor_name.empty() &&
                          !opRes->ref_topic_name.empty() &&
                          !opRes->ref_frame_id.empty());

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::onRequestSensorExtrinsics(
  const std::shared_ptr<interf::srv::SensorExtrinsics::Request> ipReq,
  std::shared_ptr<interf::srv::SensorExtrinsics::Response> opRes)
{
    //--- get empty or last sensor extrinsics and convert into tf2::Transform
    lib3d::Extrinsics extrinsics = (sensorExtrinsics_.empty())
                                     ? lib3d::Extrinsics()
                                     : sensorExtrinsics_.back();

    tf2::Transform ref2LocalTransform;
    utils::setTfTransformFromCameraExtrinsics(extrinsics, ref2LocalTransform);

    //--- transform ref2LocalTransform into request extrinsic type
    if (ipReq->extrinsic_type == interf::srv::SensorExtrinsics::Request::SENSOR_2_SENSOR &&
        !baseFrameId_.empty())
    {
        geometry_msgs::msg::TransformStamped t;
        try
        {
            //--- look up the transform from refFrameId to baseFrameId since, the inverse from
            //--- baseFrameId to refFrameId is required.
            t = tfBuffer_->lookupTransform(baseFrameId_, refFrameId_,
                                           tf2::TimePointZero);
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_ERROR(logger_, "tf2::TransformException: %s",
                         ex.what());
            return false;
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

    //--- construct response
    utils::cvtTfTransform2GeometryPose(ref2LocalTransform, opRes->extrinsics);

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
void ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  publishCalibrationResult(const lib3d::Extrinsics& iSensorExtrinsics) const
{
    // message publishing calibration result
    interf::msg::CalibrationResult calibResultMsg;

    calibResultMsg.is_successful = true;

    calibResultMsg.src_frame_id  = srcFrameId_;
    calibResultMsg.ref_frame_id  = refFrameId_;
    calibResultMsg.base_frame_id = baseFrameId_;

    tf2::Transform ref2LocalTransform;
    utils::setTfTransformFromCameraExtrinsics(iSensorExtrinsics, ref2LocalTransform);
    utils::cvtTfTransform2GeometryTransform(ref2LocalTransform.inverse(), calibResultMsg.sensor_extrinsics);

    pCalibResultPub_->publish(calibResultMsg);
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
void ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  publishLastCalibrationResult() const
{
    publishCalibrationResult(sensorExtrinsics_.back());
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  saveCalibrationSettingsToWorkspace()
{
    if (!CalibrationBase::saveCalibrationSettingsToWorkspace())
        return false;

    QSettings* pCalibSettings = pCalibrationWs_->settingsPtr();
    if (!pCalibSettings)
        return false;

    //--- base frame id
    pCalibSettings->setValue("calibration/base_frame_id",
                             QString::fromStdString(baseFrameId_));

    //--- initial guess
    pCalibSettings->setValue("calibration/use_initial_guess",
                             QVariant::fromValue(useTfTreeAsInitialGuess_));

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
void ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::setupLaunchParameters(
  rclcpp::Node* ipNode) const
{
    CalibrationBase::setupLaunchParameters(ipNode);

    //--- base frame id
    auto baseFrameIdDesc = rcl_interfaces::msg::ParameterDescriptor{};
    baseFrameIdDesc.description =
      "If specified, the extrinsic pose will be calculated with respect to frame of the given "
      "frame ID. This does not change the frame ID of the reference sensor, i.e. the LiDAR sensor, "
      "but will perform an a posteriori transformation of the estimated extrinsic pose into the "
      "specified frame. If not specified, or left empty, the extrinsic pose will be calculated "
      "with respect to the frame of the reference sensor.\n"
      "Default: \"\"";
    baseFrameIdDesc.read_only = true;
    ipNode->declare_parameter<std::string>("base_frame_id", "", baseFrameIdDesc);

    //--- use TF-Tree as initial guess
    auto initialGuessDesc = rcl_interfaces::msg::ParameterDescriptor{};
    initialGuessDesc.description =
      "Option to use an initial guess on the extrinsic sensor pose from the TF-tree, "
      "if available.\n"
      "Default: true";
    initialGuessDesc.read_only = true;
    ipNode->declare_parameter<bool>("use_initial_guess", true, initialGuessDesc);
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::readLaunchParameters(
  const rclcpp::Node* ipNode)
{
    if (!CalibrationBase::readLaunchParameters(ipNode))
        return false;

    //--- base frame id
    baseFrameId_ =
      readStringLaunchParameter(ipNode, "base_frame_id", "");

    //--- use TF-Tree as initial guess
    useTfTreeAsInitialGuess_ = ipNode->get_parameter("use_initial_guess").as_bool();

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
template <typename Id_T, typename Obs_T, typename Cloud_T, typename Pose_T>
void ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  removeObservationsFromIteration(
    const uint& iCalibrationItr,
    std::set<Id_T>& ioIds,
    std::vector<Obs_T>& ioObs,
    std::vector<Cloud_T>& ioClouds,
    std::vector<Pose_T>& ioPoses) const
{
    typename std::set<Id_T>::iterator firstId, lastId;       // start and end position of ids to remove
    typename std::vector<Obs_T>::iterator firstObs, lastObs; // start and end position of obs to remove
    typename std::vector<Cloud_T>::iterator firstCloud;      // start position of cloud to remove
    typename std::vector<Pose_T>::iterator firstPose;        // start position of pose to remove
    bool isStartPosSet = false;
    bool isEndPosSet   = false;
    int startIdIdx     = 0,
        endIdIdx       = 0; // index of start and end element of ids to remove

    //--- find start and end position of ids that are to be removed.
    for (typename std::set<Id_T>::iterator idItr = ioIds.begin();
         idItr != ioIds.end();
         ++idItr)
    {
        //--- set start pos if id is part of calibration iteration and start pos has not yet been set
        if (*idItr >= (iCalibrationItr * 100) &&
            isStartPosSet == false)
        {
            firstId       = idItr;
            startIdIdx    = std::distance(ioIds.begin(), idItr);
            isStartPosSet = true;
        }
        //--- if idItr is part of next calibration iteration and start pos is set, set end pos
        else if (*idItr >= ((iCalibrationItr + 1) * 100) &&
                 isStartPosSet == true &&
                 isEndPosSet == false)
        {
            lastId      = idItr;
            endIdIdx    = std::distance(ioIds.begin(), idItr);
            isEndPosSet = true;
        }
    }

    //--- if at end of list and end pos is not yet set, set to end of list
    if (isEndPosSet == false)
    {
        lastId      = ioIds.end();
        endIdIdx    = std::distance(ioIds.begin(), ioIds.end());
        isEndPosSet = true;
    }

    //--- calculate start and end position of observation to be removed
    firstObs = ioObs.begin() + (startIdIdx * 4);
    lastObs  = ioObs.begin() + (endIdIdx * 4);

    //--- calculate start position of clouds to be removed
    firstCloud = ioClouds.begin() + startIdIdx;

    //--- calculate start position of poses to be removed
    firstPose = ioPoses.begin() + startIdIdx;

    //--- remove elements
    ioIds.erase(firstId, lastId);
    ioObs.erase(firstObs, lastObs);
    ioClouds.erase(firstCloud);
    ioPoses.erase(firstPose);
}
template void ExtrinsicCalibrationBase<CameraDataProcessor, LidarDataProcessor>::
  removeObservationsFromIteration<
    uint, cv::Point2f, pcl::PointCloud<pcl::PointXYZ>::Ptr, lib3d::Extrinsics>(
    const uint&, std::set<uint>&, std::vector<cv::Point2f>&,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<lib3d::Extrinsics>&) const;
template void ExtrinsicCalibrationBase<CameraDataProcessor, LidarDataProcessor>::
  removeObservationsFromIteration<
    uint, cv::Point3f, pcl::PointCloud<pcl::PointXYZ>::Ptr, lib3d::Extrinsics>(
    const uint&, std::set<uint>&, std::vector<cv::Point3f>&,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<lib3d::Extrinsics>&) const;
template void ExtrinsicCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>::
  removeObservationsFromIteration<
    uint, cv::Point2f, pcl::PointCloud<pcl::PointXYZ>::Ptr, lib3d::Extrinsics>(
    const uint&, std::set<uint>&, std::vector<cv::Point2f>&,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<lib3d::Extrinsics>&) const;
template void ExtrinsicCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>::
  removeObservationsFromIteration<
    uint, cv::Point3f, pcl::PointCloud<pcl::PointXYZ>::Ptr, lib3d::Extrinsics>(
    const uint&, std::set<uint>&, std::vector<cv::Point3f>&,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<lib3d::Extrinsics>&) const;
template void ExtrinsicCalibrationBase<LidarDataProcessor, LidarDataProcessor>::
  removeObservationsFromIteration<
    uint, cv::Point3f, pcl::PointCloud<pcl::PointXYZ>::Ptr, lib3d::Extrinsics>(
    const uint&, std::set<uint>&, std::vector<cv::Point3f>&,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<lib3d::Extrinsics>&) const;
template void ExtrinsicCalibrationBase<LidarDataProcessor, ReferenceDataProcessor3d>::
  removeObservationsFromIteration<
    uint, cv::Point3f, pcl::PointCloud<pcl::PointXYZ>::Ptr, lib3d::Extrinsics>(
    const uint&, std::set<uint>&, std::vector<cv::Point3f>&,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<lib3d::Extrinsics>&) const;

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
template <typename Id_T, typename Obs_T>
void ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  removeCornerObservationsWithoutCorrespondence(
    const std::set<Id_T>& iReferenceIds,
    std::set<Id_T>& ioSrcIds,
    std::vector<Obs_T>& ioSrcObs) const
{
    //--- loop over ioSrcIds and check if corresponding ID is also in
    //--- iReferenceIds
    //--- if not remove correspondence
    //--- since the sets are ordered, it can be iterated in parallel from front to back
    typename std::set<Id_T>::iterator refIdItr = // iterator of reference id list
      iReferenceIds.begin();
    typename std::set<Id_T>::iterator srcIdItr = // iterator of source id list
      ioSrcIds.begin();
    while (srcIdItr != ioSrcIds.end())
    {
        //--- if both ref ID and src ID are equal, increment iterators and continue
        if (*refIdItr == *srcIdItr)
        {
            ++refIdItr;
            ++srcIdItr;
            continue;
        }

        //--- compute index if src iterator and remove id and observations
        uint srcIdx        = std::distance(ioSrcIds.begin(), srcIdItr);
        auto eraseStartPos = ioSrcObs.begin() + (srcIdx * 4);
        ioSrcObs.erase(eraseStartPos, eraseStartPos + 4);
        srcIdItr = ioSrcIds.erase(srcIdItr);
    }
}
template void ExtrinsicCalibrationBase<CameraDataProcessor, LidarDataProcessor>::
  removeCornerObservationsWithoutCorrespondence<uint, cv::Point2f>(
    const std::set<uint>&, std::set<uint>&, std::vector<cv::Point2f>& ioSrcObs) const;
template void ExtrinsicCalibrationBase<CameraDataProcessor, LidarDataProcessor>::
  removeCornerObservationsWithoutCorrespondence<uint, cv::Point3f>(
    const std::set<uint>&, std::set<uint>&, std::vector<cv::Point3f>& ioSrcObs) const;
template void ExtrinsicCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>::
  removeCornerObservationsWithoutCorrespondence<uint, cv::Point2f>(
    const std::set<uint>&, std::set<uint>&, std::vector<cv::Point2f>& ioSrcObs) const;
template void ExtrinsicCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>::
  removeCornerObservationsWithoutCorrespondence<uint, cv::Point3f>(
    const std::set<uint>&, std::set<uint>&, std::vector<cv::Point3f>& ioSrcObs) const;
template void ExtrinsicCalibrationBase<LidarDataProcessor, LidarDataProcessor>::
  removeCornerObservationsWithoutCorrespondence<uint, cv::Point3f>(
    const std::set<uint>&, std::set<uint>&, std::vector<cv::Point3f>& ioSrcObs) const;
template void ExtrinsicCalibrationBase<LidarDataProcessor, ReferenceDataProcessor3d>::
  removeCornerObservationsWithoutCorrespondence<uint, cv::Point3f>(
    const std::set<uint>&, std::set<uint>&, std::vector<cv::Point3f>& ioSrcObs) const;

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
void ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::reset()
{
    //--- call method of parent class
    CalibrationBase::reset();

    sensorExtrinsics_ = std::vector<lib3d::Extrinsics>(1, lib3d::Extrinsics());
    calibResult_      = CalibrationResult();
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::saveCalibration()
{
    //--- save results to calibration workspace
    if (!saveResultsToCalibrationWorkspace())
    {
        RCLCPP_WARN(logger_, "Something went wrong while writing results to calibration workspace. "
                             "Workspace: %s",
                    pCalibrationWs_->getPath().string().c_str());
    }
    else
    {
        RCLCPP_INFO(logger_, "Writing results to calibration workspace: Successful!");
    }

    //--- save to URDF model
    if (isUrdfModelAvailable_)
    {
        if (!isFrameIdInUrdfModel(srcFrameId_))
        {
            RCLCPP_WARN(logger_, "Source Frame ID is not available as link in the URDF model file. "
                                 "Results are not written to URDF model file. "
                                 "Frame ID: %s",
                        srcFrameId_.c_str());
        }
        else if (!isFrameIdInUrdfModel((!baseFrameId_.empty()) ? baseFrameId_ : refFrameId_))
        {
            RCLCPP_WARN(logger_,
                        "Base/Reference Frame ID is not available as link in the URDF model file. "
                        "Results are not written to URDF model file. "
                        "Frame ID: %s",
                        (!baseFrameId_.empty()) ? baseFrameId_.c_str() : refFrameId_.c_str());
        }
        else if (!saveCalibrationToUrdfModel())
        {
            RCLCPP_WARN(logger_, "Something went wrong while writing results to URDF model file. "
                                 "URDF model file: %s",
                        urdfModelPath_.string().c_str());
        }
        else
        {
            RCLCPP_INFO(logger_, "Writing results to URDF model file: Successful!");
        }
    }

    //--- save observations
    if (saveObservationsToWs_)
    {
        if (!saveObservationsToCalibrationWorkspace())
        {
            RCLCPP_WARN(logger_, "Something went wrong while writing observations to calibration "
                                 "workspace. "
                                 "Workspace: %s",
                        pCalibrationWs_->getPath().string().c_str());
        }
        else
        {
            RCLCPP_INFO(logger_, "Writing observations to calibration workspace: Successful!");
        }
    }

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::saveCalibrationToUrdfModel()
{
    // Pointer to the XML element of the desired joint node
    tinyxml2::XMLElement* pJointElement = nullptr;

    // Pointer to the XML element of the child node of the desired joint, i.e. source sensor
    tinyxml2::XMLElement* pChildElement = nullptr;

    // Pointer to the XML element of the parent node of the desired joint, i.e. reference sensor
    tinyxml2::XMLElement* pParentElement = nullptr;

    // Pointer to the XML element of the origin node of the desired joint, i.e. transformation between reference and source
    tinyxml2::XMLElement* pOriginElement = nullptr;

    // Indicator if data has been inserted into xml doc
    std::vector<bool> isDataInserted = std::vector<bool>(calibResult_.calibrations.size(), false);

    for (uint i = 0; i < calibResult_.calibrations.size(); ++i)
    {
        //--- loop over <joint> nodes within urdf XML file
        pJointElement = urdfModelDoc_.RootElement()->FirstChildElement("joint");
        while (pJointElement && !isDataInserted[i])
        {
            //--- loop over <child> nodes within joint
            pChildElement = pJointElement->FirstChildElement("child");
            while (pChildElement && !isDataInserted[i])
            {
                //--- if the link 'link' attribute of the <child> has the same name as the source sensor,
                //--- get <parent> and <origin> node
                if (pChildElement->Attribute("link") == calibResult_.calibrations[i].srcFrameId)
                {
                    pParentElement = pJointElement->FirstChildElement("parent");
                    pOriginElement = pJointElement->FirstChildElement("origin");

                    //--- write result into xml tree
                    if (pParentElement != nullptr && pOriginElement != nullptr)
                    {
                        pParentElement->SetAttribute(
                          "link", (calibResult_.calibrations[i].baseFrameId.empty())
                                    ? calibResult_.calibrations[i].refFrameId.c_str()
                                    : calibResult_.calibrations[i].baseFrameId.c_str());

                        std::stringstream xyzStrStream;
                        xyzStrStream << calibResult_.calibrations[i].XYZ.x() << " "
                                     << calibResult_.calibrations[i].XYZ.y() << " "
                                     << calibResult_.calibrations[i].XYZ.z();
                        pOriginElement->SetAttribute("xyz", xyzStrStream.str().c_str());

                        std::stringstream rpyStrStream;
                        rpyStrStream << calibResult_.calibrations[i].RPY.x() << " "
                                     << calibResult_.calibrations[i].RPY.y() << " "
                                     << calibResult_.calibrations[i].RPY.z();
                        pOriginElement->SetAttribute("rpy", rpyStrStream.str().c_str());

                        isDataInserted[i] = true;
                    }
                }
                pChildElement = pChildElement->NextSiblingElement("child");
            }
            pJointElement = pJointElement->NextSiblingElement("joint");
        }
    }

    for (auto success : isDataInserted)
        if (!success)
            return false;

    //--- backup urdf model file
    utils::backupFile(urdfModelPath_);

    //--- save new XML Document
    urdfModelDoc_.SaveFile(urdfModelPath_.c_str());

    //--- reload URDF Model
    urdfModel_.clear();
    urdfModel_.initFile(urdfModelPath_);

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  saveObservationsToCalibrationWorkspace() const
{
    bool isSuccessful = true;

    isSuccessful &= pSrcDataProcessor_->saveObservations(
      pCalibrationWs_->getPath().append(OBSERVATIONS_SUBDIR_NAME));
    isSuccessful &= pRefDataProcessor_->saveObservations(
      pCalibrationWs_->getPath().append(OBSERVATIONS_SUBDIR_NAME));

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  saveResultsToCalibrationWorkspace() const
{
    // lambda function to write content to file
    auto writeToFile = [&](const std::string& filename, const std::string& content) -> bool
    {
        // absolute path to results file
        fs::path resultsFilePath = pCalibrationWs_->getPath().append(filename);

        //--- write results to file
        std::fstream ofStream(resultsFilePath, std::ios_base::out);
        if (ofStream.is_open())
        {
            ofStream << content;
            ofStream.close();
        }
        else
        {
            return false;
        }

        return true;
    };

    bool isSuccesful = true;

    isSuccesful &= writeToFile(CALIB_RESULTS_FILE_NAME, calibResult_.toString());
    isSuccesful &= writeToFile(URDF_SNIPPET_FILE_NAME, calibResult_.urdfSnippet());

    return isSuccesful;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  setSensorExtrinsicsFromFrameIds(const std::string& iSourceFrameId,
                                  const std::string& iReferenceFrameId)
{
    //--- initialize extrinsics
    if (tfBuffer_->_frameExists(iSourceFrameId) &&
        tfBuffer_->_frameExists(iReferenceFrameId))
    {
        try
        {
            auto t = tfBuffer_->lookupTransform(iSourceFrameId, iReferenceFrameId,
                                                tf2::TimePointZero);

            tf2::Transform transform(tf2::Quaternion(t.transform.rotation.x,
                                                     t.transform.rotation.y,
                                                     t.transform.rotation.z,
                                                     t.transform.rotation.w),
                                     tf2::Vector3(t.transform.translation.x,
                                                  t.transform.translation.y,
                                                  t.transform.translation.z));
            utils::setCameraExtrinsicsFromTfTransform(transform,
                                                      sensorExtrinsics_.back());
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_ERROR(logger_, "tf2::TransformException: %s",
                         ex.what());
            return false;
        }
    }
    else
    {
        RCLCPP_WARN(logger_, "Frame %s or frame %s does not exists! "
                             "Initializing extrinsic transformation with null rotation and "
                             "translation.",
                    iSourceFrameId.c_str(), iReferenceFrameId.c_str());
        sensorExtrinsics_.back() = lib3d::Extrinsics();
        return false;
    }

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
std::pair<tf2::Vector3, tf2::Vector3> ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  computeTargetPoseStdDev(
    const std::vector<lib3d::Extrinsics>& iSrcTargetPoses,
    const std::vector<lib3d::Extrinsics>& iRefTargetPoses) const
{
    //--- get min num overlapping target poses
    const uint MIN_TARGET_POSES = std::min(iSrcTargetPoses.size(), iRefTargetPoses.size());

    //--- if only one overlapping target pose is available, return maximum deviation
    if (MIN_TARGET_POSES <= 1)
    {
        return std::make_pair(tf2::Vector3(FLT_MAX, FLT_MAX, FLT_MAX),
                              tf2::Vector3(FLT_MAX, FLT_MAX, FLT_MAX));
    }

    //--- get RT Matrix of last extrinsic pose to transform source sensor into reference
    const cv::Matx44d EXTR_TRANSF_MAT = sensorExtrinsics_.back().getRTMatrix(
      lib3d::Extrinsics::LOCAL_2_REF);

    // list of differences in xyz and rpy of target poses
    std::vector<tf2::Vector3> xyz_differences, rpy_differences;

    // mean values of differences in xyz of target poses
    tf2::Vector3 xyz_difference_mean = tf2::Vector3(0, 0, 0);

    // mean values of differences in rpy of target poses
    tf2::Vector3 rpy_difference_mean = tf2::Vector3(0, 0, 0);

    //--- loop over number of overlapping target poses
    for (uint i = 0; i < MIN_TARGET_POSES; ++i)
    {
        //--- transform source target pose into frame of the reference sensor based on the last
        //--- computed extrinsics
        lib3d::Extrinsics transSrcPose(lib3d::Extrinsics::LOCAL_2_REF);
        transSrcPose.setRTMatrix(
          EXTR_TRANSF_MAT * iSrcTargetPoses[i].getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF));

        // tf2::Transform of the transformed source pose and the ref pose.
        tf2::Transform transSrcPoseTransform, refPoseTransform;

        //--- get tf2::Transforms from lib3d::Extrinsics
        utils::setTfTransformFromCameraExtrinsics(transSrcPose, transSrcPoseTransform);
        utils::setTfTransformFromCameraExtrinsics(iRefTargetPoses[i], refPoseTransform);

        //--- compute and push back difference in XYZ
        xyz_differences.push_back(transSrcPoseTransform.getOrigin() - refPoseTransform.getOrigin());

        //--- compute and push back difference in RPY
        double sRoll, sPitch, sYaw, rRoll, rPitch, rYaw;
        transSrcPoseTransform.getBasis().getRPY(sRoll, sPitch, sYaw);
        refPoseTransform.getBasis().getRPY(rRoll, rPitch, rYaw);
        rpy_differences.push_back(tf2::Vector3(sRoll - rRoll, sPitch - rPitch, sYaw - rYaw));

        //--- add previously computed difference to running mean of XYZ and RPY
        xyz_difference_mean += xyz_differences.back();
        rpy_difference_mean += rpy_differences.back();
    }

    //--- normalize running mean values
    xyz_difference_mean /= MIN_TARGET_POSES;
    rpy_difference_mean /= MIN_TARGET_POSES;

    tf2::Vector3 xyz_var = tf2::Vector3(0, 0, 0); // variance in XYZ of pose
    tf2::Vector3 rpy_var = tf2::Vector3(0, 0, 0); // variance in RPY of pose

    //--- compute variance
    for (uint i = 0; i < MIN_TARGET_POSES; ++i)
    {
        xyz_var += tf2::Vector3(std::pow(xyz_differences[i].x() - xyz_difference_mean.x(), 2.f),
                                std::pow(xyz_differences[i].y() - xyz_difference_mean.y(), 2.f),
                                std::pow(xyz_differences[i].z() - xyz_difference_mean.z(), 2.f));

        rpy_var += tf2::Vector3(std::pow(rpy_differences[i].x() - rpy_difference_mean.x(), 2.f),
                                std::pow(rpy_differences[i].y() - rpy_difference_mean.y(), 2.f),
                                std::pow(rpy_differences[i].z() - rpy_difference_mean.z(), 2.f));
    }
    xyz_var /= MIN_TARGET_POSES;
    rpy_var /= MIN_TARGET_POSES;

    //--- compute and return standard deviation
    return std::make_pair(
      tf2::Vector3(std::sqrt(xyz_var.x()), std::sqrt(xyz_var.y()), std::sqrt(xyz_var.z())),
      tf2::Vector3(lib3d::radianToDegree(std::sqrt(rpy_var.x())),
                   lib3d::radianToDegree(std::sqrt(rpy_var.y())),
                   lib3d::radianToDegree(std::sqrt(rpy_var.z()))));
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
std::string ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::CalibrationResult::
  toString() const
{
    std::stringstream strStream;

    for (auto calib : calibrations)
    {
        strStream << "Transformation from";
        if (!calib.baseFrameId.empty())
            strStream << "\n  base frame (Frame ID: " << calib.baseFrameId << ") as parent";
        else
            strStream << "\n  '" << calib.refSensorName
                      << "' (Frame ID: " << calib.refFrameId << ") as parent";
        strStream << "\nto";
        strStream << "\n  '" << calib.srcSensorName
                  << "' (Frame ID: " << calib.srcFrameId << ") as child:";
        strStream << "\n\t> XYZ: "
                  << calib.XYZ.x() << " " << calib.XYZ.y() << " " << calib.XYZ.z();
        strStream << "\n\t> RPY: "
                  << calib.RPY.x() << " " << calib.RPY.y() << " " << calib.RPY.z();
        strStream << "\n- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n";
    }
    strStream << "\nNumber of observations: " << numObservations;
    strStream << std::setprecision(4) << std::fixed
              << "\n"
              << error.first << ": " << error.second;

    strStream << std::setprecision(4) << std::fixed
              << "\nDeviation in poses of calibration target"
              << "\nwhen transformed between sensor frames:";

    strStream << "\n\t> XYZ (in m): ";
    if (!std::isnan(target_poses_stdDev.first.length()))
    {
        strStream << target_poses_stdDev.first.x() << " "
                  << target_poses_stdDev.first.y() << " "
                  << target_poses_stdDev.first.z();
    }
    else
    {
        strStream << "n/a";
    }

    strStream << "\n\t> RPY (in Deg.): ";
    if (!std::isnan(target_poses_stdDev.second.length()))
    {
        strStream << target_poses_stdDev.second.x() << " "
                  << target_poses_stdDev.second.y() << " "
                  << target_poses_stdDev.second.z();
    }
    else
    {
        strStream << "n/a";
    }

    return strStream.str();
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
std::string ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::CalibrationResult::
  urdfSnippet() const
{
    std::stringstream strStream;
    for (auto calib : calibrations)
    {
        strStream << "<joint name=\"" << calib.srcSensorName << "_joint\" type=\"fixed\">";
        strStream << "\n\t<parent link=\""
                  << ((!calib.baseFrameId.empty()) ? calib.baseFrameId : calib.refFrameId)
                  << "\"/>";
        strStream << "\n\t<child link=\"" << calib.srcFrameId << "\"/>";
        strStream << "\n\t<origin xyz=\""
                  << calib.XYZ.x() << " " << calib.XYZ.y() << " " << calib.XYZ.z() << "\" "
                  << "rpy=\""
                  << calib.RPY.x() << " " << calib.RPY.y() << " " << calib.RPY.z() << "\"/>";
        strStream << "\n</joint>";
        strStream << "\n<link name=\"" << calib.srcFrameId << "\">";
        strStream << "\n</link>";

        strStream << ((calibrations.size() > 1) ? "\n\n" : "");
    }

    return strStream.str();
}

template class ExtrinsicCalibrationBase<CameraDataProcessor, LidarDataProcessor>;
template class ExtrinsicCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>;
template class ExtrinsicCalibrationBase<LidarDataProcessor, LidarDataProcessor>;
template class ExtrinsicCalibrationBase<LidarDataProcessor, ReferenceDataProcessor3d>;

} // namespace multisensor_calibration