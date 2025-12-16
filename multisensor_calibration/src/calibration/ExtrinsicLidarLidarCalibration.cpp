/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/calibration/ExtrinsicLidarLidarCalibration.h"

// Std
#include <functional>
#include <future>

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

// ROS
#include <tf2/LinearMath/Transform.hpp>

// Qt
#include <QFile>

// small_gicp
#include <small_gicp/registration/registration_helper.hpp>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/common/utils.hpp"
#include <multisensor_calibration_interface/msg/calibration_result.hpp>

namespace multisensor_calibration
{
using namespace utils;

//==================================================================================================
ExtrinsicLidarLidarCalibration::
  ExtrinsicLidarLidarCalibration(const std::string& nodeName,
                                 const rclcpp::NodeOptions& options) :
  Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>(
    EXTRINSIC_LIDAR_LIDAR_CALIBRATION),
  rclcpp::Node(nodeName, options),
  pCloudCloudApproxSync_(nullptr),
  pCloudCloudExactSync_(nullptr),
  alignGroundPlanes_(false),
  uprightFrameId_(""),
  syncQueueSize_(DEFAULT_SYNC_QUEUE_SIZE),
  useExactSync_(false)
{
    CalibrationBase::init(this);
}

//==================================================================================================
ExtrinsicLidarLidarCalibration::ExtrinsicLidarLidarCalibration(const rclcpp::NodeOptions& options) :
  ExtrinsicLidarLidarCalibration(
    CALIB_TYPE_2_NODE_NAME.at(EXTRINSIC_LIDAR_LIDAR_CALIBRATION),
    options)
{
}

//==================================================================================================
ExtrinsicLidarLidarCalibration::~ExtrinsicLidarLidarCalibration()
{
    //--- reset pointers message filters before sensor processors in order to avoid seg fault during
    //--- disconnection of callbacks.
    pCloudCloudApproxSync_.reset();
    pCloudCloudExactSync_.reset();
    pSrcLidarDataProcessor_.reset();
    pRefLidarDataProcessor_.reset();
}

//==================================================================================================
void ExtrinsicLidarLidarCalibration::calibrateLastObservation()
{
    using namespace pcl::registration;

    //--- check that for this calibration iteration, both camera and lidar observations are available
    //--- i.e. if list is empty, or if last item in list is smaller than iterationCnt * 100, return
    if (pSrcLidarDataProcessor_->getNumCalibIterations() < calibrationItrCnt_ ||
        pRefLidarDataProcessor_->getNumCalibIterations() < calibrationItrCnt_)
        return;

    //--- copy marker corner observations to point cloud
    //--- make uneven in order to remove ambiguities
    pcl::PointCloud<pcl::PointXYZ>::Ptr pSrcMarkerCornerCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pRefMarkerCornerCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (uint i = 1; i <= calibrationItrCnt_; ++i)
    {
        //--- get observations from source LiDAR
        std::set<uint> srcObservationIds;
        std::vector<cv::Point3f> srcCornerObservations;
        pSrcLidarDataProcessor_->getOrderedObservations(srcObservationIds, srcCornerObservations,
                                                        i, 1);

        //--- get observations from reference LiDAR
        std::set<uint> refObservationIds;
        std::vector<cv::Point3f> refCornerObservations;
        pRefLidarDataProcessor_->getOrderedObservations(refObservationIds, refCornerObservations,
                                                        i, 1);

        //--- remove observations that do not have a correspondence in the other list
        removeCornerObservationsWithoutCorrespondence(srcObservationIds,
                                                      refObservationIds,
                                                      refCornerObservations);
        removeCornerObservationsWithoutCorrespondence(refObservationIds,
                                                      srcObservationIds,
                                                      srcCornerObservations);

        //--- if number of IDs is even (i.e. even number of markers), remove id and corners marker
        //--- with id of first correspondences to make orientation unique
        auto makeUneven = [&](std::set<uint>& ids, std::vector<cv::Point3f>& corners,
                              const uint& idToRemove)
        {
            if ((ids.size() % 2) == 0)
            {
                // find it to be removed in list
                std::set<uint>::iterator idItr = std::find(ids.begin(), ids.end(), idToRemove);
                if (idItr == ids.end())
                    return;

                //--- compute index if src iterator and remove id and observations
                uint idIdx               = std::distance(ids.begin(), idItr);
                auto cornersEaseStartPos = corners.begin() + (idIdx * 4);

                ids.erase(idItr);
                corners.erase(cornersEaseStartPos, cornersEaseStartPos + 4);
            }
        };
        uint idToRemove = *srcObservationIds.begin();
        makeUneven(srcObservationIds, srcCornerObservations, idToRemove);
        makeUneven(refObservationIds, refCornerObservations, idToRemove);

        //--- push remaining to point cloud
        auto pushToCloud = [&](const std::vector<cv::Point3f>& cornerVec,
                               pcl::PointCloud<pcl::PointXYZ>& cornerCloud)
        {
            for (auto itr = cornerVec.cbegin(); itr != cornerVec.cend(); ++itr)
            {
                cornerCloud.push_back(pcl::PointXYZ(itr->x, itr->y, itr->z));
            }
        };
        pushToCloud(srcCornerObservations, *pSrcMarkerCornerCloud);
        pushToCloud(refCornerObservations, *pRefMarkerCornerCloud);
    }

    // Correspondences of marker corners between src and ref cloud. Since the marker corners are
    // extracted in order, the correspondence list simply consists of increasing index numbers
    pcl::Correspondences correspondences;
    for (uint i = 0; i < pSrcMarkerCornerCloud->size(); ++i)
    {
        correspondences.push_back(pcl::Correspondence(i, i, 1.f));
    }

    //--- estimate sensor extrinsics from marker corners
    auto sensorExtrinsic = computeExtrinsicsFromPointCorrespondences<pcl::PointXYZ>(
      pSrcMarkerCornerCloud, pRefMarkerCornerCloud, correspondences);
    sensorExtrinsics_.push_back(sensorExtrinsic);

    //--- increment iteration count
    calibrationItrCnt_++;

    //--- publish last sensor extrinsics
    ExtrinsicCalibrationBase::publishLastCalibrationResult();
}

//==================================================================================================
bool ExtrinsicLidarLidarCalibration::finalizeCalibration()
{

    //--- merge target clouds

    // pointer to cloud holding calibration target clouds of all observations from the src lidar
    pcl::PointCloud<InputPointType>::Ptr pSrcLidarTargetClouds(
      new pcl::PointCloud<InputPointType>());

    // pointer to cloud holding calibration target clouds of all observations from the ref lidar
    pcl::PointCloud<InputPointType>::Ptr pRefLidarTargetClouds(
      new pcl::PointCloud<InputPointType>());

    auto srcLidarTargetCloudPtrs = pSrcLidarDataProcessor_->getCalibrationTargetCloudPtrs();
    for (pcl::PointCloud<InputPointType>::Ptr pSubCloud : srcLidarTargetCloudPtrs)
        pSrcLidarTargetClouds->insert(pSrcLidarTargetClouds->end(),
                                      pSubCloud->begin(),
                                      pSubCloud->end());
    auto refLidarTargetCloudPtrs = pRefLidarDataProcessor_->getCalibrationTargetCloudPtrs();
    for (pcl::PointCloud<InputPointType>::Ptr pSubCloud : refLidarTargetCloudPtrs)
        pRefLidarTargetClouds->insert(pRefLidarTargetClouds->end(),
                                      pSubCloud->begin(),
                                      pSubCloud->end());

    if (pSrcLidarTargetClouds->empty() || pRefLidarTargetClouds->empty())
    {
        RCLCPP_ERROR(logger_, "Could not finalize calibration. "
                              "No common observations available.");
        return false;
    }

    // list of indices of target points in source cloud
    pcl::IndicesPtr pSrcTargetIndices = nullptr;

    // list of indices of target points in reference cloud
    pcl::IndicesPtr pRefTargetIndices = nullptr;

    //--- additionally align ground planes if flag
    //--- whether an upright frame is specified, is already checked in 'readLaunchParameters'
    if (alignGroundPlanes_)
    {

        //--- fill list of point indices lying on targets with increasing number starting from 0
        //--- this is needed to only compute RMSE between target points and not to incorporate the
        //--- ground plane
        //--- to this end, only the points on the targets are in the corresponding point clouds, thus,
        //--- the size of the point clouds can be used as upper bound of the indices.
        pSrcTargetIndices.reset(new pcl::Indices(pSrcLidarTargetClouds->size()));
        std::iota(std::begin(*pSrcTargetIndices), std::end(*pSrcTargetIndices), 0);
        pRefTargetIndices.reset(new pcl::Indices(pRefLidarTargetClouds->size()));
        std::iota(std::begin(*pRefTargetIndices), std::end(*pRefTargetIndices), 0);

        //--- check if specified upright frame exists
        if (tfBuffer_->_frameExists(uprightFrameId_))
        {
            geometry_msgs::msg::TransformStamped srcTransfMsg, refTransfMsg;

            bool isLookupSuccessful;
            try
            {
                srcTransfMsg = tfBuffer_->lookupTransform(srcCloudFrameId_, uprightFrameId_,
                                                          tf2::TimePointZero);
                refTransfMsg = tfBuffer_->lookupTransform(
                  ((baseFrameId_.empty()) ? refCloudFrameId_ : baseFrameId_),
                  uprightFrameId_, tf2::TimePointZero);

                isLookupSuccessful = true;
            }
            catch (tf2::TransformException& ex)
            {
                RCLCPP_ERROR(logger_, "tf2::TransformException in trying to get transform between "
                                      "upright frame and sensor frames or base frame: %s"
                                      "\nAlignment of ground planes will be skipped.",
                             ex.what());
                isLookupSuccessful = false;
            }

            //--- if lookup is successful, find ground planes and add to point clouds that are used
            //--- for alignment
            if (isLookupSuccessful)
            {
                tf2::Transform srcTransform(tf2::Quaternion(srcTransfMsg.transform.rotation.x,
                                                            srcTransfMsg.transform.rotation.y,
                                                            srcTransfMsg.transform.rotation.z,
                                                            srcTransfMsg.transform.rotation.w),
                                            tf2::Vector3(srcTransfMsg.transform.translation.x,
                                                         srcTransfMsg.transform.translation.y,
                                                         srcTransfMsg.transform.translation.z));
                tf2::Transform refTransform(tf2::Quaternion(refTransfMsg.transform.rotation.x,
                                                            refTransfMsg.transform.rotation.y,
                                                            refTransfMsg.transform.rotation.z,
                                                            refTransfMsg.transform.rotation.w),
                                            tf2::Vector3(refTransfMsg.transform.translation.x,
                                                         refTransfMsg.transform.translation.y,
                                                         refTransfMsg.transform.translation.z));

                //--- find upright vector in frame of sensor data by transforming the z-axis with
                //--- the rotational part of the transform between the upright frame and the
                //--- sensor frames.
                tf2::Vector3 srcUprightVec = srcTransform.getBasis() * tf2::Vector3(0.f, 0.f, 1.f);
                srcUprightVec.normalize();
                tf2::Vector3 refUprightVec = refTransform.getBasis() * tf2::Vector3(0.f, 0.f, 1.f);
                refUprightVec.normalize();

                // Pointer to ground plane cloud of reference data.
                pcl::PointCloud<InputPointType>::Ptr pRefGroundPlane(
                  new pcl::PointCloud<InputPointType>);

                // Pointer to ground plane cloud of source data.
                pcl::PointCloud<InputPointType>::Ptr pSrcGroundPlane(
                  new pcl::PointCloud<InputPointType>);

                //--- extract ground planes in sensor data with an angle tolerance of 5 degrees
                //--- to the upright vectors.
                LidarDataProcessor::extractPlaneFromPointCloud(
                  pRefLidarDataProcessor_->getLastInputCloud(), refUprightVec, 5, pRefGroundPlane);
                LidarDataProcessor::extractPlaneFromPointCloud(
                  pSrcLidarDataProcessor_->getLastInputCloud(), srcUprightVec, 5, pSrcGroundPlane);

                //--- insert the extracted ground planes into the point clouds used for the ICP
                pRefLidarTargetClouds->insert(pRefLidarTargetClouds->end(),
                                              pRefGroundPlane->begin(),
                                              pRefGroundPlane->end());

                pSrcLidarTargetClouds->insert(pSrcLidarTargetClouds->end(),
                                              pSrcGroundPlane->begin(),
                                              pSrcGroundPlane->end());
            }
        }
        else
        {
            RCLCPP_ERROR(logger_, "Specified upright frame with the id '%s' does not exist."
                                  "\nAlignment of ground planes will be skipped.",
                         uprightFrameId_.c_str());
        }
    }

    //--- run ICP
    runIcp<InputPointType>(
      pSrcLidarTargetClouds, pRefLidarTargetClouds,
      static_cast<small_gicp::RegistrationSetting::RegistrationType>(
        registrationParams_.registration_icp_variant.value),
      registrationParams_.registration_icp_max_correspondence_distance.value,
      registrationParams_.registration_icp_rotation_tolerance.value,
      registrationParams_.registration_icp_translation_tolerance.value);

    //--- calculate RMSE
    //--- if no ground plane is used, the pointer to the indices will be nullptr, thus, the full
    //--- cloud will be used
    double rmse = utils::calculateRootMeanSquaredError<InputPointType, InputPointType>(
      pSrcLidarTargetClouds, pRefLidarTargetClouds,
      ExtrinsicCalibrationBase::sensorExtrinsics_.back(),
      pSrcTargetIndices, pRefTargetIndices);
    calibResult_.error = std::make_pair("Root Mean Squared Error (in m)", rmse);

    //--- computer target pose deviation if more than 1 observation is available
    if (pSrcDataProcessor_->getNumCalibIterations() > 1 &&
        pRefDataProcessor_->getNumCalibIterations() > 1)
    {
        calibResult_.target_poses_stdDev =
          computeTargetPoseStdDev(pSrcDataProcessor_->getCalibrationTargetPoses(),
                                  pRefDataProcessor_->getCalibrationTargetPoses());
    }

    //--- set calibration meta data
    calibResult_.calibrations[0].srcSensorName = srcSensorName_;
    calibResult_.calibrations[0].srcFrameId    = srcFrameId_;
    calibResult_.calibrations[0].refSensorName = refSensorName_;
    calibResult_.calibrations[0].refFrameId    = refFrameId_;
    calibResult_.calibrations[0].baseFrameId   = baseFrameId_;

    //--- get extrinsics
    const lib3d::Extrinsics& extrinsics = sensorExtrinsics_.back();

    //--- get transformation from lib3d::Extrinsics.
    // resulting transformation from ref to src sensor
    tf2::Transform refToSrcTransform;
    setTfTransformFromCameraExtrinsics(extrinsics,
                                       refToSrcTransform);
    calibResult_.calibrations[0].XYZ = refToSrcTransform.inverse().getOrigin(); // invert to get LOCAL_2_REF
    double roll, pitch, yaw;
    refToSrcTransform.inverse().getBasis().getRPY(roll, pitch, yaw); // invert to get LOCAL_2_REF
    calibResult_.calibrations[0].RPY = tf2::Vector3(roll, pitch, yaw);

    //--- store meta information into calibResult
    calibResult_.numObservations = static_cast<int>(pRefDataProcessor_->getNumCalibIterations());

    //--- print out final transformation
    RCLCPP_INFO(logger_,
                "\n==================================================================================="
                "\n%s"
                "\n===================================================================================",
                calibResult_.toString().c_str());

    //--- publish last sensor extrinsics
    ExtrinsicCalibrationBase::publishLastCalibrationResult();

    return true;
}

//==================================================================================================
bool ExtrinsicLidarLidarCalibration::initializeDataProcessors()
{

    bool isSuccessful = true;

    // Lambda function to initialize pointer to LidarDataProcessor
    auto initializeLidarDataProcessor = [&](std::shared_ptr<LidarDataProcessor>& iopProcessor,
                                            const std::string& iSensorName)
    {
        iopProcessor.reset(
          new LidarDataProcessor(logger_.get_name(), iSensorName, calibTargetFilePath_));
        if (iopProcessor)
        {
            iopProcessor->initializeServices(this);
            iopProcessor->initializePublishers(this);
            iopProcessor->setParameters(lidarTargetDetectionParams_);
        }
        else
        {
            isSuccessful = false;
        }
    };

    //--- initialize data processors
    initializeLidarDataProcessor(pSrcLidarDataProcessor_, srcSensorName_);
    initializeLidarDataProcessor(pRefLidarDataProcessor_, refSensorName_);

    return isSuccessful;
}

//==================================================================================================
bool ExtrinsicLidarLidarCalibration::initializeSubscribers(rclcpp::Node* ipNode)
{
    //--- subscribe to topics
    srcCloudSubsc_.subscribe(ipNode, srcLidarCloudTopic_);
    refCloudSubsc_.subscribe(ipNode, refLidarCloudTopic_);

    //--- initialize synchronizers
    if (useExactSync_)
    {
        pCloudCloudExactSync_ =
          std::make_shared<message_filters::Synchronizer<CloudCloudExactSync>>(
            CloudCloudExactSync(10), srcCloudSubsc_, refCloudSubsc_);
        pCloudCloudExactSync_->registerCallback(
          std::bind(&ExtrinsicLidarLidarCalibration::onSensorDataReceived, this,
                    std::placeholders::_1, std::placeholders::_2));
    }
    else
    {
        pCloudCloudApproxSync_ =
          std::make_shared<message_filters::Synchronizer<CloudCloudApproxSync>>(
            CloudCloudApproxSync(syncQueueSize_), srcCloudSubsc_, refCloudSubsc_);
        pCloudCloudApproxSync_->registerCallback(
          std::bind(&ExtrinsicLidarLidarCalibration::onSensorDataReceived, this,
                    std::placeholders::_1, std::placeholders::_2));
    }

    return true;
}

//==================================================================================================
bool ExtrinsicLidarLidarCalibration::initializeWorkspaceObjects()
{
    bool retVal = true;

    retVal &= CalibrationBase::initializeWorkspaceObjects();

    //--- initialize calibration workspace
    fs::path calibWsPath = robotWsPath_;
    calibWsPath /=
      std::string(srcLidarSensorName_ + "_" + refLidarSensorName_ + "_extrinsic_calibration");
    pCalibrationWs_ =
      std::make_shared<ExtrinsicLidarLidarCalibWorkspace>(calibWsPath, logger_);
    retVal &= (pCalibrationWs_ != nullptr);

    if (pCalibrationWs_)
    {
        //--- create backup
        retVal &= pCalibrationWs_->createBackup();
    }

    return retVal;
}

//==================================================================================================
bool ExtrinsicLidarLidarCalibration::onRequestRemoveObservation(
  const std::shared_ptr<interf::srv::RemoveLastObservation::Request> ipReq,
  std::shared_ptr<interf::srv::RemoveLastObservation::Response> opRes)
{
    if (ExtrinsicCalibrationBase<LidarDataProcessor, LidarDataProcessor>::onRequestRemoveObservation(ipReq, opRes))
        sensorExtrinsics_.pop_back();
    else
        return false;

    return true;
}

//==================================================================================================
void ExtrinsicLidarLidarCalibration::onSensorDataReceived(
  const InputCloud_Message_T::ConstSharedPtr& ipSrcCloudMsg,
  const InputCloud_Message_T::ConstSharedPtr& ipRefCloudMsg)
{
    //--- check if node is initialized
    if (!isInitialized_)
    {
        RCLCPP_ERROR(logger_, "Node is not initialized.");
        return;
    }
    if (pSrcLidarDataProcessor_ == nullptr)
    {
        RCLCPP_ERROR(logger_, "Source lidar data processor is not initialized.");
        return;
    }
    if (pRefLidarDataProcessor_ == nullptr)
    {
        RCLCPP_ERROR(logger_, "Reference lidar data processor is not initialized.");
        return;
    }

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(dataProcessingMutex_);

    //--- convert input messages to data
    bool isConversionSuccessful = true;

    // source point cloud
    pcl::PointCloud<InputPointType> srcPointCloud;
    isConversionSuccessful &=
      pSrcLidarDataProcessor_->getSensorDataFromMsg(ipSrcCloudMsg, srcPointCloud);

    // reference point cloud
    pcl::PointCloud<InputPointType> refPointCloud;
    isConversionSuccessful &=
      pRefLidarDataProcessor_->getSensorDataFromMsg(ipRefCloudMsg, refPointCloud);

    if (!isConversionSuccessful)
    {
        RCLCPP_ERROR(logger_,
                     "Something went wrong in getting the sensor data from the input messages.");
        return;
    }

    //--- set frame ids and initialize sensor extrinsics if applicable
    if (srcCloudFrameId_ != ipSrcCloudMsg->header.frame_id ||
        refCloudFrameId_ != ipRefCloudMsg->header.frame_id)
    {
        srcCloudFrameId_ = ipSrcCloudMsg->header.frame_id;
        refCloudFrameId_ = ipRefCloudMsg->header.frame_id;

        //--- compute sensor extrinsics between source frame id and ref or base frame id
        //--- if base frame id is not empty and unequal to refCloudFrameId also get transform
        //--- between refFrameID and baseFrameID and pass to refLidarProcessor.
        if (!baseFrameId_.empty() && baseFrameId_ != refCloudFrameId_)
        {
            if (useTfTreeAsInitialGuess_)
                setSensorExtrinsicsFromFrameIds(srcCloudFrameId_, baseFrameId_);

            if (tfBuffer_->_frameExists(baseFrameId_))
            {
                try
                {

                    auto t = tfBuffer_->lookupTransform(baseFrameId_, refFrameId_,
                                                        tf2::TimePointZero);
                    pRefLidarDataProcessor_->setDataTransform(
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
                                 "to reference LiDAR data processor: %s",
                                 ex.what());
                }
            }
            else
            {
                RCLCPP_WARN(logger_, "Base Frame '%s' does not exists! "
                                     "Removing base frame and calibrating relative "
                                     "to reference cloud.",
                            baseFrameId_.c_str());
                baseFrameId_ = "";
                pRefLidarDataProcessor_->setDataTransform(nullptr);
            }
        }
        else
        {
            if (useTfTreeAsInitialGuess_)
                setSensorExtrinsicsFromFrameIds(srcCloudFrameId_, refCloudFrameId_);
        }
    }

    // Level at which to do the processing
    LidarDataProcessor::EProcessingLevel procLevel = (captureCalibrationTarget_)
                                                       ? LidarDataProcessor::TARGET_DETECTION
                                                       : LidarDataProcessor::PREVIEW;

    //--- process data from source lidar asynchronously
    std::future<LidarDataProcessor::EProcessingResult> srcLidarProcFuture =
      std::async(&LidarDataProcessor::processData,
                 pSrcLidarDataProcessor_,
                 srcPointCloud,
                 procLevel);

    //--- process data from reference lidar asynchronously
    std::future<LidarDataProcessor::EProcessingResult> refLidarProcFuture =
      std::async(&LidarDataProcessor::processData,
                 pRefLidarDataProcessor_,
                 refPointCloud,
                 procLevel);

    //--- wait for processing to return
    LidarDataProcessor::EProcessingResult srcLidarProcResult = srcLidarProcFuture.get();
    LidarDataProcessor::EProcessingResult refLidarProcResult = refLidarProcFuture.get();

    //--- if processing level is set to preview, publish preview from each sensor if successful
    if (procLevel == LidarDataProcessor::PREVIEW)
    {
        if (srcLidarProcResult == LidarDataProcessor::SUCCESS)
            pSrcLidarDataProcessor_->publishPreview(ipSrcCloudMsg->header);
        if (refLidarProcResult == LidarDataProcessor::SUCCESS)
            pRefLidarDataProcessor_->publishPreview(ipRefCloudMsg->header.stamp,
                                                    (baseFrameId_.empty())
                                                      ? refFrameId_
                                                      : baseFrameId_);
    }
    //--- else if, processing level is target_detection,
    //--- calibrate only if processing for both sensors is successful
    else if (procLevel == LidarDataProcessor::TARGET_DETECTION)
    {
        if (srcLidarProcResult == LidarDataProcessor::SUCCESS &&
            refLidarProcResult == LidarDataProcessor::SUCCESS)
        {
            //--- publish detections
            pSrcLidarDataProcessor_->publishLastTargetDetection(ipSrcCloudMsg->header);
            pRefLidarDataProcessor_->publishLastTargetDetection(ipRefCloudMsg->header.stamp,
                                                                (baseFrameId_.empty())
                                                                  ? refFrameId_
                                                                  : baseFrameId_);

            //--- perform single calibration iteration
            //--- To this end, this only removes observations without correspondence.
            calibrateLastObservation();
        }
        else
        {
            if (srcLidarProcResult != LidarDataProcessor::SUCCESS &&
                refLidarProcResult == LidarDataProcessor::SUCCESS)
                pRefLidarDataProcessor_->removeCalibIteration(calibrationItrCnt_);

            if (srcLidarProcResult == LidarDataProcessor::SUCCESS &&
                refLidarProcResult != LidarDataProcessor::SUCCESS)
                pSrcLidarDataProcessor_->removeCalibIteration(calibrationItrCnt_);

            interf::msg::CalibrationResult calibResultMsg;
            calibResultMsg.is_successful = false;
            pCalibResultPub_->publish(calibResultMsg);
        }
    }

    //--- if this point is reachted, the target detection was not successful.
    //--- thus, if data processor is not pending for more data, set capturing flag to false.
    if (srcLidarProcResult != LidarDataProcessor::PENDING &&
        refLidarProcResult != LidarDataProcessor::PENDING)
    {
        captureCalibrationTarget_ = false;
    }
}

//==================================================================================================
bool ExtrinsicLidarLidarCalibration::saveCalibrationSettingsToWorkspace()
{
    if (!ExtrinsicCalibrationBase::saveCalibrationSettingsToWorkspace())
        return false;

    QSettings* pCalibSettings = pCalibrationWs_->settingsPtr();
    if (!pCalibSettings)
        return false;

    //--- source lidar sensor name
    pCalibSettings->setValue("source_lidar/sensor_name",
                             QString::fromStdString(srcLidarSensorName_));

    //--- source lidar image topic
    pCalibSettings->setValue("source_lidar/cloud_topic",
                             QString::fromStdString(srcLidarCloudTopic_));

    //--- reference lidar sensor name
    pCalibSettings->setValue("reference_lidar/sensor_name",
                             QString::fromStdString(refLidarSensorName_));

    //--- reference lidar image topic
    pCalibSettings->setValue("reference_lidar/cloud_topic",
                             QString::fromStdString(refLidarCloudTopic_));

    //--- align_ground_planes
    pCalibSettings->setValue("calibration/align_ground_planes",
                             QVariant::fromValue(alignGroundPlanes_));

    //--- upright frame id
    pCalibSettings->setValue("calibration/upright_frame_id",
                             QString::fromStdString(uprightFrameId_));

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
void ExtrinsicLidarLidarCalibration::setupLaunchParameters(rclcpp::Node* ipNode) const
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

    //--- ref_lidar_sensor_name
    auto refSensorNameDesc = rcl_interfaces::msg::ParameterDescriptor{};
    refSensorNameDesc.description =
      "Name of the reference LiDAR sensor with respect to which the source LiDAR sensor "
      "is to be calibrated.\n "
      "Default: \"lidar\"";
    refSensorNameDesc.read_only = true;
    ipNode->declare_parameter<std::string>("ref_lidar_sensor_name", DEFAULT_LIDAR_SENSOR_NAME,
                                           refSensorNameDesc);

    //--- ref_lidar_cloud_topic
    auto refCloudTopicDesc = rcl_interfaces::msg::ParameterDescriptor{};
    refCloudTopicDesc.description =
      "Topic name of the corresponding LiDAR cloud.\n"
      "Default: \"/lidar/cloud\"";
    refCloudTopicDesc.read_only = true;
    ipNode->declare_parameter<std::string>("ref_lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC,
                                           refCloudTopicDesc);

    //--- align_groun_planes
    auto alignGroundPlanesDesc = rcl_interfaces::msg::ParameterDescriptor{};
    alignGroundPlanesDesc.description =
      "Set to true, to additionally align the ground planes in the sensor"
      "data.Additionally specify the upright frame ID.\n "
      "Default: false";
    alignGroundPlanesDesc.read_only = true;
    ipNode->declare_parameter<bool>("align_ground_planes", false, alignGroundPlanesDesc);

    //--- upright_frame_id
    auto uprightFrameIdDesc = rcl_interfaces::msg::ParameterDescriptor{};
    uprightFrameIdDesc.description =
      "ID of Frame which has an upwards pointing z-axis. Used to detect ground"
      "plane in sensor data.\n"
      "Default: \"\"";
    uprightFrameIdDesc.read_only = true;
    ipNode->declare_parameter<std::string>("upright_frame_id", "",
                                           uprightFrameIdDesc);

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
void ExtrinsicLidarLidarCalibration::setupDynamicParameters(rclcpp::Node* ipNode) const
{
    registrationParams_.declareDynamic(ipNode);
    lidarTargetDetectionParams_.declareDynamic(ipNode);
}

//==================================================================================================
bool ExtrinsicLidarLidarCalibration::readLaunchParameters(const rclcpp::Node* ipNode)
{
    if (!ExtrinsicCalibrationBase::readLaunchParameters(ipNode))
        return false;

    //--- src_lidar_sensor_name
    srcLidarSensorName_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "src_lidar_sensor_name", DEFAULT_LIDAR_SENSOR_NAME);

    //--- src_lidar_cloud_topic
    srcLidarCloudTopic_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "src_lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC);

    //--- ref_lidar_sensor_name
    refLidarSensorName_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "ref_lidar_sensor_name", DEFAULT_LIDAR_SENSOR_NAME);

    //--- ref_lidar_cloud_topic
    refLidarCloudTopic_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "ref_lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC);

    //--- align_ground_planes
    alignGroundPlanes_ = ipNode->get_parameter("align_ground_planes").as_bool();

    //--- upright frame id
    uprightFrameId_ = CalibrationBase::readStringLaunchParameter(ipNode, "upright_frame_id", "");
    if (alignGroundPlanes_ && uprightFrameId_.empty())
    {
        RCLCPP_WARN(logger_, "'align_ground_planes' is activated but 'upright_frame_id' is empty.'"
                             "\nThe alignment of the ground planes will be deactivated."
                             "\nPlease specify a ID of a frame that has an upright z-axes.");
    }

    //--- sync queue
    syncQueueSize_ = CalibrationBase::readNumericLaunchParameter<int>(
      ipNode, "sync_queue_size", DEFAULT_SYNC_QUEUE_SIZE, 1, INT_MAX);

    //--- exact sync
    useExactSync_ = ipNode->get_parameter("use_exact_sync").as_bool();

    return true;
}

//==================================================================================================
bool ExtrinsicLidarLidarCalibration::setDynamicParameter(const rclcpp::Parameter& iParameter)
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
        pSrcLidarDataProcessor_->setParameters(lidarTargetDetectionParams_);
        pRefLidarDataProcessor_->setParameters(lidarTargetDetectionParams_);
        return true;
    }
    else
    {
        return false;
    }
}

//==================================================================================================
void ExtrinsicLidarLidarCalibration::reset()
{
    ExtrinsicCalibrationBase::reset();

    pSrcLidarDataProcessor_->reset();
    pSrcLidarDataProcessor_->setPreprocFilter(nullptr);
    pRefLidarDataProcessor_->reset();
    pRefLidarDataProcessor_->setPreprocFilter(nullptr);
}

//==================================================================================================
bool ExtrinsicLidarLidarCalibration::shutdownSubscribers()
{
    //--- check if node is initialized
    if (!CalibrationBase::isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(CalibrationBase::dataProcessingMutex_);

    //--- unsubscribe subscribers
    srcCloudSubsc_.unsubscribe();
    refCloudSubsc_.unsubscribe();

    return true;
}

} // namespace multisensor_calibration

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(multisensor_calibration::ExtrinsicLidarLidarCalibration)
