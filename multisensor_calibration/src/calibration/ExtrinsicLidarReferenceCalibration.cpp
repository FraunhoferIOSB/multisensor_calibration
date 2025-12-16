/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/calibration/ExtrinsicLidarReferenceCalibration.h"

// Std
#include <functional>
#include <future>

// PCL
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

// Qt
#include <QFile>

// small_gicp
#include <small_gicp/registration/registration_helper.hpp>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"

namespace multisensor_calibration
{
//==================================================================================================
ExtrinsicLidarReferenceCalibration::ExtrinsicLidarReferenceCalibration(
  const std::string& nodeName,
  const rclcpp::NodeOptions& options) :
  Extrinsic3d3dCalibrationBase<LidarDataProcessor, ReferenceDataProcessor3d>(
    EXTRINSIC_LIDAR_REFERENCE_CALIBRATION),
  rclcpp::Node(nodeName, options),
  pSrcCloudSubsc_(nullptr)
{
    //--- do base class initialization
    CalibrationBase::init(this);
}

//==================================================================================================
ExtrinsicLidarReferenceCalibration::ExtrinsicLidarReferenceCalibration(
  const rclcpp::NodeOptions& options) :
  ExtrinsicLidarReferenceCalibration(
    CALIB_TYPE_2_NODE_NAME.at(EXTRINSIC_LIDAR_REFERENCE_CALIBRATION),
    options)
{
}

//==================================================================================================
ExtrinsicLidarReferenceCalibration::~ExtrinsicLidarReferenceCalibration()
{
    pSrcLidarDataProcessor_.reset();
    pRefDataProcessor_.reset();
}

//==================================================================================================
bool ExtrinsicLidarReferenceCalibration::finalizeCalibration()
{

    //--- calculate coarse extrinsic pose based on the marker observations
    //--- in case of LiDAR-LiDAR calibration, this is done after the detection of the target
    //--- when a single iteration is calibrated
    //--- but since, this is not done in case of the LiDAR-Reference calibration, it needs to
    //--- be done prior to the final calibration

    //--- copy marker corner observations to point cloud
    //--- make uneven in order to remove ambiguities
    pcl::PointCloud<pcl::PointXYZ>::Ptr pSrcMarkerCornerCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pRefMarkerCornerCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (uint i = 1; i < calibrationItrCnt_; ++i)
    {
        //--- get observations from source LiDAR
        std::set<uint> srcObservationIds;
        std::vector<cv::Point3f> srcCornerObservations;
        pSrcLidarDataProcessor_->getOrderedObservations(srcObservationIds, srcCornerObservations,
                                                        i, 1);

        //--- get observations from reference LiDAR
        std::set<uint> refObservationIds;
        std::vector<cv::Point3f> refCornerObservations;
        pRefDataProcessor_->getOrderedObservations(refObservationIds, refCornerObservations,
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

    if (pSrcMarkerCornerCloud->empty() || pRefMarkerCornerCloud->empty())
    {
        RCLCPP_ERROR(logger_, "Could not finalize calibration. No common observations available.");
        return false;
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

    //--- do fine calibration with icp
    //--- in this, first merge target clouds

    // pointer to cloud holding calibration target clouds of all observations from the src lidar
    pcl::PointCloud<InputPointType>::Ptr pSrcLidarTargetClouds(
      new pcl::PointCloud<InputPointType>());

    // pointer to cloud holding calibration target clouds of all observations from the ref lidar
    pcl::PointCloud<InputPointType>::Ptr pRefTargetClouds(
      new pcl::PointCloud<InputPointType>());

    auto srcLidarTargetCloudPtrs = pSrcLidarDataProcessor_->getCalibrationTargetCloudPtrs();
    for (pcl::PointCloud<InputPointType>::Ptr pSubCloud : srcLidarTargetCloudPtrs)
        pSrcLidarTargetClouds->insert(pSrcLidarTargetClouds->end(),
                                      pSubCloud->begin(),
                                      pSubCloud->end());
    auto refLidarTargetCloudPtrs = pRefDataProcessor_->getCalibrationTargetCloudPtrs();
    for (pcl::PointCloud<InputPointType>::Ptr pSubCloud : refLidarTargetCloudPtrs)
        pRefTargetClouds->insert(pRefTargetClouds->end(),
                                 pSubCloud->begin(),
                                 pSubCloud->end());

    //--- run ICP
    double icpRmse = runIcp<InputPointType>(
      pSrcLidarTargetClouds, pRefTargetClouds,
      static_cast<small_gicp::RegistrationSetting::RegistrationType>(
        registrationParams_.registration_icp_variant.value),
      registrationParams_.registration_icp_max_correspondence_distance.value,
      registrationParams_.registration_icp_rotation_tolerance.value,
      registrationParams_.registration_icp_translation_tolerance.value);

    ExtrinsicCalibrationBase::updateCalibrationResult(std::make_pair("Root Mean Squared Error (in m)", icpRmse), static_cast<int>(pRefDataProcessor_->getNumCalibIterations()));

    //--- publish last sensor extrinsics
    ExtrinsicCalibrationBase::publishLastCalibrationResult();

    return true;
}

//==================================================================================================
bool ExtrinsicLidarReferenceCalibration::initializeDataProcessors()
{
    bool isSuccessful = true;

    //--- initialize source lidar data processor
    pSrcLidarDataProcessor_.reset(
      new LidarDataProcessor(logger_.get_name(), srcSensorName_, calibTargetFilePath_));
    if (pSrcLidarDataProcessor_)
    {
        pSrcLidarDataProcessor_->initializeServices(this);
        pSrcLidarDataProcessor_->initializePublishers(this);
        pSrcLidarDataProcessor_->setParameters(lidarTargetDetectionParams_);
    }
    else
    {
        isSuccessful = false;
    }

    //--- initialize reference data processor
    pRefDataProcessor_.reset(
      new ReferenceDataProcessor3d(logger_.get_name(), referenceName_, calibTargetFilePath_));
    if (pRefDataProcessor_)
    {
        pRefDataProcessor_->initializeServices(this);
        pRefDataProcessor_->initializePublishers(this);
    }
    else
    {
        isSuccessful = false;
    }

    return isSuccessful;
}

//==================================================================================================
bool ExtrinsicLidarReferenceCalibration::initializeSubscribers(rclcpp::Node* ipNode)
{
    //--- subscribe to topic from source sensor
    pSrcCloudSubsc_ =
      ipNode->create_subscription<InputCloud_Message_T>(
        srcLidarCloudTopic_, 1,
        std::bind(&ExtrinsicLidarReferenceCalibration::onSensorDataReceived, this,
                  std::placeholders::_1));

    return true;
}

//==================================================================================================
bool ExtrinsicLidarReferenceCalibration::initializeWorkspaceObjects()
{
    bool retVal = true;

    retVal &= CalibrationBase::initializeWorkspaceObjects();

    //--- initialize calibration workspace
    fs::path calibWsPath = robotWsPath_;
    calibWsPath /=
      std::string(srcLidarSensorName_ + "_" + refSensorName_ + "_extrinsic_calibration");
    pCalibrationWs_ =
      std::make_shared<ExtrinsicLidarReferenceCalibWorkspace>(calibWsPath, logger_);
    retVal &= (pCalibrationWs_ != nullptr);

    if (pCalibrationWs_)
    {
        //--- create backup
        retVal &= pCalibrationWs_->createBackup();
    }

    return retVal;
}

//==================================================================================================
void ExtrinsicLidarReferenceCalibration::onSensorDataReceived(
  const InputCloud_Message_T::ConstSharedPtr& ipSrcCloudMsg)
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
    if (pRefDataProcessor_ == nullptr)
    {
        RCLCPP_ERROR(logger_, "Reference data processor is not initialized.");
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

    if (!isConversionSuccessful)
    {
        RCLCPP_ERROR(logger_,
                     "Something went wrong in getting the sensor data from the input messages.");
        return;
    }

    //--- set frame ids and initialize sensor extrinsics if applicable
    if (srcCloudFrameId_ != ipSrcCloudMsg->header.frame_id)
    {
        srcCloudFrameId_ = ipSrcCloudMsg->header.frame_id;

        //--- compute sensor extrinsics between source frame id and ref or base frame id
        //--- if base frame id is not empty and unequal to refCloudFrameId also get transform
        //--- between refFrameID and baseFrameID and pass to refLidarProcessor.
        if (!baseFrameId_.empty() && baseFrameId_ != refFrameId_)
        {
            if (useTfTreeAsInitialGuess_)
                setSensorExtrinsicsFromFrameIds(srcCloudFrameId_, baseFrameId_);

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
                    RCLCPP_ERROR(logger_, "tf2::TransformException: %s",
                                 ex.what());
                }
            }
            else
            {
                RCLCPP_WARN(logger_, "Base Frame %s does not exists! "
                                     "Removing base frame and calibrating relative "
                                     "to reference cloud.",
                            baseFrameId_.c_str());
                baseFrameId_ = "";
                pRefDataProcessor_->setDataTransform(nullptr);
            }
        }
        else
        {
            if (useTfTreeAsInitialGuess_)
                setSensorExtrinsicsFromFrameIds(srcCloudFrameId_, refFrameId_);
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

    //--- wait for processing to return
    LidarDataProcessor::EProcessingResult srcLidarProcResult = srcLidarProcFuture.get();

    //--- if processing level is set to preview, publish preview from each sensor if successful
    if (procLevel == LidarDataProcessor::PREVIEW)
    {
        if (srcLidarProcResult == LidarDataProcessor::SUCCESS)
            pSrcLidarDataProcessor_->publishPreview(ipSrcCloudMsg->header);
    }
    //--- else if, processing level is target_detection,
    //--- calibrate only if processing for both sensors is successful
    else if (procLevel == LidarDataProcessor::TARGET_DETECTION)
    {
        interf::msg::CalibrationResult calibResultMsg;

        if (srcLidarProcResult == LidarDataProcessor::SUCCESS)
        {
            //--- publish detections
            pSrcLidarDataProcessor_->publishLastTargetDetection(ipSrcCloudMsg->header);

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

    //--- if data processor is not pending for more data, set capturing flag to false
    if (srcLidarProcResult != LidarDataProcessor::PENDING)
    {
        captureCalibrationTarget_ = false;
    }
}

//==================================================================================================
bool ExtrinsicLidarReferenceCalibration::saveCalibrationSettingsToWorkspace()
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

    //--- reference name
    pCalibSettings->setValue("reference/name",
                             QString::fromStdString(referenceName_));

    //--- reference frame id
    pCalibSettings->setValue("reference/frame_id",
                             QString::fromStdString(refFrameId_));

    //--- sync settings file
    pCalibSettings->sync();

    return true;
}

//==================================================================================================
void ExtrinsicLidarReferenceCalibration::setupLaunchParameters(rclcpp::Node* ipNode) const
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
void ExtrinsicLidarReferenceCalibration::setupDynamicParameters(rclcpp::Node* ipNode) const
{
    registrationParams_.declareDynamic(ipNode);
    lidarTargetDetectionParams_.declareDynamic(ipNode);
}

//==================================================================================================
bool ExtrinsicLidarReferenceCalibration::readLaunchParameters(const rclcpp::Node* ipNode)
{
    if (!ExtrinsicCalibrationBase::readLaunchParameters(ipNode))
        return false;

    //--- source_lidar_sensor_name
    srcLidarSensorName_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "src_lidar_sensor_name", DEFAULT_LIDAR_SENSOR_NAME);

    //--- source_lidar_cloud_topic
    srcLidarCloudTopic_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "src_lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC);

    //--- reference_name
    referenceName_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "reference_name", "reference");

    //--- ref_lidar_sensor_name
    refFrameId_ = CalibrationBase::readStringLaunchParameter(
      ipNode, "reference_frame_id", "reference");

    //--- need to set ref in order for the calibration meta data to be complete. However,
    //--- this is not evaluated
    refTopicName_ = "/cloud";

    //--- initial guess is not supported for sensor-reference calibration
    useTfTreeAsInitialGuess_ = false;

    return true;
}

//==================================================================================================
bool ExtrinsicLidarReferenceCalibration::setDynamicParameter(const rclcpp::Parameter& iParameter)
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
        return true;
    }
    else
    {
        return false;
    }
}

//==================================================================================================
void ExtrinsicLidarReferenceCalibration::reset()
{
    ExtrinsicCalibrationBase::reset();

    pSrcLidarDataProcessor_->reset();
    pSrcLidarDataProcessor_->setPreprocFilter(nullptr);
    pRefDataProcessor_->reset();
}

//==================================================================================================
bool ExtrinsicLidarReferenceCalibration::shutdownSubscribers()
{
    //--- check if node is initialized
    if (!isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(dataProcessingMutex_);

    //--- unsubscribe subscribers
    pSrcCloudSubsc_.reset();
    pSrcCloudSubsc_ = nullptr;

    return true;
}

} // namespace multisensor_calibration

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(multisensor_calibration::ExtrinsicLidarReferenceCalibration)
