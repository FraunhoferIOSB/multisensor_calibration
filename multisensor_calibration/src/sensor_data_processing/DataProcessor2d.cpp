/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/sensor_data_processing/DataProcessor2d.h"

// Std
#include <fstream>

// ROS
#include <cv_bridge/cv_bridge.hpp>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/filters/passthrough.h>

// OpenCV
#include <opencv2/imgcodecs.hpp>

namespace multisensor_calibration
{

//==================================================================================================
DataProcessor2d::DataProcessor2d(const std::string& iLoggerName,
                                 const std::string& iSensorName,
                                 const fs::path& iCalibTargetFilePath) :
  SensorDataProcessorBase<cv::Mat>(iLoggerName,
                                   iSensorName,
                                   iCalibTargetFilePath),
  annotatedCameraImgPreview_(cv::Mat()),
  annotatedCameraImageDetection_(),
  capturedMarkerIds_(),
  capturedMarkerCorners_(),
  calibrationTargetCloudPtrs_(),
  pMarkerCornersPub_(nullptr),
  pTargetCloudPub_(nullptr),
  pTargetBoardPosePub_(nullptr)
{
}

//==================================================================================================
DataProcessor2d::~DataProcessor2d()
{
}

//==================================================================================================
cv::Mat DataProcessor2d::getAnnotatedCameraImagePreview() const
{
    return annotatedCameraImgPreview_;
}

//==================================================================================================
pcl::PointCloud<pcl::PointXYZRGB>::Ptr DataProcessor2d::getLastCalibrationTargetCloudPtr() const
{
    return calibrationTargetCloudPtrs_.back();
}

//==================================================================================================
std::vector<std::array<cv::Point2f, 4>> DataProcessor2d::getLastCapturedMarkerCorners() const
{
    return capturedMarkerCorners_.back();
}

//==================================================================================================
std::vector<int> DataProcessor2d::getLastCapturedMarkerIds() const
{
    return capturedMarkerIds_.back();
}

//==================================================================================================
void DataProcessor2d::getOrderedObservations(std::set<uint>& oObservationIds,
                                             std::vector<cv::Point2f>& oCornerObservations,
                                             const int& iIterationBegin,
                                             const int& iNumIterations) const
{
    oObservationIds.clear();
    oCornerObservations.clear();

    //--- if iteration begin is larger than the amount of iterations available, return
    if (iIterationBegin > static_cast<int>(capturedMarkerIds_.size()))
        return;

    //--- set begin iterator
    auto capturedMarkerIdsItr = capturedMarkerIds_.cbegin() + (iIterationBegin - 1);

    // number of iterations
    uint tmpNumIterations = (iNumIterations <= 0) ? capturedMarkerIds_.size() : iNumIterations;

    //--- loop through iterations
    for (uint i = 0;
         i < tmpNumIterations && capturedMarkerIdsItr != capturedMarkerIds_.end();
         ++i, ++capturedMarkerIdsItr)
    {
        auto calibrationItrIdx = std::distance(capturedMarkerIds_.cbegin(), capturedMarkerIdsItr);

        //--- loop through all markers in this iteration
        for (uint m = 0; m < capturedMarkerIds_[calibrationItrIdx].size(); ++m)
        {
            int markerId = capturedMarkerIds_[calibrationItrIdx][m];

            //--- construct observationId, insert into list and get position
            uint obsId = (calibrationItrIdx + 1) * 100 + markerId; // observation id
            std::pair<std::set<uint>::iterator, bool> retVal =
              oObservationIds.insert(obsId);

            // index of marker observation in list
            auto obsIdx = std::distance(oObservationIds.begin(), retVal.first);

            //--- loop over corners and insert into list
            for (uchar c = 0; c < 4; ++c)
            {
                cv::Point2f markerCorner = capturedMarkerCorners_[calibrationItrIdx][m][c];

                // insertion position
                auto insertionPos = oCornerObservations.begin() + obsIdx * 4 + c;
                oCornerObservations.insert(insertionPos, markerCorner);
            }
        }
    }
}

//==================================================================================================
bool DataProcessor2d::getSensorDataFromMsg(const InputImage_Message_T::ConstSharedPtr& ipMsg,
                                           cv::Mat& oData) const
{
    //--- get image from message
    cv_bridge::CvImageConstPtr pCvImg;
    try
    {
        pCvImg = cv_bridge::toCvShare(ipMsg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_FATAL(logger_, "cv_bridge::Exception: %s", e.what());
        return false;
    }

    //--- convert to bgr or simply copy to member variable
    if (pCvImg->encoding == "mono8")
        cv::cvtColor(pCvImg->image, oData, CV_GRAY2BGR);
    else if (pCvImg->encoding == "rgb8")
        cv::cvtColor(pCvImg->image, oData, CV_RGB2BGR);
    else
        pCvImg->image.copyTo(oData);

    return true;
}

//==================================================================================================
bool DataProcessor2d::initializePublishers(rclcpp::Node* ipNode)
{
    // lambda to append sensorName_ (if not empty) to topic name
    auto constructTopicName = [&](const std::string& iTopicName) -> std::string
    {
        return "~/" + ((sensorName_.empty()) ? iTopicName : (sensorName_ + "/" + iTopicName));
    };

    pAnnotatedImgPub_ = ipNode->create_publisher<AnnotatedImage_Message_T>(constructTopicName(ANNOTATED_CAMERA_IMAGE_TOPIC_NAME), 10);

    //--- advertise topics for marker corners and target cloud
    pMarkerCornersPub_ = ipNode->create_publisher<MarkerCornersImgPnts_Message_T>(
      constructTopicName(MARKER_CORNERS_TOPIC_NAME), 10);

    pTargetCloudPub_ = ipNode->create_publisher<TargetPatternCloud_Message_T>(
      constructTopicName(TARGET_PATTERN_CLOUD_TOPIC_NAME), 10);

    pTargetBoardPosePub_ = ipNode->create_publisher<TargetBoardPose_Message_T>(
      constructTopicName(TARGET_BOARD_POSE_TOPIC_NAME), 10);

    return true;
}

//==================================================================================================
bool DataProcessor2d::initializeServices(rclcpp::Node* ipNode)
{
    UNUSED_VAR(ipNode);

    return true;
}

//==================================================================================================
void DataProcessor2d::publishPreview(const std_msgs::msg::Header& iHeader) const
{
    //--- publish annotated image
    AnnotatedImage_Message_T::SharedPtr annotatedImgMsg =
      cv_bridge::CvImage(iHeader, "bgr8", annotatedCameraImgPreview_)
        .toImageMsg();
    pAnnotatedImgPub_->publish(*annotatedImgMsg);
}

//==================================================================================================
void DataProcessor2d::publishLastTargetDetection(const std_msgs::msg::Header& iHeader) const
{
    //--- publish annotated image
    publishPreview(iHeader);

    //--- publish target point cloud
    if (calibrationTargetCloudPtrs_.back())
    {
        TargetPatternCloud_Message_T cloudMsg;
        pcl::toROSMsg(*calibrationTargetCloudPtrs_.back(), cloudMsg);
        cloudMsg.header = iHeader;
        pTargetCloudPub_->publish(cloudMsg);

        RCLCPP_INFO(logger_,
                    "Published target cloud! Number of Points: %ld",
                    calibrationTargetCloudPtrs_.back()->size());
    }

    //--- Publish multi-array with IDs and image points of detected markers

    // ROS message holding image points of marker corners
    MarkerCornersImgPnts_Message_T imgPntsMsg;
    imgPntsMsg.header = iHeader;
    imgPntsMsg.array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    imgPntsMsg.array.layout.dim[0].label  = "markers";
    imgPntsMsg.array.layout.dim[0].size   = capturedMarkerIds_.size();
    imgPntsMsg.array.layout.dim[0].stride = 9; // id + four corners (x,y)
    imgPntsMsg.array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    imgPntsMsg.array.layout.dim[1].label  = "data";
    imgPntsMsg.array.layout.dim[1].size   = imgPntsMsg.array.layout.dim[0].stride;
    imgPntsMsg.array.layout.dim[1].stride = 1;

    imgPntsMsg.array.data.resize(static_cast<unsigned int>(imgPntsMsg.array.layout.dim[0].size *
                                                           imgPntsMsg.array.layout.dim[1].size));
    int dataItr = 0;
    for (uint i = 0; i < capturedMarkerIds_.size(); ++i)
    {
        imgPntsMsg.array.data[dataItr++] = capturedMarkerIds_.back()[i];

        for (uchar j = 0; j < 4; ++j)
        {
            imgPntsMsg.array.data[dataItr++] = capturedMarkerCorners_.back()[i][j].x;
            imgPntsMsg.array.data[dataItr++] = capturedMarkerCorners_.back()[i][j].y;
        }
    }

    pMarkerCornersPub_->publish(imgPntsMsg);

    RCLCPP_INFO(logger_, "Published image points of marker corners!");

    //--- Publish multi-array with up vector and center of detected target
    publishLastCalibrationTargetPose(iHeader, capturedCalibTargetPoses_.back(), true,
                                     pTargetBoardPosePub_);
}

//==================================================================================================
bool DataProcessor2d::removeCalibIteration(const uint& iIterationId)
{
    bool isSuccessful = SensorDataProcessorBase::removeCalibIteration(iIterationId);

    if (!isSuccessful)
        return false;

    annotatedCameraImageDetection_.erase(annotatedCameraImageDetection_.begin() +
                                         (iIterationId - 1));
    capturedMarkerIds_.erase(capturedMarkerIds_.begin() +
                             (iIterationId - 1));
    capturedMarkerCorners_.erase(capturedMarkerCorners_.begin() +
                                 (iIterationId - 1));
    calibrationTargetCloudPtrs_.erase(calibrationTargetCloudPtrs_.begin() +
                                      (iIterationId - 1));

    return true;
}

//==================================================================================================
void DataProcessor2d::reset()
{
    SensorDataProcessorBase::reset();

    annotatedCameraImageDetection_.clear();
    capturedMarkerIds_.clear();
    capturedMarkerCorners_.clear();
    calibrationTargetCloudPtrs_.clear();
}

//==================================================================================================
bool DataProcessor2d::saveObservations(fs::path iOutputPath) const
{
    bool isSuccessful = SensorDataProcessorBase::saveObservations(iOutputPath);
    if (!isSuccessful)
        return false;

    //--- loop through calibration iterations
    for (uint calibItr = 0; calibItr < capturedCalibTargetPoses_.size(); ++calibItr)
    {
        try
        {
            // directory of each calibration iteration
            fs::path iterationOutputPath = iOutputPath;
            iterationOutputPath.append(std::to_string(calibItr + 1));
            if (!fs::exists(iterationOutputPath))
                fs::create_directories(iterationOutputPath);

            //--- store annotated image
            cv::imwrite(iterationOutputPath.string() + "/" + sensorName_ + ANNOTATED_IMAGE_FILE_SUFFIX,
                        annotatedCameraImageDetection_[calibItr]);

            //--- marker corner observations
            writeMarkerObservationsToFile(
              iterationOutputPath.string() + "/" + sensorName_ + OBSERVATIONS_FILE_SUFFIX,
              capturedMarkerIds_[calibItr], capturedMarkerCorners_[calibItr]);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(logger_, "std::exception: %s", e.what());
            isSuccessful = false;
        }
    }

    return isSuccessful;
}

//==================================================================================================
void DataProcessor2d::writeMarkerObservationsToFile(
  const fs::path& iFilePath,
  const std::vector<int>& iMarkerIds,
  const std::vector<std::array<cv::Point2f, 4>>& iMarkerCorners)
{
    try
    {
        std::fstream observationsFout(iFilePath, std::ios_base::out);
        observationsFout
          << "# 2D image points of the detected marker corners of the calibration target." << '\n';
        observationsFout << "#    Marker-ID Top-Left Top-Right Bottom-Right Bottom-Left" << '\n';
        for (uint m = 0; m < iMarkerIds.size(); ++m)
        {
            observationsFout << iMarkerIds[m];
            for (uint c = 0; c < 4; ++c)
            {
                observationsFout << " " << iMarkerCorners[m][c];
            }
            observationsFout << '\n';
        }
        observationsFout.close();
    }
    catch (const std::exception& e)
    {
        throw e;
    }
}

} // namespace multisensor_calibration