/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/sensor_data_processing/CameraDataProcessor.h"

// ROS
#include <cv_bridge/cv_bridge.hpp>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/filters/passthrough.h>

// OpenCV
#include <opencv2/imgcodecs.hpp>

// multisensor_calibration
#include <multisensor_calibration/common/lib3D/core/camera.hpp>
#include <multisensor_calibration/common/utils.hpp>

namespace multisensor_calibration
{

//==================================================================================================
CameraDataProcessor::CameraDataProcessor(const std::string& iLoggerName,
                                         const std::string& iSensorName,
                                         const fs::path& iCalibTargetFilePath) :
  DataProcessor2d(iLoggerName, iSensorName, iCalibTargetFilePath),
  cameraIntrinsics_(lib3d::Intrinsics()),
  isCameraIntrinsicsSet_(false),
  pArucoDetectorParameters_(nullptr),
  imageState_(STR_2_IMG_STATE.at(DEFAULT_IMG_STATE_STR))
{
    //--- initialized settings of aruco detection
    pArucoDetectorParameters_                         = cv::aruco::DetectorParameters::create();
    pArucoDetectorParameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    //--- initialize marker id color lookup
    //--- rviz rainbow colormap is actually HSV cropped at 83.33%: https://answers.ros.org/question/182444/rviz-colorbar/
    cv::Mat tmpMap = cv::Mat((markerIdRange_.second - markerIdRange_.first + 1), // row number equals number of marker IDs in range
                             1, CV_8UC1);
    for (auto i = markerIdRange_.first; i <= markerIdRange_.second; ++i)
        tmpMap.at<uchar>(i - markerIdRange_.first) = static_cast<uchar>( // normalize into the range [0, 255]
          static_cast<float>(i - markerIdRange_.first) /
          static_cast<float>(markerIdRange_.second - markerIdRange_.first) *
          (255.f * 0.8333)); // crop colormap
    cv::applyColorMap(tmpMap, markerIdColorLookup_, cv::COLORMAP_HSV);
}

//==================================================================================================
CameraDataProcessor::~CameraDataProcessor()
{
}

//==================================================================================================
lib3d::Intrinsics CameraDataProcessor::getCameraIntrinsics() const
{
    return cameraIntrinsics_;
}

//==================================================================================================
const lib3d::Intrinsics& CameraDataProcessor::cameraIntrinsics() const
{
    return cameraIntrinsics_;
}

//==================================================================================================
EImageState CameraDataProcessor::getImageState() const
{
    return imageState_;
}

//==================================================================================================
bool CameraDataProcessor::isCameraIntrinsicsSet() const
{
    return isCameraIntrinsicsSet_;
}

//==================================================================================================
CameraDataProcessor::EProcessingResult CameraDataProcessor::processData(
  const cv::Mat& iCameraImage,
  const EProcessingLevel& iProcLevel)
{
    //--- detect markers
    std::vector<int> detectedMarkerIds;
    std::vector<std::array<cv::Point2f, 4>> detectedMarkerCorners;
    bool isSuccessful = detectMarkersInImage(iCameraImage, detectedMarkerIds,
                                             detectedMarkerCorners, annotatedCameraImgPreview_);
    if (!isSuccessful)
    {
        RCLCPP_ERROR(logger_, "Marker detection was not successful!");
        return FAILED;
    }

    //--- if processing level is set to preview, draw previous detections if available, then return true
    if (iProcLevel == PREVIEW)
    {
        if (!capturedMarkerCorners_.empty())
            drawMarkerCornersIntoImage(capturedMarkerIds_.back(), capturedMarkerCorners_.back(),
                                       annotatedCameraImgPreview_);
        return SUCCESS;
    }

    //--- if not enough markers are detected, return false
    if (detectedMarkerIds.size() < static_cast<uint>(calibrationTarget_.minMarkerDetection))
        return FAILED;

    RCLCPP_INFO(logger_, "Found observation. Detected Markers: %li / %li ",
                detectedMarkerIds.size(), calibrationTarget_.markerIds.size());

    //--- draw detections and publish
    drawMarkerCornersIntoImage(detectedMarkerIds, detectedMarkerCorners,
                               annotatedCameraImgPreview_);

    //--- compute board pose
    lib3d::Extrinsics targetPose = lib3d::Extrinsics();
    isSuccessful                 = estimateBoardPose(detectedMarkerIds,
                                                     detectedMarkerCorners,
                                                     targetPose);

    if (!isSuccessful)
    {
        RCLCPP_ERROR(logger_, "Board estimation was not successful!");
        return FAILED;
    }

    //--- compute point cloud of target board
    calibrationTargetCloudPtrs_.push_back(nullptr);
    isSuccessful = computeTargetCloud(iCameraImage, targetPose, calibrationTargetCloudPtrs_.back());

    if (!isSuccessful)
    {
        RCLCPP_ERROR(logger_, "Target cloud computation was not successful!");
        calibrationTargetCloudPtrs_.pop_back();
        return FAILED;
    }

    //--- persistently store detections
    capturedMarkerIds_.push_back(detectedMarkerIds);
    capturedMarkerCorners_.push_back(detectedMarkerCorners);
    capturedCalibTargetPoses_.push_back(targetPose);

    annotatedCameraImageDetection_.push_back(cv::Mat());
    annotatedCameraImgPreview_.copyTo(annotatedCameraImageDetection_.back());

    return SUCCESS;
}

//==================================================================================================
void CameraDataProcessor::setCameraIntrinsics(const lib3d::Intrinsics& iIntrinsics)
{
    cameraIntrinsics_ = iIntrinsics;

    if (cameraIntrinsics_.getWidth() > 0 && cameraIntrinsics_.getHeight() > 0 &&
        cameraIntrinsics_.getFx() > 0 && cameraIntrinsics_.getFy() > 0 &&
        cameraIntrinsics_.getCx() > 0 && cameraIntrinsics_.getCy() > 0)
        isCameraIntrinsicsSet_ = true;
}

//==================================================================================================
void CameraDataProcessor::setImageState(const EImageState& iState)
{
    imageState_ = iState;
}

//==================================================================================================
bool CameraDataProcessor::computeTargetCloud(
  const cv::Mat& iCameraImage,
  const lib3d::Extrinsics& iBoardPose,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& opTargetCloud)
{

    if (iCameraImage.empty() || isCameraIntrinsicsSet_ == false)
        return false;

    //--- compute plane parameters from board pose
    cv::Vec4d boardPlaneParams; // 4D plane parameters of calibration board
    CalibrationTarget::computePlaneParametersFromPose(iBoardPose, boardPlaneParams);

    //--- get RT matrices of board pose
    const cv::Matx44d boardPoseLocal2RefRT = iBoardPose.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF);
    const cv::Matx44d boardPoseRef2LocalRT = iBoardPose.getRTMatrix(lib3d::Extrinsics::REF_2_LOCAL);

    //--- undistort input image
    cv::Mat undistortedImg; // undistoted image of camera image
    if (imageState_ == DISTORTED)
        cv::undistort(iCameraImage, undistortedImg,
                      cv::Mat(cameraIntrinsics_.getK_as3x3()),
                      cameraIntrinsics_.getDistortionCoeffs());
    else
        iCameraImage.copyTo(undistortedImg);

    // copy of camera object without distortion parameters
    lib3d::Camera pinholeCamera;
    pinholeCamera.intrinsics.setImageSize(undistortedImg.size());
    pinholeCamera.intrinsics.setBy_K(cameraIntrinsics_.getK_as3x3());

    // vector of image projections of board corner points on undistorted image
    // (order: clockwise, starting from top-left)
    std::vector<cv::Point2i> boardCornerImgPnts(4, cv::Point2i());

    // references to individual image projections of board corner points on undistorted image
    cv::Point2i &boardTLImgPnt = boardCornerImgPnts[0],
                &boardTRImgPnt = boardCornerImgPnts[1],
                &boardBRImgPnt = boardCornerImgPnts[2],
                &boardBLImgPnt = boardCornerImgPnts[3];

    //--- computer image points of board corners by projecting corner point into undistorted image
    const double WIDTH_2     = calibrationTarget_.boardSize.width / 2.0;  // HALF WIDTH
    const double HEIGHT_2    = calibrationTarget_.boardSize.height / 2.0; // HALF HEIGHT
    cv::Vec4d boardCornerPnt = boardPoseLocal2RefRT * cv::Vec4d(-WIDTH_2, HEIGHT_2, 0, 1);
    boardTLImgPnt =
      pinholeCamera.projectLocal2Pixel(static_cast<float>(boardCornerPnt(0)),
                                       static_cast<float>(boardCornerPnt(1)),
                                       static_cast<float>(boardCornerPnt(2)));
    boardCornerPnt = boardPoseLocal2RefRT * cv::Vec4d(WIDTH_2, HEIGHT_2, 0, 1);
    boardTRImgPnt =
      pinholeCamera.projectLocal2Pixel(static_cast<float>(boardCornerPnt(0)),
                                       static_cast<float>(boardCornerPnt(1)),
                                       static_cast<float>(boardCornerPnt(2)));
    boardCornerPnt = boardPoseLocal2RefRT * cv::Vec4d(WIDTH_2, -HEIGHT_2, 0, 1);
    boardBRImgPnt =
      pinholeCamera.projectLocal2Pixel(static_cast<float>(boardCornerPnt(0)),
                                       static_cast<float>(boardCornerPnt(1)),
                                       static_cast<float>(boardCornerPnt(2)));
    boardCornerPnt = boardPoseLocal2RefRT * cv::Vec4d(-WIDTH_2, -HEIGHT_2, 0, 1);
    boardBLImgPnt =
      pinholeCamera.projectLocal2Pixel(static_cast<float>(boardCornerPnt(0)),
                                       static_cast<float>(boardCornerPnt(1)),
                                       static_cast<float>(boardCornerPnt(2)));

    // axis aligned bounding box encapsulation the target in the image
    cv::Rect targetBoundingROI(cv::Point2i(std::min(boardTLImgPnt.x, boardBLImgPnt.x),
                                           std::min(boardTLImgPnt.y, boardTRImgPnt.y)),
                               cv::Point2i(std::max(boardTRImgPnt.x, boardBRImgPnt.x),
                                           std::max(boardBLImgPnt.y, boardBRImgPnt.y)));

    //--- loop over pixels in ROI of undistorted image, check if they lie on calibration board,
    //--- reproject based on board normal vector into 3D points and store in output point cloud

    // Output point cloud of calibration board
    opTargetCloud.reset(
      new pcl::PointCloud<pcl::PointXYZRGB>(targetBoundingROI.width, targetBoundingROI.height));

#ifndef DEBUG_BUILD
#pragma omp parallel shared(opTargetCloud, undistortedImg, boardCornerImgPnts, iCameraImage)
#endif
    {
#ifndef DEBUG_BUILD
#pragma omp for collapse(2)
#endif
        for (int y = targetBoundingROI.y;
             y < (targetBoundingROI.y + targetBoundingROI.height);
             ++y)
        {
            for (int x = targetBoundingROI.x;
                 x < (targetBoundingROI.x + targetBoundingROI.width);
                 ++x)
            {
                //--- if x or y is outside image, continue
                if (y < 0 || y >= undistortedImg.size().height ||
                    x < 0 || x >= undistortedImg.size().width)
                    continue;

                //--- check that pixel actually lie on target, based on image projections
                //--- continue to next point if outside the polygon
                double testResult =
                  cv::pointPolygonTest(boardCornerImgPnts, cv::Point2d(x, y), false);
                if (testResult < 0) // if point lies outside polygon, test result is -1
                    continue;

                //--- reproject point onto plane of calibration target

                // 3D point on calibration target
                cv::Vec3d targetPnt3D =
                  lib3d::reprojectImagePointOntoPlane(pinholeCamera.intrinsics.getK_as3x3(),
                                                      cv::Vec2d(x, y),
                                                      cv::Vec3d(boardPlaneParams(0),
                                                                boardPlaneParams(1),
                                                                boardPlaneParams(2)),
                                                      static_cast<float>(boardPlaneParams(3)));

                // local 2D point on target
                cv::Vec4d localTargetPnt = boardPoseRef2LocalRT *
                                           cv::Vec4d(targetPnt3D(0),
                                                     targetPnt3D(1),
                                                     targetPnt3D(2),
                                                     1.0);

                //--- check if point lies within cutouts, continue with next if true
                if (calibrationTarget_.isLocalPointInsideCutout(static_cast<float>(localTargetPnt(0)),
                                                                static_cast<float>(localTargetPnt(1))))
                    continue;

                // color value at designated pixel (in BGR)
                cv::Vec3b pxColor = undistortedImg.at<cv::Vec3b>(y, x);

                //--- construct point for point cloud

                pcl::PointXYZRGB& pclPnt = opTargetCloud->at(x - targetBoundingROI.x,
                                                             y - targetBoundingROI.y);
                pclPnt.x                 = static_cast<float>(targetPnt3D(0));
                pclPnt.y                 = static_cast<float>(targetPnt3D(1));
                pclPnt.z                 = static_cast<float>(targetPnt3D(2));
                pclPnt.r                 = pxColor(2);
                pclPnt.g                 = pxColor(1);
                pclPnt.b                 = pxColor(0);
            }
        }
    }

    //--- remove points with a z-value = 0 (i.e. points without coordinates)

    pcl::PassThrough<pcl::PointXYZRGB> zPassthrough;
    zPassthrough.setInputCloud(opTargetCloud);
    zPassthrough.setFilterFieldName("z");
    zPassthrough.setFilterLimits(0.001f, FLT_MAX);
    zPassthrough.filter(*opTargetCloud);

    return true;
}

//==================================================================================================
bool CameraDataProcessor::detectMarkersInImage(
  const cv::Mat& iCameraImage,
  std::vector<int>& oDetectedMarkerIds,
  std::vector<std::array<cv::Point2f, 4>>& oDetectedMarkerCorners,
  cv::Mat& oAnnotatedCameraImage) const
{
    if (iCameraImage.empty())
        return false;

    //--- clear output containers
    oDetectedMarkerIds.clear();
    oDetectedMarkerCorners.clear();

    //--- detect markers on target board
    //--- ArUco dictionary: DICT_6X6_250
    //--- Markers order:
    //--- 1-------2
    //--- |       |
    //--- |   C   |
    //--- |       |
    //--- 4-------3

    //-- detect markers
    std::vector<std::vector<cv::Point2f>> tmpDetectedMarkerCorners, rejectedCandidates;
    cv::aruco::detectMarkers(iCameraImage, calibrationTarget_.pArucoDictionary,
                             tmpDetectedMarkerCorners, oDetectedMarkerIds,
                             pArucoDetectorParameters_, rejectedCandidates);

    //--- draw markers into image
    iCameraImage.copyTo(oAnnotatedCameraImage);
    cv::aruco::drawDetectedMarkers(oAnnotatedCameraImage,
                                   tmpDetectedMarkerCorners, oDetectedMarkerIds);

    //--- copy marker corner from vector to array
    for (uint i = 0; i < tmpDetectedMarkerCorners.size(); ++i)
    {
        oDetectedMarkerCorners.push_back(std::array<cv::Point2f, 4>());
        for (uint j = 0; j < 4; ++j)
        {
            oDetectedMarkerCorners.back()[j] = tmpDetectedMarkerCorners[i][j];
        }
    }

    return true;
}

//==================================================================================================
void CameraDataProcessor::drawMarkerCornersIntoImage(
  const std::vector<int>& iMarkerIds,
  const std::vector<std::array<cv::Point2f, 4>>& iMarkerCorners,
  cv::Mat& ioImage) const
{
    //--- loop over marker IDs
    for (uint i = 0; i < iMarkerIds.size(); ++i)
    {
        auto markerId         = iMarkerIds[i];
        cv::Vec3b markerColor = markerIdColorLookup_.at<cv::Vec3b>(markerId - markerIdRange_.first);

        //--- loop over corners for each marker
        for (cv::Point2f cornerPnt : iMarkerCorners[i])
            cv::circle(ioImage,
                       cv::Point(static_cast<int>(cornerPnt.x), static_cast<int>(cornerPnt.y)),
                       std::min(ioImage.size[0], ioImage.size[1]) / 100, // 1% of smallest image dimension
                       cv::Scalar(markerColor[0], markerColor[1], markerColor[2]),
                       -1);
    }
}

//==================================================================================================
bool CameraDataProcessor::estimateBoardPose(
  const std::vector<int>& iDetectedMarkerIds,
  const std::vector<std::array<cv::Point2f, 4>>& iDetectedMarkerCorners,
  lib3d::Extrinsics& oBoardPose) const
{
    //--- check if input data is valid
    if (iDetectedMarkerIds.size() == 0 ||
        iDetectedMarkerIds.size() != iDetectedMarkerCorners.size())
        return false;

    //--- detect board and estimate position relative to camera

    // board rotation and translation
    cv::Vec3d boardRVec(0, 0, 0);
    cv::Vec3d boardTVec(0, 0, 0);

#if 0
    // Estimated 3D position of the markers
    std::vector<cv::Vec3d> markerRVecs, markerTVecs;
    cv::Vec3d rvec_sin, rvec_cos;
    cv::aruco::estimatePoseSingleMarkers(iDetectedMarkerCorners, calibrationTarget_.markerSize,
                                         cv::Mat(cameraIntrinsics_.getK_as3x3()),
                                         cameraIntrinsics_.getDistortionCoeffs(),
                                         markerRVecs, markerTVecs);

    for (uint i = 0; i < markerRVecs.size(); i++)
    {
        //--- Accumulate marker pose for initial guess
        boardTVec += markerTVecs[i];
        rvec_sin(0) += sin(markerRVecs[i](0));
        rvec_sin(1) += sin(markerRVecs[i](1));
        rvec_sin(2) += sin(markerRVecs[i](2));
        rvec_cos(0) += cos(markerRVecs[i](0));
        rvec_cos(1) += cos(markerRVecs[i](1));
        rvec_cos(2) += cos(markerRVecs[i](2));
    }

    //--- Compute average pose. Rotation computed as atan2(sin/cos)
    boardTVec /= int(markerRVecs.size());
    rvec_sin     = rvec_sin / int(markerRVecs.size()); // Average sin
    rvec_cos     = rvec_cos / int(markerRVecs.size()); // Average cos
    boardRVec[0] = atan2(rvec_sin[0], rvec_cos[0]);
    boardRVec[1] = atan2(rvec_sin[1], rvec_cos[1]);
    boardRVec[2] = atan2(rvec_sin[2], rvec_cos[2]);
#endif

    // copy array to vector
    std::vector<std::vector<cv::Point2f>> tmpMarkerCorners;
    for (auto marker : iDetectedMarkerCorners)
    {
        tmpMarkerCorners.push_back(std::vector<cv::Point2f>());
        for (auto corner : marker)
            tmpMarkerCorners.back().push_back(corner);
    }

    // return value of estimatePoseBoard
    int retValEstimateBoardPose =
      cv::aruco::estimatePoseBoard(tmpMarkerCorners,
                                   iDetectedMarkerIds,
                                   calibrationTarget_.pArucoBoard,
                                   cv::Mat(cameraIntrinsics_.getK_as3x3()),
                                   cameraIntrinsics_.getDistortionCoeffs(),
                                   boardRVec, boardTVec, false);

    //--- check if board pose is estimated
    if (static_cast<uint>(retValEstimateBoardPose) != iDetectedMarkerIds.size() ||
        std::isnan(cv::norm(boardRVec)) ||
        std::isnan(cv::norm(boardTVec)))
        return false;

    //--- set board pose
    oBoardPose = lib3d::Extrinsics(lib3d::Extrinsics::LOCAL_2_REF);
    oBoardPose.setRotationVec(boardRVec);
    oBoardPose.setTranslationVec(boardTVec);

    return true;
}

} // namespace multisensor_calibration