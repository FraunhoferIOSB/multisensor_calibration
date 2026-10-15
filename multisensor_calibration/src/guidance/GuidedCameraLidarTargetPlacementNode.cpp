/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

// Std
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include <cstddef>
#include <functional>
#define USE_MATH_DEFINES
#include <cmath>

// ROS
#include <cv_bridge/cv_bridge.hpp>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

// OpenCV
#include <opencv2/imgproc.hpp>

// multisensor_calibration
#include "multisensor_calibration/calibration_target/Cutout.h"
#include "multisensor_calibration/guidance/GuidedCameraLidarTargetPlacementNode.h"
#include <multisensor_calibration/common/lib3D/core/camera.hpp>
#include <multisensor_calibration/common/utils.hpp>

// Line color of target overlay.
const cv::Scalar TARGET_OVERLAY_COLOR = cv::Scalar(255, 0, 0);

// Line thickness of target overlay.
const int TARGET_OVERLAY_LINE_THICKNESS = 6;

// Text font.
const int TEXT_FONT_FACE = cv::FONT_HERSHEY_SIMPLEX;

// Text font scale.
const double TEXT_FONT_SCALE = 2;

// Text thickness.
const int TEXT_THICKNESS = 4;

// Text Color
const cv::Scalar TEXT_COLOR = cv::Scalar(0, 0, 255);

namespace multisensor_calibration
{

//==================================================================================================
GuidedCameraLidarTargetPlacementNode::GuidedCameraLidarTargetPlacementNode(std::string appTitle, rclcpp::NodeOptions iOptions) :
  GuidanceBase(this),
  rclcpp::Node(appTitle + "_" + GUIDANCE_SUB_NAMESPACE, iOptions),
  pCamIntrinsicsTimer_(nullptr)
{
    resetNextTargetPose();
    //--- set node name and parent namespace
    appTitle_           = appTitle;
    calibratorNodeName_ = appTitle_;

    init();
}

//==================================================================================================
GuidedCameraLidarTargetPlacementNode::GuidedCameraLidarTargetPlacementNode(const rclcpp::NodeOptions& iOptions) :
  GuidanceBase(this),
  rclcpp::Node(GUIDANCE_SUB_NAMESPACE, iOptions),
  pCamIntrinsicsTimer_(nullptr)
{
    resetNextTargetPose();

    calibratorNodeName_ = declare_parameter<std::string>("calibrator_node_name",
                                                         CALIB_TYPE_2_NODE_NAME.at(EXTRINSIC_CAMERA_LIDAR_CALIBRATION));
    appTitle_           = calibratorNodeName_;

    init();
}

//==================================================================================================
GuidedCameraLidarTargetPlacementNode::~GuidedCameraLidarTargetPlacementNode()
{
}

//==================================================================================================
void GuidedCameraLidarTargetPlacementNode::computeExtrinsicFovBoundingPlanes()
{
    // TODO:
    // //--- compute ground plane from lidar data and max hight of target
    // //--- get single point cloud message
    // //--- get camera intrinsics from camera_info
    // InputCloud_Message_T::ConstSharedPtr pLidarCloudMsg =
    //   ros::topic::waitForMessage<InputCloud_Message_T>(calibrationMetaData_.ref_topic_name,
    //                                                    ros::Duration(3, 0));
    // pcl::PointCloud<InputPointType>::Ptr pLidarCloud(new pcl::PointCloud<InputPointType>);
    // pcl::fromROSMsg(*pLidarCloudMsg, *pLidarCloud);

    // //--- detect plane with ransac and project points onto plane
    // Eigen::VectorXf groundPlaneCoefficients;
    // pcl::SampleConsensusModelPerpendicularPlane<InputPointType>::Ptr pPlaneSacModel(
    //   new pcl::SampleConsensusModelPerpendicularPlane<InputPointType>(pLidarCloud));
    // pPlaneSacModel->setAxis(Eigen::Vector3f(0.f, 0.f, 1.f));
    // pPlaneSacModel->setEpsAngle(M_PI / 6.0);
    // pcl::RandomSampleConsensus<InputPointType> planeRansac(pPlaneSacModel);
    // planeRansac.setDistanceThreshold(0.05);

    // bool isSuccessful = false;
    // while (!isSuccessful)
    // {
    //     isSuccessful = planeRansac.computeModel();
    // }
    // planeRansac.getModelCoefficients(groundPlaneCoefficients);

    // Eigen::Vector4d lidarGroundPlane = Eigen::Vector4d(groundPlaneCoefficients(0),
    //                                                    groundPlaneCoefficients(1),
    //                                                    groundPlaneCoefficients(2),
    //                                                    groundPlaneCoefficients(3));

    // fovBoundingPlanes_.push_back(lidarGroundPlane);

    // ROS_INFO("%f, %f, %f, %f", groundPlaneCoefficients(0),
    //          groundPlaneCoefficients(1),
    //          groundPlaneCoefficients(2),
    //          groundPlaneCoefficients(3));
}

//==================================================================================================
bool GuidedCameraLidarTargetPlacementNode::computeIntrinsicFovBoundingPlanes()
{
    //--- return false if required data is not yet available
    if (cameraIntrinsics_.getImageSize() == cv::Size(0, 0) ||
        calibrationTarget_.isValid() == false)
        return false;

    fovBoundingPlanes_.clear();

    //--- compute bounding planes of camera view frustum
    //--- left, right, up, and bottom planes are given by horizontal and vertical FoV
    //--- distance of near-plane is computed in such a way hat teh target just fits into the view
    //--- distance of far-plane is computed based on min GSD

    cv::Vec3d tmpNormalVec =
      lib3d::Rotation::createRotationX_deg(cameraIntrinsics_.getVFov() / 2.0) *
      cv::Vec3d(0.0, 1.0, 0.0);
    Eigen::Vector4d camFovTop = // top plane of camera FoV
      Eigen::Vector4d(tmpNormalVec(0), tmpNormalVec(1), tmpNormalVec(2), 0.0);

    tmpNormalVec =
      lib3d::Rotation::createRotationX_deg(-cameraIntrinsics_.getVFov() / 2.0) *
      cv::Vec3d(0.0, -1.0, 0.0);
    Eigen::Vector4d camFovBottom = // bottom plane of camera FoV
      Eigen::Vector4d(tmpNormalVec(0), tmpNormalVec(1), tmpNormalVec(2), 0.0);

    tmpNormalVec =
      lib3d::Rotation::createRotationY_deg(-cameraIntrinsics_.getHFov() / 2.0) *
      cv::Vec3d(1.0, 0.0, 0.0);
    Eigen::Vector4d camFovLeft = // left plane of camera FoV
      Eigen::Vector4d(tmpNormalVec(0), tmpNormalVec(1), tmpNormalVec(2), 0.0);

    tmpNormalVec =
      lib3d::Rotation::createRotationY_deg(cameraIntrinsics_.getHFov() / 2.0) *
      cv::Vec3d(-1.0, 0.0, 0.0);
    Eigen::Vector4d camFovRight = // right plane of camera FoV
      Eigen::Vector4d(tmpNormalVec(0), tmpNormalVec(1), tmpNormalVec(2), 0.0);

    tmpNormalVec        = cv::Vec3d(0.0, 0.0, 1.0);
    double tmpDistanceV = (calibrationTarget_.boardSize.height / 2.0) /
                          std::tan((cameraIntrinsics_.getVFov() / 2.0) / 180.0 * M_PI);
    double tmpDistanceH = (calibrationTarget_.boardSize.width / 2.0) /
                          std::tan((cameraIntrinsics_.getHFov() / 2.0) / 180.0 * M_PI);
    Eigen::Vector4d camFovNear = // near plane of camera FoV
      Eigen::Vector4d(tmpNormalVec(0), tmpNormalVec(1), tmpNormalVec(2),
                      std::max(0.0, std::min(tmpDistanceH, tmpDistanceV)));

    tmpNormalVec = cv::Vec3d(0.0, 0.0, -1.0);
    tmpDistanceV = (calibrationTarget_.boardSize.height * 1.7) / // so that target fits just over 3 times TODO: change to gsd
                   std::tan((cameraIntrinsics_.getVFov() / 2.0) / 180.0 * M_PI);
    tmpDistanceH = (calibrationTarget_.boardSize.width * 1.7) / // so that target fits just over 3 times TODO: change to gsd
                   std::tan((cameraIntrinsics_.getHFov() / 2.0) / 180.0 * M_PI);
    Eigen::Vector4d camFovFar = // far plane of camera FoV
      Eigen::Vector4d(tmpNormalVec(0), tmpNormalVec(1), tmpNormalVec(2),
                      std::max(tmpDistanceH, tmpDistanceV));

    fovBoundingPlanes_.push_back(camFovTop);
    fovBoundingPlanes_.push_back(camFovBottom);
    fovBoundingPlanes_.push_back(camFovLeft);
    fovBoundingPlanes_.push_back(camFovRight);
    fovBoundingPlanes_.push_back(camFovNear);
    fovBoundingPlanes_.push_back(camFovFar);

    return true;
}

//==================================================================================================
void GuidedCameraLidarTargetPlacementNode::computeNextTargetPose()
{
    //--- if no bounding planes are available, compute intrinsic bounding planes
    if (fovBoundingPlanes_.empty())
        computeIntrinsicFovBoundingPlanes();

    //--- if no detections are available so far place target at center, in between near and far
    if (detectedTargetPoses_.empty())
    {

        nextTargetPose_.setTransfDirection(lib3d::Extrinsics::LOCAL_2_REF);
        nextTargetPose_.setRotationMat(lib3d::Rotation::createRotationX_deg(180));

        lib3d::Extrinsics poseCandidate = nextTargetPose_;

        double zDistance        = fovBoundingPlanes_[4](3); // distance of near plane
        bool isDistanceComputed = false;
        while (!isDistanceComputed)
        {
            poseCandidate.setTranslationVec(0, 0, zDistance);

            const cv::Matx44d POSE_RT = poseCandidate.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF);

            lib3d::Camera camera(cameraIntrinsics_, lib3d::Extrinsics());

            // TOP-LEFT
            cv::Vec4d localPnt = POSE_RT * cv::Vec4d(-calibrationTarget_.boardSize.width / 2,
                                                     calibrationTarget_.boardSize.height / 2,
                                                     0.0,
                                                     1.0);
            cv::Point2i tlImgPnt =
              camera.projectLocal2Pixel(cv::Point3f(static_cast<float>(localPnt(0)),
                                                    static_cast<float>(localPnt(1)),
                                                    static_cast<float>(localPnt(2))));

            // // TOP-RIGHT
            // localPnt = POSE_RT * cv::Vec4d(calibrationTarget_.boardSize.width / 2,
            //                                calibrationTarget_.boardSize.height / 2,
            //                                0.0,
            //                                1.0);
            // cv::Point2i trImgPnt =
            //   camera.projectLocal2Pixel(cv::Point3f(static_cast<float>(localPnt(0)),
            //                                         static_cast<float>(localPnt(1)),
            //                                         static_cast<float>(localPnt(2))));

            // BOTTOM-RIGHT
            localPnt = POSE_RT * cv::Vec4d(calibrationTarget_.boardSize.width / 2,
                                           -calibrationTarget_.boardSize.height / 2,
                                           0.0,
                                           1.0);
            cv::Point2i brImgPnt =
              camera.projectLocal2Pixel(cv::Point3f(static_cast<float>(localPnt(0)),
                                                    static_cast<float>(localPnt(1)),
                                                    static_cast<float>(localPnt(2))));

            // // BOTTOM-LEFT
            // localPnt = POSE_RT * cv::Vec4d(-calibrationTarget_.boardSize.width / 2,
            //                                -calibrationTarget_.boardSize.height / 2,
            //                                0.0,
            //                                1.0);
            // cv::Point2i blImgPnt =
            //   camera.projectLocal2Pixel(cv::Point3f(static_cast<float>(localPnt(0)),
            //                                         static_cast<float>(localPnt(1)),
            //                                         static_cast<float>(localPnt(2))));

            //--- if target diagonal is third of image diagonal, set z distance as correct
            float targetDiagonal = static_cast<float>(std::sqrt(
              std::pow(brImgPnt.x - tlImgPnt.x, 2) +
              std::pow(brImgPnt.y - tlImgPnt.y, 2)));
            float imgDiagonal    = static_cast<float>(std::sqrt(
              cameraIntrinsics_.getImageSize().width * cameraIntrinsics_.getImageSize().width +
              cameraIntrinsics_.getImageSize().height * cameraIntrinsics_.getImageSize().height));
            if (targetDiagonal <= imgDiagonal / 3.f ||
                zDistance >= fovBoundingPlanes_[5](3)) // distance far plane
            {
                isDistanceComputed = true;
                nextTargetPose_    = poseCandidate;
            }
            else
            {
                zDistance += 0.2;
            }
        }
    }
    else
    {
        //--- compute anchor point of axis
        Eigen::Vector3d axisPnt = Eigen::Vector3d(0, 0, 0);
        for (lib3d::Extrinsics pose : detectedTargetPoses_)
        {
            axisPnt(0) += pose.getTranslationVec()(0);
            axisPnt(1) += pose.getTranslationVec()(1);
            axisPnt(2) += pose.getTranslationVec()(2);
        }
        axisPnt /= detectedTargetPoses_.size();

        //--- find axes index
        for (size_t axIdx = 0; axIdx < axes_.size(); ++axIdx)
        {
            //--- compute axes bounds
            std::pair<float, float> axisBound = std::make_pair(-FLT_MAX, FLT_MAX);
            for (Eigen::Vector4d plane : fovBoundingPlanes_)
            {
                float minBound = -1 * computeAxisBound(axisPnt, -1 * axes_[axIdx], plane);
                if (std::isfinite(minBound) && minBound > axisBound.first)
                    axisBound.first = minBound;

                float maxBound = computeAxisBound(axisPnt, axes_[axIdx], plane);
                if (std::isfinite(maxBound) && maxBound < axisBound.second)
                    axisBound.second = maxBound;
            }

            //--- compute amount which has been covered on axis
            double minV = DBL_MAX;
            double maxV = -DBL_MAX;
            for (lib3d::Extrinsics pose : detectedTargetPoses_)
            {
                const cv::Matx44d POSE_RT = pose.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF);

                auto findMinMAx = [&](const cv::Vec4d& iLocalPnt)
                {
                    cv::Vec4d refPnt = POSE_RT * iLocalPnt;
                    double tmpV      = refPnt(0) * axes_[axIdx](0) +
                                  refPnt(1) * axes_[axIdx](1) +
                                  refPnt(2) * axes_[axIdx](2);
                    minV = std::min(tmpV, minV);
                    maxV = std::max(maxV, tmpV);
                };

                // Top-Left
                findMinMAx(cv::Vec4d(-calibrationTarget_.boardSize.width / 2,
                                     calibrationTarget_.boardSize.height / 2,
                                     0.0,
                                     1.0));

                // Top-Right
                findMinMAx(cv::Vec4d(calibrationTarget_.boardSize.width / 2,
                                     calibrationTarget_.boardSize.height / 2,
                                     0.0,
                                     1.0));

                // Bottom-Right
                findMinMAx(cv::Vec4d(calibrationTarget_.boardSize.width / 2,
                                     -calibrationTarget_.boardSize.height / 2,
                                     0.0,
                                     1.0));

                // Bottom-Left
                findMinMAx(cv::Vec4d(-calibrationTarget_.boardSize.width / 2,
                                     -calibrationTarget_.boardSize.height / 2,
                                     0.0,
                                     1.0));
            }

            //--- if coverage big enough, continue
            float axisCoverage = static_cast<float>(maxV - minV) /
                                 (axisBound.second - axisBound.first);
            if (axisCoverage > 0.6) // TODO: dynConfig
                continue;

            //--- find direction into which to move the target by taking the highest absolute value of
            //--- the axis bounds, i.e. identifying the side that is furthest away
            double lambda = (std::abs(axisBound.first) < std::abs(axisBound.second)) ? -0.2 : 0.2;

            //--- find next pose
            Eigen::Vector3f sensorAnchorPnt(0, 0, 0); // TODO: compute based on sensor extrinsic
            int itr = 1;
            Eigen::Vector3f centerVec(static_cast<float>(axisPnt(0) + itr * lambda * axes_[axIdx](0)),
                                      static_cast<float>(axisPnt(1) + itr * lambda * axes_[axIdx](1)),
                                      static_cast<float>(axisPnt(2) + itr * lambda * axes_[axIdx](2)));
            cv::Matx44d local2RefRTMat = nextTargetPose_.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF);
            Eigen::Vector3f upVec(static_cast<float>(local2RefRTMat(0, 1)),
                                  static_cast<float>(local2RefRTMat(1, 1)),
                                  static_cast<float>(local2RefRTMat(2, 1)));
            Eigen::Vector3f normalVec = (sensorAnchorPnt - centerVec).normalized();
            lib3d::Extrinsics poseCandidate;
            CalibrationTarget::computePose(centerVec, upVec, normalVec, poseCandidate);
            while (isTargetPoseWithinBoundingPlanes(poseCandidate))
            {
                nextTargetPose_ = poseCandidate;

                itr++;
                centerVec = Eigen::Vector3f(static_cast<float>(axisPnt(0) + itr * lambda * axes_[axIdx](0)),
                                            static_cast<float>(axisPnt(1) + itr * lambda * axes_[axIdx](1)),
                                            static_cast<float>(axisPnt(2) + itr * lambda * axes_[axIdx](2)));
                normalVec = (sensorAnchorPnt - centerVec).normalized();
                CalibrationTarget::computePose(centerVec, upVec, normalVec, poseCandidate);
            }

            break;
        }
    }
}

//==================================================================================================
void GuidedCameraLidarTargetPlacementNode::drawOutlineOntoImage(
  cv::Mat& ioImage, const std::vector<cv::Point2i>& iPixels,
  const cv::Scalar& iColor, const int& iThickness) const
{
    if (iPixels.size() < 3)
        return;

    for (uint i = 0; i < (iPixels.size() - 1); ++i)
        cv::line(ioImage, iPixels[i], iPixels[i + 1], iColor, iThickness);
    cv::line(ioImage, iPixels[iPixels.size() - 1], iPixels[0], iColor, iThickness);
}

//==================================================================================================
void GuidedCameraLidarTargetPlacementNode::drawNextTargetPoseOntoImage(cv::Mat& ioImage)
{
    // temp image which is later to be overlayed half transparent
    cv::Mat tmpImgTransparent = cv::Mat::zeros(ioImage.size(), ioImage.type());

    //--- outer boarder
    std::vector<cv::Point2i> borderPixels =
      projectTargetBorderFrameToImage(calibrationTarget_.boardSize);

    std::vector<std::vector<cv::Point2i>> borderPoly; // polygon container of border pixels
    borderPoly.push_back(borderPixels);

    //--- cutouts
    std::vector<std::vector<cv::Point2i>> cutoutsPoly; // polygon container of border pixels
    for (std::shared_ptr<Cutout> pCutout : calibrationTarget_.pBoardCutouts)
    {
        std::vector<cv::Point2i> cutoutPixels =
          projectCircularCutoutToImage(pCutout->getCoefficients());

        cutoutsPoly.push_back(cutoutPixels);
    }

    //--- draw filled areas
    cv::fillPoly(tmpImgTransparent, borderPoly, TARGET_OVERLAY_COLOR);
    cv::fillPoly(tmpImgTransparent, cutoutsPoly, cv::Scalar(0, 0, 0));

    //--- add weighted images
    cv::addWeighted(ioImage, 1.0, tmpImgTransparent, 0.6, 0.0, ioImage);

    //--- draw outlines
    for (std::vector<cv::Point2i> outline : borderPoly)
        drawOutlineOntoImage(ioImage, outline,
                             TARGET_OVERLAY_COLOR, TARGET_OVERLAY_LINE_THICKNESS);

    for (std::vector<cv::Point2i> outline : cutoutsPoly)
        drawOutlineOntoImage(ioImage, outline,
                             TARGET_OVERLAY_COLOR, TARGET_OVERLAY_LINE_THICKNESS);
}

//==================================================================================================
void GuidedCameraLidarTargetPlacementNode::drawTextOntoImage(
  const std::string& iText, cv::Mat& ioImage) const
{
    //--- get size of text to put
    int baseline      = 0;
    cv::Size textSize = cv::getTextSize(iText, TEXT_FONT_FACE,
                                        TEXT_FONT_SCALE, TEXT_THICKNESS, &baseline);
    baseline += TEXT_THICKNESS;

    //--- center the text
    cv::Point textOrg((ioImage.cols - textSize.width) / 2,
                      (ioImage.rows + textSize.height) / 2);

    //--- draw the box that is not fully opaque
    cv::Mat rectangleImg = cv::Mat::zeros(ioImage.size(), ioImage.type());
    cv::rectangle(rectangleImg, textOrg + cv::Point(0, baseline),
                  textOrg + cv::Point(textSize.width, -(textSize.height + 2 * TEXT_THICKNESS)),
                  cv::Scalar(255, 255, 255), -1);
    cv::addWeighted(ioImage, 1.0, rectangleImg, 0.6, 0, ioImage);

    //--- put the text itself
    cv::putText(ioImage, iText, textOrg, TEXT_FONT_FACE, TEXT_FONT_SCALE,
                TEXT_COLOR, TEXT_THICKNESS, 8);
}

//==================================================================================================
void GuidedCameraLidarTargetPlacementNode::getCameraIntrinsics()
{
    intrinsicsClient_       = this->create_client<CameraIntrinsicsSrv>(calibratorNodeName_ +
                                                                       "/" + REQUEST_CAM_INTRINSICS_SRV_NAME);
    bool isServiceAvailable = false;
    const int MAX_TRIES     = 10;
    int cntr                = 0;

    while (!isServiceAvailable && cntr < MAX_TRIES)
    {
        isServiceAvailable = intrinsicsClient_->wait_for_service(std::chrono::milliseconds(500));
        cntr++;
    }

    if (!isServiceAvailable)
    {
        RCLCPP_ERROR(pNode_->get_logger(),
                     "Service %s is not available", intrinsicsClient_->get_service_name());
        return;
    }

    auto request = std::make_shared<CameraIntrinsicsSrv::Request>();
    auto future  = intrinsicsClient_->async_send_request(
      request,
      [this](rclcpp::Client<CameraIntrinsicsSrv>::SharedFuture response)
      {
          utils::setCameraIntrinsicsFromCameraInfo(response.get()->intrinsics,
                                                    cameraIntrinsics_);

          //--- if camera intrinsics is valid stop timer
          if (cameraIntrinsics_.getImageSize() != cv::Size(0, 0))
          {
              pCamIntrinsicsTimer_->cancel();
          }
      });
}

//==================================================================================================
bool GuidedCameraLidarTargetPlacementNode::initializePublishers()
{
    //--- advertise topic of annotated input images
    pGuidanceImgPub_ = this->create_publisher<sensor_msgs::msg::Image>(std::string(this->get_name()) + "/" + PLACEMENT_GUIDANCE_TOPIC_NAME, 10);

    return true;
}

//==================================================================================================
bool GuidedCameraLidarTargetPlacementNode::initializeSubscribers()
{
    //--- call parent method
    bool isSuccessful = GuidanceBase::initializeSubscribers();
    if (!isSuccessful)
        return false;

    //--- initialize image subscriber
    pCameraImageSubsc_ = this->create_subscription<sensor_msgs::msg::Image>(
      pCalibrationMetaData_->src_topic_name, 10,
      std::bind(&GuidedCameraLidarTargetPlacementNode::onImageReceived,
                this, std::placeholders::_1));

    //--- initialize subscriber to target pose
    pTargetPoseSubsc_ = this->create_subscription<TargetBoardPose_Message_T>(
      calibratorNodeName_ + "/" +
        pCalibrationMetaData_->src_sensor_name + "/" +
        TARGET_BOARD_POSE_TOPIC_NAME,
      1,
      std::bind(&GuidedCameraLidarTargetPlacementNode::onTargetPoseReceived, this, std::placeholders::_1));

    return true;
}

//==================================================================================================
bool GuidedCameraLidarTargetPlacementNode::initializeTimers()
{
    //--- call parent method
    auto baseRet = GuidanceBase::initializeTimers();

    //--- initialize trigger to call routine to get intrinsic camera data
    pCamIntrinsicsTimer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&GuidedCameraLidarTargetPlacementNode::getCameraIntrinsics, this), nullptr, false);

    return pCamIntrinsicsTimer_ != nullptr && baseRet;
}

//==================================================================================================
void GuidedCameraLidarTargetPlacementNode::onImageReceived(
  const InputImage_Message_T::ConstSharedPtr& ipImgMsg)
{
    //--- check if initialized
    if (!isInitialized_)
    {
        RCLCPP_FATAL(this->get_logger(), "Node is not initialized!");
        return;
    }

    if (!initialPoseReceived_)
    {
        RCLCPP_WARN(this->get_logger(), "Initial Pose not received yet");
        return;
    }

    //--- get image from message
    cv_bridge::CvImageConstPtr pCvImg;
    try
    {
        pCvImg = cv_bridge::toCvShare(ipImgMsg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_FATAL(this->get_logger(), "cv_bridge::Exception: %s", e.what());
        return;
    }

    // Object of camera image
    cv::Mat image;

    //--- convert to bgr or simply copy to member variable
    if (pCvImg->encoding == "mono8")
        cv::cvtColor(pCvImg->image, image, CV_GRAY2BGR);
    else if (pCvImg->encoding == "rgb8")
        cv::cvtColor(pCvImg->image, image, CV_RGB2BGR);
    else
        pCvImg->image.copyTo(image);

    //--- draw next calibration target pose  onto image
    if (nextTargetPose_ != lib3d::Extrinsics())
        drawNextTargetPoseOntoImage(image);

    //--- draw text if no sensor pose is available
    if (extrinsicSensorPose_ == lib3d::Extrinsics())
        drawTextOntoImage("Please add first Observation!", image);

    //--- publish annotated image
    auto annotatedImgMsg =
      cv_bridge::CvImage(ipImgMsg->header, "bgr8", image).toImageMsg();
    pGuidanceImgPub_->publish(*annotatedImgMsg);
}

//==================================================================================================
void GuidedCameraLidarTargetPlacementNode::init()
{
    //--- read launch parameters
    isInitialized_ &= readLaunchParameters();

    //--- initialize publishers
    isInitialized_ &= initializePublishers();

    //--- initialize services
    isInitialized_ &= initializeServices();

    //--- initialize timers
    isInitialized_ &= initializeTimers();

    //--- start ros timer
    if (isInitialized_)
    {
        pCamIntrinsicsTimer_->reset();
        pCalibMetaDataTimer_->reset();
    }
}

//==================================================================================================
std::vector<cv::Point2i> GuidedCameraLidarTargetPlacementNode::projectTargetBorderFrameToImage(
  const cv::Size2f& iBoardSize) const
{
    // list of pixels holding the projected corner points
    std::vector<cv::Point2i> cornerPixels;

    //--- construct meta data needed for projection
    const lib3d::Camera CAMERA(cameraIntrinsics_);
    const cv::Matx44d TARGET_POSE = nextTargetPose_.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF);

    cv::Vec4d tlPoint = TARGET_POSE * cv::Vec4d(-iBoardSize.width / 2,
                                                iBoardSize.height / 2,
                                                0.0, 1.0);
    cornerPixels.push_back(CAMERA.projectLocal2Pixel(static_cast<float>(tlPoint(0)),
                                                     static_cast<float>(tlPoint(1)),
                                                     static_cast<float>(tlPoint(2))));

    cv::Vec4d trPoint = TARGET_POSE * cv::Vec4d(iBoardSize.width / 2,
                                                iBoardSize.height / 2,
                                                0.0, 1.0);
    cornerPixels.push_back(CAMERA.projectLocal2Pixel(static_cast<float>(trPoint(0)),
                                                     static_cast<float>(trPoint(1)),
                                                     static_cast<float>(trPoint(2))));

    cv::Vec4d brPoint = TARGET_POSE * cv::Vec4d(iBoardSize.width / 2,
                                                -iBoardSize.height / 2,
                                                0.0, 1.0);
    cornerPixels.push_back(CAMERA.projectLocal2Pixel(static_cast<float>(brPoint(0)),
                                                     static_cast<float>(brPoint(1)),
                                                     static_cast<float>(brPoint(2))));

    cv::Vec4d blPoint = TARGET_POSE * cv::Vec4d(-iBoardSize.width / 2,
                                                -iBoardSize.height / 2,
                                                0.0, 1.0);
    cornerPixels.push_back(CAMERA.projectLocal2Pixel(static_cast<float>(blPoint(0)),
                                                     static_cast<float>(blPoint(1)),
                                                     static_cast<float>(blPoint(2))));

    return cornerPixels;
}

//==================================================================================================
std::vector<cv::Point2i> GuidedCameraLidarTargetPlacementNode::projectCircularCutoutToImage(
  const std::vector<float>& iCoefficients) const
{
    // list of pixels holding the projected outline points
    std::vector<cv::Point2i> outlinePixels;

    //--- construct meta data needed for projection
    const lib3d::Camera CAMERA(cameraIntrinsics_);
    const cv::Matx44d TARGET_POSE = nextTargetPose_.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF);

    float degree = 0.f;
    while (degree < 360.f)
    {
        cv::Vec4d point = TARGET_POSE * cv::Vec4d(
                                          iCoefficients[0] + (iCoefficients[2] * std::sin(degree / 180.f * M_PI)),
                                          iCoefficients[1] + (iCoefficients[2] * std::cos(degree / 180.f * M_PI)),
                                          0.0, 1.0);
        outlinePixels.push_back(CAMERA.projectLocal2Pixel(static_cast<float>(point(0)),
                                                          static_cast<float>(point(1)),
                                                          static_cast<float>(point(2))));

        degree += 10.f;
    }

    return outlinePixels;
}

//==================================================================================================
void GuidedCameraLidarTargetPlacementNode::resetNextTargetPose()
{
    nextTargetPose_.setTransfDirection(lib3d::Extrinsics::LOCAL_2_REF);
    nextTargetPose_.setTranslationVec(0, 0, 1.5);
    nextTargetPose_.setRotationMat(lib3d::Rotation::createRotationX_deg(180));
}

} // namespace multisensor_calibration

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(multisensor_calibration::GuidedCameraLidarTargetPlacementNode)
