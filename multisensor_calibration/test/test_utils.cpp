/***********************************************************************
 *
 * Copyright (c) Fraunhofer Institute of Optronics,
 * System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/
/***********************************************************************
 *
 * Unit tests for the math and conversion helpers in common/utils.hpp.
 *
 **********************************************************************/

// Std
#include <filesystem>
#include <fstream>

// GTest
#include <gtest/gtest.h>

// multisensor_calibration
#include <multisensor_calibration/common/utils.hpp>

namespace fs    = std::filesystem;
namespace utils = multisensor_calibration::utils;

TEST(UtilsTest, AverageQuaternionOfIdenticalQuaternions)
{
    const Eigen::Vector4f q = Eigen::Vector4f(0.f, 0.f, 0.382683f, 0.923880f).normalized();

    Eigen::Vector4f avg = utils::averageQuaternion({q, q, q});

    // eigenvector sign is ambiguous
    EXPECT_NEAR(std::abs(avg.dot(q)), 1.f, 1e-5f);
}

TEST(UtilsTest, AverageQuaternionOfSymmetricRotations)
{
    // rotations of +/- 20 degrees about z should average to identity
    const float HALF_ANGLE = 10.f * static_cast<float>(M_PI) / 180.f;
    const Eigen::Vector4f qPlus(0.f, 0.f, std::sin(HALF_ANGLE), std::cos(HALF_ANGLE));
    const Eigen::Vector4f qMinus(0.f, 0.f, -std::sin(HALF_ANGLE), std::cos(HALF_ANGLE));
    const Eigen::Vector4f qIdentity(0.f, 0.f, 0.f, 1.f);

    Eigen::Vector4f avg = utils::averageQuaternion({qPlus, qMinus});

    EXPECT_NEAR(std::abs(avg.dot(qIdentity)), 1.f, 1e-5f);
}

TEST(UtilsTest, AverageQuaternionOfEmptySetIsZero)
{
    EXPECT_TRUE(utils::averageQuaternion({}).isZero());
}

TEST(UtilsTest, MeanReprojectionErrorOfPerfectProjectionIsZero)
{
    const cv::Matx33d K(800, 0, 320,
                        0, 800, 240,
                        0, 0, 1);
    const cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    const cv::Vec3d rvec(0.1, -0.05, 0.02);
    const cv::Vec3d tvec(0.2, -0.1, 0.5);

    std::vector<cv::Point3f> objectPnts;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            objectPnts.push_back(cv::Point3f(0.1f * i - 0.2f, 0.1f * j - 0.2f, 2.f + 0.05f * i));

    std::vector<cv::Point2f> imagePnts;
    cv::projectPoints(objectPnts, rvec, tvec, K, distCoeffs, imagePnts);

    EXPECT_NEAR(utils::calculateMeanReprojectionError(imagePnts, objectPnts, K, distCoeffs,
                                                      rvec, tvec),
                0.0, 1e-6);

    // restricted to a subset of indices
    EXPECT_NEAR(utils::calculateMeanReprojectionError(imagePnts, objectPnts, K, distCoeffs,
                                                      rvec, tvec, {0, 3, 7}),
                0.0, 1e-6);
}

TEST(UtilsTest, Image2CameraInfoTopic)
{
    EXPECT_EQ(utils::image2CameraInfoTopic("/camera/image_color", ""),
              "/camera/camera_info");
    EXPECT_EQ(utils::image2CameraInfoTopic("/stereo/left/image_raw", ""),
              "/stereo/left/camera_info");
    EXPECT_EQ(utils::image2CameraInfoTopic("/camera/image_color", "/thermal/"),
              "/thermal/camera_info");
}

TEST(UtilsTest, GeometryPoseTfTransformRoundTrip)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = -2.0;
    pose.position.z = 0.5;
    tf2::Quaternion q;
    q.setRPY(0.1, -0.2, 0.3);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    tf2::Transform transform;
    utils::cvtGeometryPose2TfTransform(pose, transform);

    geometry_msgs::msg::Pose poseOut;
    utils::cvtTfTransform2GeometryPose(transform, poseOut);

    EXPECT_NEAR(poseOut.position.x, pose.position.x, 1e-12);
    EXPECT_NEAR(poseOut.position.y, pose.position.y, 1e-12);
    EXPECT_NEAR(poseOut.position.z, pose.position.z, 1e-12);
    EXPECT_NEAR(poseOut.orientation.x, pose.orientation.x, 1e-12);
    EXPECT_NEAR(poseOut.orientation.w, pose.orientation.w, 1e-12);
}

TEST(UtilsTest, GeometryTransformTfTransformRoundTrip)
{
    geometry_msgs::msg::Transform transformMsg;
    transformMsg.translation.x = -0.3;
    transformMsg.translation.y = 1.2;
    transformMsg.translation.z = 4.5;
    tf2::Quaternion q;
    q.setRPY(-0.4, 0.1, 1.5);
    transformMsg.rotation.x = q.x();
    transformMsg.rotation.y = q.y();
    transformMsg.rotation.z = q.z();
    transformMsg.rotation.w = q.w();

    tf2::Transform transform;
    utils::cvtGeometryTransform2TfTransform(transformMsg, transform);

    geometry_msgs::msg::Transform transformMsgOut;
    utils::cvtTfTransform2GeometryTransform(transform, transformMsgOut);

    EXPECT_NEAR(transformMsgOut.translation.x, transformMsg.translation.x, 1e-12);
    EXPECT_NEAR(transformMsgOut.rotation.z, transformMsg.rotation.z, 1e-12);
}

TEST(UtilsTest, CameraExtrinsicsTfTransformRoundTrip)
{
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(0.5, -1.0, 2.0));
    tf2::Quaternion q;
    q.setRPY(0.2, 0.3, -0.1);
    transform.setRotation(q);

    lib3d::Extrinsics extrinsics;
    utils::setCameraExtrinsicsFromTfTransform(transform, extrinsics);

    tf2::Transform transformOut;
    utils::setTfTransformFromCameraExtrinsics(extrinsics, transformOut);

    EXPECT_NEAR(transformOut.getOrigin().x(), transform.getOrigin().x(), 1e-5);
    EXPECT_NEAR(transformOut.getOrigin().y(), transform.getOrigin().y(), 1e-5);
    EXPECT_NEAR(transformOut.getOrigin().z(), transform.getOrigin().z(), 1e-5);
    EXPECT_NEAR(std::abs(transformOut.getRotation().dot(transform.getRotation())), 1.0, 1e-5);
}

TEST(UtilsTest, SetCameraIntrinsicsFromCameraInfo)
{
    sensor_msgs::msg::CameraInfo camInfo;
    camInfo.width  = 640;
    camInfo.height = 480;
    camInfo.k      = {800, 0, 320, 0, 810, 240, 0, 0, 1};
    camInfo.d      = {0.1, -0.05, 0.001, 0.002, 0.0};
    camInfo.p      = {790, 0, 315, 0, 0, 795, 235, 0, 0, 0, 1, 0};

    lib3d::Intrinsics intrinsics;
    utils::setCameraIntrinsicsFromCameraInfo(camInfo, intrinsics, multisensor_calibration::DISTORTED);
    EXPECT_EQ(intrinsics.getImageSize(), cv::Size(640, 480));
    EXPECT_NEAR(intrinsics.getK_as3x3()(0, 0), 800.0, 1e-9);
    EXPECT_NEAR(intrinsics.getK_as3x3()(1, 2), 240.0, 1e-9);
    EXPECT_EQ(intrinsics.getDistortionCoeffs().total(), 5u);

    utils::setCameraIntrinsicsFromCameraInfo(camInfo, intrinsics,
                                             multisensor_calibration::UNDISTORTED);
    EXPECT_NEAR(intrinsics.getK_as3x3()(0, 0), 790.0, 1e-9);
    EXPECT_TRUE(intrinsics.getDistortionCoeffs().empty());
}

TEST(UtilsTest, RootMeanSquaredErrorOfAlignedCloudsIsZero)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < 10; ++i)
        for (int j = 0; j < 10; ++j)
            pCloud->push_back(pcl::PointXYZ(0.1f * i, 0.1f * j, 1.f + 0.01f * i));

    double rmse = utils::calculateRootMeanSquaredError<pcl::PointXYZ, pcl::PointXYZ>(
      pCloud, pCloud);
    EXPECT_NEAR(rmse, 0.0, 1e-6);
}

TEST(UtilsTest, RootMeanSquaredErrorWithCorrectingTransformIsZero)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pRefCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < 10; ++i)
        for (int j = 0; j < 10; ++j)
            pRefCloud->push_back(pcl::PointXYZ(0.1f * i, 0.1f * j, 1.f));

    // source cloud shifted by -1 along x; transform shifting it back
    Eigen::Matrix4f shift = Eigen::Matrix4f::Identity();
    shift(0, 3)           = -1.f;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pSrcCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*pRefCloud, *pSrcCloud, shift);

    Eigen::Matrix4f correction = Eigen::Matrix4f::Identity();
    correction(0, 3)           = 1.f;

    double rmse = utils::calculateRootMeanSquaredError<pcl::PointXYZ, pcl::PointXYZ>(
      pSrcCloud, pRefCloud, correction);
    EXPECT_NEAR(rmse, 0.0, 1e-5);

    // without correction the error equals the shift
    double rmseShifted = utils::calculateRootMeanSquaredError<pcl::PointXYZ, pcl::PointXYZ>(
      pSrcCloud, pRefCloud, Eigen::Matrix4f(Eigen::Matrix4f::Identity()));
    EXPECT_GT(rmseShifted, 0.1);
}

TEST(UtilsTest, MeanBoardPoseDifferenceOfEqualPosesIsZero)
{
    std::vector<std::pair<cv::Vec3d, cv::Vec3d>> poses = {
      {cv::Vec3d(0, 1, 0), cv::Vec3d(1, 2, 3)},
      {cv::Vec3d(0, 1, 0), cv::Vec3d(-1, 0, 2)}};

    cv::Vec3d meanTranslDiff;
    double meanAngularError = 0.0;
    utils::calculateMeanBoardPoseDifference(poses, poses, cv::Matx44d::eye(),
                                            meanTranslDiff, meanAngularError);

    EXPECT_NEAR(cv::norm(meanTranslDiff), 0.0, 1e-9);
    EXPECT_NEAR(meanAngularError, 0.0, 1e-6);
}

TEST(UtilsTest, ComputeOrientedBoundingBoxOfPlanarBoard)
{
    // planar board of 1.2 x 0.8 m in the x-y plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = -30; i <= 30; ++i)
        for (int j = -20; j <= 20; ++j)
            pCloud->push_back(pcl::PointXYZ(0.02f * i, 0.02f * j, 0.f));

    multisensor_calibration::CalibrationTarget target;

    float width = 0.f, height = 0.f, depth = 0.f;
    Eigen::Matrix4f bboxTransform;
    utils::computeOrientedBoundingBox<pcl::PointXYZ>(pCloud, target, false,
                                                     width, height, depth, bboxTransform);

    EXPECT_NEAR(width, 1.2f, 0.05f);
    EXPECT_NEAR(height, 0.8f, 0.05f);
    EXPECT_NEAR(depth, 0.f, 1e-3f);
    EXPECT_NEAR(bboxTransform(0, 3), 0.f, 1e-3f);
    EXPECT_NEAR(bboxTransform(1, 3), 0.f, 1e-3f);
}

TEST(UtilsTest, BackupFileRenamesFile)
{
    fs::path tmpDir = fs::temp_directory_path() / "msc_utils_backup_test";
    fs::create_directories(tmpDir);
    fs::path filePath = tmpDir / "calib_results.txt";
    {
        std::ofstream file(filePath);
        file << "content";
    }

    utils::backupFile(filePath);

    EXPECT_FALSE(fs::exists(filePath));
    int numBackups = 0;
    for (const auto& entry : fs::directory_iterator(tmpDir))
    {
        EXPECT_EQ(entry.path().extension(), ".txt");
        EXPECT_NE(entry.path().stem().string().find("calib_results_"), std::string::npos);
        ++numBackups;
    }
    EXPECT_EQ(numBackups, 1);

    fs::remove_all(tmpDir);
}
