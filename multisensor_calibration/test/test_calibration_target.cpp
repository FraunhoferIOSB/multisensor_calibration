/***********************************************************************
 *
 * Copyright (c) Fraunhofer Institute of Optronics,
 * System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/
/***********************************************************************
 *
 * Unit tests for CalibrationTarget and the cutout geometry classes.
 *
 **********************************************************************/

// Std
#include <filesystem>

// GTest
#include <gtest/gtest.h>

// multisensor_calibration
#include <multisensor_calibration/calibration_target/CalibrationTarget.hpp>
#include <multisensor_calibration/calibration_target/CircularCutout.h>

using multisensor_calibration::CalibrationTarget;
using multisensor_calibration::CircularCutout;

namespace
{

std::string targetConfigPath()
{
    return (std::filesystem::path(TEST_DATA_DIR) / "TargetWithCirclesAndAruco.yaml").string();
}

CalibrationTarget loadTestTarget()
{
    CalibrationTarget target;
    target.readFromYamlFile(targetConfigPath());
    return target;
}

} // namespace

TEST(CircularCutoutTest, GeometryAccessors)
{
    CircularCutout cutout({0.5f, -0.25f, 0.1f});

    EXPECT_EQ(cutout.getGeometryId(), 1);
    EXPECT_EQ(cutout.getNumCoefficients(), 3);
    EXPECT_FLOAT_EQ(cutout.getCenterX(), 0.5f);
    EXPECT_FLOAT_EQ(cutout.getCenterY(), -0.25f);
    EXPECT_FLOAT_EQ(cutout.getRadius(), 0.1f);
}

TEST(CircularCutoutTest, PointInsideTest)
{
    CircularCutout cutout({0.f, 0.f, 0.1f});

    EXPECT_TRUE(cutout.isPointInside(0.f, 0.f));
    EXPECT_TRUE(cutout.isPointInside(0.05f, 0.05f));
    EXPECT_FALSE(cutout.isPointInside(0.2f, 0.f));
    EXPECT_FALSE(cutout.isPointInside(0.08f, 0.08f));
}

TEST(CalibrationTargetTest, ReadFromYamlFile)
{
    CalibrationTarget target = loadTestTarget();

    EXPECT_FLOAT_EQ(target.boardSize.width, 1.2f);
    EXPECT_FLOAT_EQ(target.boardSize.height, 0.8f);
    EXPECT_FLOAT_EQ(target.markerSize, 0.18f);
    ASSERT_EQ(target.markerIds.size(), 4u);
    EXPECT_EQ(target.markerIds, (std::vector<int>{1, 2, 3, 4}));
    ASSERT_EQ(target.markerPositions.size(), 4u);
    EXPECT_FLOAT_EQ(target.markerPositions[0].x, -0.55f);
    EXPECT_FLOAT_EQ(target.markerPositions[0].y, 0.35f);
    ASSERT_EQ(target.pBoardCutouts.size(), 3u);
    EXPECT_FLOAT_EQ(target.pBoardCutouts[0]->getRadius(), 0.12f);
    EXPECT_EQ(target.minMarkerDetection, 2);
    EXPECT_TRUE(target.isValid());

    EXPECT_TRUE(std::filesystem::path(target.cadModelMeshPath).is_absolute());
    EXPECT_TRUE(std::filesystem::exists(target.cadModelMeshPath));
    EXPECT_TRUE(std::filesystem::exists(target.cadModelCloudPath));
}

TEST(CalibrationTargetTest, DefaultConstructedIsInvalid)
{
    CalibrationTarget target;
    EXPECT_FALSE(target.isValid());
}

TEST(CalibrationTargetTest, LocalPointInlierTest)
{
    CalibrationTarget target = loadTestTarget();

    // center of board, outside any cutout
    EXPECT_TRUE(target.isLocalPointInlier(0.f, 0.f));
    // inside top-left cutout (center -0.15, 0.15, radius 0.12)
    EXPECT_FALSE(target.isLocalPointInlier(-0.15f, 0.15f));
    EXPECT_TRUE(target.isLocalPointInsideCutout(-0.15f, 0.15f));
    // outside board
    EXPECT_FALSE(target.isLocalPointInlier(0.7f, 0.f));
    EXPECT_FALSE(target.isLocalPointInlier(0.f, 0.5f));
    // close to board edge but inside
    EXPECT_TRUE(target.isLocalPointInlier(0.55f, 0.35f));
}

TEST(CalibrationTargetTest, ComputeDecomposePoseRoundTrip)
{
    const Eigen::Vector3f center(0.5f, -0.3f, 2.0f);
    Eigen::Vector3f up(0.1f, 1.f, 0.05f);
    Eigen::Vector3f normal = up.cross(Eigen::Vector3f(1.f, 0.f, 0.f)).cross(up);
    up.normalize();
    normal.normalize();

    lib3d::Extrinsics pose;
    CalibrationTarget::computePose(center, up, normal, pose);

    Eigen::Vector3f outCenter, outUp, outNormal;
    CalibrationTarget::decomposePose(pose, outCenter, outUp, outNormal);

    EXPECT_NEAR((outCenter - center).norm(), 0.f, 1e-5f);
    EXPECT_NEAR((outUp - up).norm(), 0.f, 1e-5f);
    EXPECT_NEAR((outNormal - normal).norm(), 0.f, 1e-5f);
}

TEST(CalibrationTargetTest, MarkerCornersPoseRoundTrip)
{
    CalibrationTarget target = loadTestTarget();

    const Eigen::Vector3f center(0.2f, 0.1f, 3.0f);
    const Eigen::Vector3f up     = Eigen::Vector3f(0.f, 1.f, 0.2f).normalized();
    const Eigen::Vector3f normal = Eigen::Vector3f(1.f, 0.f, 0.f).cross(up).normalized();

    std::vector<uint> markerIds;
    std::vector<std::array<cv::Point3f, 4>> markerCorners;
    target.computeMarkerCornersFromPose<cv::Point3f>(center, up, normal,
                                                     markerIds, markerCorners);

    ASSERT_EQ(markerIds.size(), 4u);
    ASSERT_EQ(markerCorners.size(), 4u);

    std::vector<cv::Point3f> tlCorners;
    for (const auto& corners : markerCorners)
        tlCorners.push_back(corners[0]);

    Eigen::Vector3f outCenter, outUp, outRight, outNormal;
    target.computePoseFromTopLeftMarkerCorners<cv::Point3f>(markerIds, tlCorners,
                                                            outCenter, outUp, outRight,
                                                            outNormal);

    EXPECT_NEAR((outUp - up).norm(), 0.f, 1e-4f);
    EXPECT_NEAR((outNormal - normal).norm(), 0.f, 1e-4f);
    EXPECT_NEAR((outCenter - center).norm(), 0.f, 1e-3f);
}

TEST(CalibrationTargetTest, PlaneParametersFromPose)
{
    // board at z = 2 facing the origin along -z
    lib3d::Extrinsics pose(lib3d::Extrinsics::LOCAL_2_REF);
    pose.setTranslationVec(0.0, 0.0, 2.0);
    pose.setRotationMat(cv::Matx33d::eye());

    cv::Vec4d plane;
    CalibrationTarget::computePlaneParametersFromPose(pose, plane);

    // normal along z, orthogonal distance 2
    EXPECT_NEAR(std::abs(plane(2)), 1.0, 1e-9);
    EXPECT_NEAR(plane(0), 0.0, 1e-9);
    EXPECT_NEAR(plane(1), 0.0, 1e-9);
    EXPECT_NEAR(plane(3), -2.0 * plane(2), 1e-9);
}
