/***********************************************************************
 *
 * Copyright (c) Fraunhofer Institute of Optronics,
 * System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/
/***********************************************************************
 *
 * Unit tests for the extrinsic calibration solvers (SVD from point
 * correspondences, GICP refinement, PnP) via stub subclasses.
 *
 **********************************************************************/

// Std
#include <random>

// GTest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Geometry>

// PCL
#include <pcl/common/transforms.h>

// multisensor_calibration
#include <multisensor_calibration/calibration/Extrinsic2d3dCalibrationBase.h>
#include <multisensor_calibration/calibration/Extrinsic3d3dCalibrationBase.h>
#include <multisensor_calibration/sensor_data_processing/CameraDataProcessor.h>
#include <multisensor_calibration/sensor_data_processing/LidarDataProcessor.h>

using namespace multisensor_calibration;

namespace
{

/// Exposes the protected solver methods; the pure virtuals of the node workflow are stubbed out.
class Solver3d3dStub
  : public Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>
{
  public:
    Solver3d3dStub() :
      Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>(
        EXTRINSIC_LIDAR_LIDAR_CALIBRATION)
    {
    }

    using Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>::
      computeExtrinsicsFromPointCorrespondences;
    using Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>::runIcp;
    using ExtrinsicCalibrationBase<LidarDataProcessor, LidarDataProcessor>::sensorExtrinsics_;

  private:
    bool finalizeCalibration() override
    {
        return true;
    }
    bool initializeDataProcessors() override
    {
        return true;
    }
    bool initializeSubscribers(rclcpp::Node*) override
    {
        return true;
    }
    void setupDynamicParameters(rclcpp::Node*) const override
    {
    }
    bool shutdownSubscribers() override
    {
        return true;
    }
};

class Solver2d3dStub
  : public Extrinsic2d3dCalibrationBase<CameraDataProcessor, LidarDataProcessor>
{
  public:
    Solver2d3dStub() :
      Extrinsic2d3dCalibrationBase<CameraDataProcessor, LidarDataProcessor>(
        EXTRINSIC_CAMERA_LIDAR_CALIBRATION)
    {
    }

    using Extrinsic2d3dCalibrationBase<CameraDataProcessor, LidarDataProcessor>::runPnp;
    using ExtrinsicCalibrationBase<CameraDataProcessor, LidarDataProcessor>::sensorExtrinsics_;

  private:
    bool finalizeCalibration() override
    {
        return true;
    }
    bool initializeDataProcessors() override
    {
        return true;
    }
    void setupDynamicParameters(rclcpp::Node*) const override
    {
    }
    bool shutdownSubscribers() override
    {
        return true;
    }
};

Eigen::Matrix4f makeTransform(const float iRotX, const float iRotY, const float iRotZ,
                              const Eigen::Vector3f& iTranslation)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) =
      (Eigen::AngleAxisf(iRotZ, Eigen::Vector3f::UnitZ()) *
       Eigen::AngleAxisf(iRotY, Eigen::Vector3f::UnitY()) *
       Eigen::AngleAxisf(iRotX, Eigen::Vector3f::UnitX()))
        .toRotationMatrix();
    transform.block<3, 1>(0, 3) = iTranslation;
    return transform;
}

Eigen::Matrix4f toEigen(const cv::Matx44d& iMat)
{
    Eigen::Matrix4f out;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out(r, c) = static_cast<float>(iMat(r, c));
    return out;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr makeScatteredCloud(const size_t iNumPoints)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(-2.f, 2.f);
    for (size_t i = 0; i < iNumPoints; ++i)
        pCloud->push_back(pcl::PointXYZ(dist(rng), dist(rng), dist(rng) + 4.f));
    return pCloud;
}

/// Three orthogonal planes (floor and two walls), geometry that constrains GICP in all 6 DOF.
pcl::PointCloud<pcl::PointXYZ>::Ptr makeRoomCornerCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (float a = 0.05f; a <= 1.f; a += 0.05f)
    {
        for (float b = 0.05f; b <= 1.f; b += 0.05f)
        {
            pCloud->push_back(pcl::PointXYZ(a, b, 0.f)); // floor
            pCloud->push_back(pcl::PointXYZ(a, 0.f, b)); // wall y = 0
            pCloud->push_back(pcl::PointXYZ(0.f, a, b)); // wall x = 0
        }
    }
    return pCloud;
}

} // namespace

TEST(ExtrinsicSolverTest, ComputeExtrinsicsFromPointCorrespondencesRecoversTransform)
{
    Solver3d3dStub solver;

    const Eigen::Matrix4f srcToRef = makeTransform(0.1f, -0.05f, 0.2f,
                                                   Eigen::Vector3f(0.4f, -0.2f, 0.3f));

    pcl::PointCloud<pcl::PointXYZ>::Ptr pRefCloud = makeScatteredCloud(50);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pSrcCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*pRefCloud, *pSrcCloud, Eigen::Matrix4f(srcToRef.inverse()));

    pcl::Correspondences correspondences;
    for (size_t i = 0; i < pRefCloud->size(); ++i)
        correspondences.push_back(pcl::Correspondence(static_cast<int>(i),
                                                      static_cast<int>(i), 0.f));

    lib3d::Extrinsics extrinsics =
      solver.computeExtrinsicsFromPointCorrespondences<pcl::PointXYZ>(pSrcCloud, pRefCloud,
                                                                      correspondences);

    const Eigen::Matrix4f estimated =
      toEigen(extrinsics.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF));
    EXPECT_LT((estimated - srcToRef).norm(), 1e-4f);
}

TEST(ExtrinsicSolverTest, RunIcpRefinesPerturbedPose)
{
    Solver3d3dStub solver;
    ASSERT_EQ(solver.sensorExtrinsics_.size(), 1u); // seeded with identity guess

    const Eigen::Matrix4f srcToRef =
      makeTransform(0.f, 0.f, 2.f * static_cast<float>(M_PI) / 180.f,
                    Eigen::Vector3f(0.05f, -0.03f, 0.02f));

    pcl::PointCloud<pcl::PointXYZ>::Ptr pRefCloud = makeRoomCornerCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pSrcCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*pRefCloud, *pSrcCloud, Eigen::Matrix4f(srcToRef.inverse()));

    const double rmse = solver.runIcp<pcl::PointXYZ>(pSrcCloud, pRefCloud);

    EXPECT_LT(rmse, 0.01);
    ASSERT_EQ(solver.sensorExtrinsics_.size(), 2u); // refined pose appended

    const Eigen::Matrix4f estimated =
      toEigen(solver.sensorExtrinsics_.back().getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF));
    EXPECT_LT((estimated.block<3, 1>(0, 3) - srcToRef.block<3, 1>(0, 3)).norm(), 0.01f);
    EXPECT_LT((estimated.block<3, 3>(0, 0) - srcToRef.block<3, 3>(0, 0)).norm(), 0.01f);
}

TEST(ExtrinsicSolverTest, RunPnpRecoversCameraPose)
{
    Solver2d3dStub solver;

    lib3d::Intrinsics intrinsics;
    intrinsics.setImageSize(640, 480);
    intrinsics.setBy_K(cv::Matx33d(800, 0, 320,
                                   0, 800, 240,
                                   0, 0, 1));
    intrinsics.setDistortionCoeffs(cv::Mat::zeros(5, 1, CV_64F));

    const cv::Vec3d rvecGt(0.05, -0.1, 0.02);
    const cv::Vec3d tvecGt(0.1, -0.2, 0.5);

    std::vector<cv::Point3f> lidarObs;
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> lateral(-1.f, 1.f);
    std::uniform_real_distribution<float> depth(3.f, 6.f);
    for (int i = 0; i < 30; ++i)
        lidarObs.push_back(cv::Point3f(lateral(rng), lateral(rng), depth(rng)));

    std::vector<cv::Point2f> camObs;
    cv::projectPoints(lidarObs, rvecGt, tvecGt, intrinsics.getK_as3x3(),
                      intrinsics.getDistortionCoeffs(), camObs);

    lib3d::Extrinsics newExtrinsics;
    const std::pair<double, int> result =
      solver.runPnp(camObs.cbegin(), camObs.cend(),
                    lidarObs.cbegin(), lidarObs.cend(),
                    intrinsics, 2.f, false, newExtrinsics);

    EXPECT_NEAR(result.first, 0.0, 1e-3);
    EXPECT_GE(result.second, 27); // nearly all points are inliers

    EXPECT_EQ(newExtrinsics.getTransfDirection(), lib3d::Extrinsics::REF_2_LOCAL);
    const cv::Vec3d tvecEst = newExtrinsics.getTranslationVec();
    EXPECT_NEAR(cv::norm(tvecEst - tvecGt), 0.0, 1e-3);

    const cv::Vec3d rvecEst = lib3d::Rotation(newExtrinsics.getRotationMat()).getRodriguesVec();
    EXPECT_NEAR(cv::norm(rvecEst - rvecGt), 0.0, 1e-3);
}
