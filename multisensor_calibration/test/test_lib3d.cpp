/***********************************************************************
 *
 * Copyright (c) Fraunhofer Institute of Optronics,
 * System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/
/***********************************************************************
 *
 * Unit tests for lib3d::Extrinsics, lib3d::Rotation and lib3d::Intrinsics.
 *
 **********************************************************************/

// GTest
#include <gtest/gtest.h>

// multisensor_calibration
#include <multisensor_calibration/common/lib3D/core/extrinsics.hpp>
#include <multisensor_calibration/common/lib3D/core/intrinsics.hpp>

namespace
{

void expectMatNear(const cv::Matx44d& iActual, const cv::Matx44d& iExpected, const double iTol)
{
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            EXPECT_NEAR(iActual(r, c), iExpected(r, c), iTol)
              << "element (" << r << ", " << c << ")";
}

} // namespace

TEST(Lib3dRotationTest, AxisRotations)
{
    // 90 degrees about z maps x onto y
    cv::Matx33d rotZ = lib3d::Rotation::createRotationZ_deg(90.0);
    cv::Vec3d mapped = rotZ * cv::Vec3d(1, 0, 0);
    EXPECT_NEAR(mapped(0), 0.0, 1e-12);
    EXPECT_NEAR(mapped(1), 1.0, 1e-12);
    EXPECT_NEAR(mapped(2), 0.0, 1e-12);

    // 90 degrees about x maps y onto z
    cv::Matx33d rotX = lib3d::Rotation::createRotationX_deg(90.0);
    mapped           = rotX * cv::Vec3d(0, 1, 0);
    EXPECT_NEAR(mapped(2), 1.0, 1e-12);

    // 90 degrees about y maps z onto x
    cv::Matx33d rotY = lib3d::Rotation::createRotationY_deg(90.0);
    mapped           = rotY * cv::Vec3d(0, 0, 1);
    EXPECT_NEAR(mapped(0), 1.0, 1e-12);
}

TEST(Lib3dRotationTest, RodriguesRoundTrip)
{
    const cv::Vec3d rodrigues(0.2, -0.4, 0.1);

    lib3d::Rotation rotation(rodrigues);
    cv::Vec3d recovered = rotation.getRodriguesVec();

    EXPECT_NEAR(recovered(0), rodrigues(0), 1e-12);
    EXPECT_NEAR(recovered(1), rodrigues(1), 1e-12);
    EXPECT_NEAR(recovered(2), rodrigues(2), 1e-12);
}

TEST(Lib3dExtrinsicsTest, DefaultIsIdentity)
{
    lib3d::Extrinsics extrinsics;
    EXPECT_EQ(extrinsics.getTransfDirection(), lib3d::Extrinsics::REF_2_LOCAL);
    expectMatNear(extrinsics.getRTMatrix(), cv::Matx44d::eye(), 1e-12);
}

TEST(Lib3dExtrinsicsTest, InverseComposition)
{
    lib3d::Extrinsics extrinsics(lib3d::Rotation::createRotationZ_deg(30.0),
                                 cv::Vec3d(1.0, -2.0, 0.5),
                                 lib3d::Extrinsics::REF_2_LOCAL);

    const cv::Matx44d rt    = extrinsics.getRTMatrix();
    const cv::Matx44d rtInv = extrinsics.getInverse().getRTMatrix();

    expectMatNear(rt * rtInv, cv::Matx44d::eye(), 1e-10);
}

TEST(Lib3dExtrinsicsTest, DirectionalRTMatrixIsInverse)
{
    lib3d::Extrinsics extrinsics(lib3d::Rotation::createRotationX_deg(45.0),
                                 cv::Vec3d(0.3, 0.1, 2.0),
                                 lib3d::Extrinsics::REF_2_LOCAL);

    const cv::Matx44d ref2Local = extrinsics.getRTMatrix(lib3d::Extrinsics::REF_2_LOCAL);
    const cv::Matx44d local2Ref = extrinsics.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF);

    expectMatNear(ref2Local * local2Ref, cv::Matx44d::eye(), 1e-10);
}

TEST(Lib3dExtrinsicsTest, SetRTMatrixWithDirectionRoundTrip)
{
    cv::Matx44d rt  = cv::Matx44d::eye();
    cv::Matx33d rot = lib3d::Rotation::createRotationXYZ_deg(10.0, -20.0, 30.0);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            rt(r, c) = rot(r, c);
    rt(0, 3) = 1.0;
    rt(1, 3) = 2.0;
    rt(2, 3) = 3.0;

    // stored as REF_2_LOCAL internally, passed as LOCAL_2_REF
    lib3d::Extrinsics extrinsics;
    extrinsics.setRTMatrix(rt, lib3d::Extrinsics::LOCAL_2_REF);

    expectMatNear(extrinsics.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF), rt, 1e-10);
}

TEST(Lib3dExtrinsicsTest, TranslationAndRotationAccessors)
{
    lib3d::Extrinsics extrinsics;
    extrinsics.setTranslationVec(1.0, 2.0, 3.0);
    extrinsics.setRotationMat(lib3d::Rotation::createRotationZ_deg(90.0));

    EXPECT_NEAR(extrinsics.getTranslationVec()(0), 1.0, 1e-12);
    EXPECT_NEAR(extrinsics.getTranslationVec()(1), 2.0, 1e-12);
    EXPECT_NEAR(extrinsics.getTranslationVec()(2), 3.0, 1e-12);
    EXPECT_NEAR(extrinsics.getRotationMat()(1, 0), 1.0, 1e-12);
}

TEST(Lib3dIntrinsicsTest, KMatrixRoundTrip)
{
    const cv::Matx33d K(800, 0, 320,
                        0, 810, 240,
                        0, 0, 1);

    lib3d::Intrinsics intrinsics;
    intrinsics.setImageSize(640, 480);
    intrinsics.setBy_K(K);

    EXPECT_EQ(intrinsics.getImageSize(), cv::Size(640, 480));

    const cv::Matx33d recovered = intrinsics.getK_as3x3();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            EXPECT_NEAR(recovered(r, c), K(r, c), 1e-9);
}

TEST(Lib3dIntrinsicsTest, DistortionCoefficients)
{
    lib3d::Intrinsics intrinsics;
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 0.1, -0.05, 0.001, 0.002, 0.0);
    intrinsics.setDistortionCoeffs(distCoeffs);

    cv::Mat recovered = intrinsics.getDistortionCoeffs();
    ASSERT_EQ(recovered.total(), 5u);
    EXPECT_NEAR(recovered.at<double>(0), 0.1, 1e-12);
    EXPECT_NEAR(recovered.at<double>(1), -0.05, 1e-12);
}
