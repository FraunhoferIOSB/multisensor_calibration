/***********************************************************************
 *
 * Copyright (c) Fraunhofer Institute of Optronics,
 * System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/
/***********************************************************************
 *
 * Unit tests for the calibration workspace classes.
 *
 **********************************************************************/

// Std
#include <filesystem>

// GTest
#include <gtest/gtest.h>

// multisensor_calibration
#include <multisensor_calibration/io/Workspace.h>

namespace fs = std::filesystem;
using namespace multisensor_calibration;

class WorkspaceTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        logger_ = rclcpp::get_logger("workspace_test");
        tmpDir_ = fs::temp_directory_path() /
                  ("msc_ws_test_" +
                   std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
        fs::remove_all(tmpDir_);
        fs::create_directories(tmpDir_);
    }

    void TearDown() override
    {
        fs::remove_all(tmpDir_);
    }

    rclcpp::Logger logger_ = rclcpp::get_logger("");
    fs::path tmpDir_;
};

TEST_F(WorkspaceTest, LoadNonExistingWithoutForceFails)
{
    ExtrinsicCameraLidarCalibWorkspace workspace((tmpDir_ / "ws").string(), logger_);
    EXPECT_FALSE(workspace.load(false));
}

TEST_F(WorkspaceTest, ForceCreateInitializesWorkspace)
{
    const fs::path wsPath = tmpDir_ / "camera_lidar_ws";
    ExtrinsicCameraLidarCalibWorkspace workspace(wsPath.string(), logger_);

    ASSERT_TRUE(workspace.load(true));

    EXPECT_TRUE(fs::is_directory(wsPath));
    EXPECT_TRUE(fs::exists(wsPath / "settings.ini"));
    EXPECT_EQ(workspace.getPath(), wsPath);
    ASSERT_NE(workspace.settingsPtr(), nullptr);
    EXPECT_TRUE(workspace.isValid());
    EXPECT_TRUE(ExtrinsicCameraLidarCalibWorkspace::isValid(wsPath));
}

TEST_F(WorkspaceTest, ReloadingExistingWorkspaceSucceeds)
{
    const fs::path wsPath = tmpDir_ / "lidar_lidar_ws";
    {
        ExtrinsicLidarLidarCalibWorkspace workspace(wsPath.string(), logger_);
        ASSERT_TRUE(workspace.load(true));
    }

    ExtrinsicLidarLidarCalibWorkspace workspace(wsPath.string(), logger_);
    EXPECT_TRUE(workspace.load(false));
    EXPECT_TRUE(workspace.isValid());
}

TEST_F(WorkspaceTest, TypeMismatchIsRejected)
{
    const fs::path wsPath = tmpDir_ / "typed_ws";
    {
        ExtrinsicCameraLidarCalibWorkspace workspace(wsPath.string(), logger_);
        ASSERT_TRUE(workspace.load(true));
    }

    // same directory opened as a different workspace type
    ExtrinsicLidarLidarCalibWorkspace wrongWorkspace(wsPath.string(), logger_);
    EXPECT_FALSE(wrongWorkspace.load(false));
    EXPECT_FALSE(wrongWorkspace.isValid());
    EXPECT_FALSE(ExtrinsicLidarLidarCalibWorkspace::isValid(wsPath));
    EXPECT_TRUE(ExtrinsicCameraLidarCalibWorkspace::isValid(wsPath));
}

TEST_F(WorkspaceTest, IsValidOnNonWorkspaceDirectories)
{
    EXPECT_FALSE(RobotWorkspace::isValid(tmpDir_ / "does_not_exist"));

    // existing, but empty directory
    const fs::path emptyDir = tmpDir_ / "empty";
    fs::create_directories(emptyDir);
    EXPECT_FALSE(RobotWorkspace::isValid(emptyDir));
}

TEST_F(WorkspaceTest, RobotWorkspaceForceCreate)
{
    const fs::path wsPath = tmpDir_ / "robot_ws";
    RobotWorkspace workspace(wsPath.string(), logger_);

    ASSERT_TRUE(workspace.load(true));
    EXPECT_TRUE(RobotWorkspace::isValid(wsPath));
    EXPECT_FALSE(ExtrinsicCameraLidarCalibWorkspace::isValid(wsPath));
}
