/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

// Std
#include <filesystem>

// ROS
#include <rclcpp/rclcpp.hpp>

// multisensor_calibration
#include "multisensor_calibration/io/Workspace.h"

#if !defined(TARGET_NAME)
#define TARGET_NAME ""
#endif

namespace fs = std::filesystem;

/**
 * @brief Entry point for the 'initialize_robot_workspace' node.
 *
 * This will initialize a robot workspace with given information.
 */

class RobotWsInitializer : public rclcpp::Node
{
  public:
    RobotWsInitializer() :
      Node(TARGET_NAME),
      isInitialized_(false)
    {
        //--- Setup launch parameters
        declare_parameter<std::string>("robot_ws_path", "");
        declare_parameter<std::string>("robot_name", "");
        declare_parameter<std::string>("urdf_model_path", "");

        //--- Get the launch parameters

        // string containing read robot_ws_path
        std::string robotWsPathStr = get_parameter("robot_ws_path").as_string();
        if (robotWsPathStr.empty())
        {
            RCLCPP_ERROR(this->get_logger(),
                         "None or empty path string passed to 'robot_ws_path'."
                         "\nPlease provide valid path to robot workspace.");
            return;
        }

        // name of robot
        std::string robotName = get_parameter("robot_name").as_string();
        if (robotName.empty())
        {
            RCLCPP_ERROR(this->get_logger(),
                         "None or empty string passed to 'robot_name'."
                         "\nPlease provide valid robot name.");
            return;
        }

        // string containing read robot_ws_path
        std::string urdfModelPathStr = get_parameter("urdf_model_path").as_string();

        //--- initialize workspace
        multisensor_calibration::RobotWorkspace robotWs(robotWsPathStr, this->get_logger());
        if (robotWs.isValid())
        {
            RCLCPP_WARN(this->get_logger(),
                        "Robot workspace already exists and will not be reinitialized."
                        "\nWorkspace path: %s",
                        robotWsPathStr.c_str());
            return;
        }
        else if (fs::exists(fs::absolute(robotWsPathStr)))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Given path to workspace already exists but is not a robot workspace."
                        "No initialization is performed."
                        "\nWorkspace path: %s",
                        robotWsPathStr.c_str());
            return;
        }

        robotWs.initialize();
        robotWs.load();

        //--- save settings
        QSettings* pSettings = robotWs.settingsPtr();
        pSettings->setValue("robot/name", QString::fromStdString(robotName));
        pSettings->setValue("robot/urdf_model_path", QString::fromStdString(urdfModelPathStr));
        pSettings->sync();

        RCLCPP_INFO(this->get_logger(),
                    "Robot workspace successfully initialized."
                    "\nWorkspace path: %s",
                    robotWsPathStr.c_str());

        isInitialized_ = true;
    }

    bool isInitialized() const
    {
        return isInitialized_;
    }

  private:
    bool isInitialized_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotWsInitializer>();
    if (!node->isInitialized())
        return 1;

    rclcpp::shutdown();
    return 0;
}