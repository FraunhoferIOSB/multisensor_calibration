/***********************************************************************
*
* Copyright (c) Fraunhofer Institute of Optronics,
* System Technologies and Image Exploitation IOSB
*
**********************************************************************/
/***********************************************************************
*
* Implementation of runCalibrationApplication(): sets up ROS and Qt and
* launches the nodes and GUI of the selected calibration application.
*
**********************************************************************/

#include "multisensor_calibration/ui/CalibrationApplication.h"

// Std
#include <algorithm>
#include <clocale>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>

// Qt
#include <QApplication>
#include <QProcessEnvironment>

// multisensor_calibration
#include "multisensor_calibration/calibration/ExtrinsicCameraCameraCalibration.h"
#include "multisensor_calibration/calibration/ExtrinsicCameraLidarCalibration.h"
#include "multisensor_calibration/calibration/ExtrinsicCameraReferenceCalibration.h"
#include "multisensor_calibration/calibration/ExtrinsicLidarLidarCalibration.h"
#include "multisensor_calibration/calibration/ExtrinsicLidarReferenceCalibration.h"
#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/guidance/GuidedCameraLidarTargetPlacementNode.h"
#include "multisensor_calibration/guidance/GuidedLidarLidarTargetPlacementNode.h"
#include "multisensor_calibration/ui/CameraCameraCalibrationGui.h"
#include "multisensor_calibration/ui/CameraLidarCalibrationGui.h"
#include "multisensor_calibration/ui/CameraReferenceCalibrationGui.h"
#include "multisensor_calibration/ui/GuiBase.h"
#include "multisensor_calibration/ui/LidarLidarCalibrationGui.h"
#include "multisensor_calibration/ui/LidarReferenceCalibrationGui.h"
#include "multisensor_calibration/ui/MultiSensorCalibrationGui.h"

namespace multisensor_calibration
{

namespace
{

/**
 * @brief Launches a node of the given type. With MULTI_THREADED (release builds) the
 * node is spun in its own thread, otherwise it is added to the given executor.
 * Nodes added to the executor are stored in ioNodes, since the executor only holds
 * weak references.
 */
template <typename NodeT>
void launchNode(const std::string& iAppName,
                const rclcpp::NodeOptions& iOptions,
                const std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>& ipExec,
                std::vector<std::shared_ptr<void>>& ioNodes,
                std::vector<std::thread>& ioThreads)
{
#ifdef MULTI_THREADED
    (void)ipExec;
    (void)ioNodes;
    ioThreads.emplace_back(
      [iAppName, iOptions]()
      {
          auto pNode = std::make_shared<NodeT>(iAppName, iOptions);
          rclcpp::spin(pNode);
      });
#else
    (void)ioThreads;
    auto pNode = std::make_shared<NodeT>(iAppName, iOptions);
    ipExec->add_node(pNode);
    ioNodes.push_back(pNode);
#endif
}

} // namespace

//==================================================================================================
int runCalibrationApplication(ECalibrationApplication iAppType,
                              const std::string& iAppName,
                              int argc, char** argv)
{
    std::setlocale(LC_ALL, "en_US.UTF-8");

    std::vector<std::string> non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
    auto env                              = QProcessEnvironment::systemEnvironment();
    if (env.value("XDG_SESSION_TYPE") == "wayland" &&
        non_ros_args.end() == std::find(non_ros_args.begin(), non_ros_args.end(),
                                        "-platform") &&
        !env.contains("QT_QPA_PLATFORM"))
    {
        non_ros_args.emplace_back("-platform");
        non_ros_args.emplace_back("xcb");
    }

    std::vector<char*> non_ros_args_c_strings;
    for (auto& arg : non_ros_args)
    {
        non_ros_args_c_strings.push_back(arg.data());
    }
    int non_ros_argc = static_cast<int>(non_ros_args_c_strings.size());

    //--- initialize ROS
    rclcpp::init(argc, argv);

    //--- initialize executor
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> pExec =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    //--- Qt platform attributes must be set before QApplication construction (Qt5 requirement)
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
#endif

    //--- initialize Qt
    QApplication app(non_ros_argc, non_ros_args_c_strings.data());

    //--- launch nodes and set up GUI
    std::shared_ptr<GuiBase> pGui                = nullptr;
    std::vector<std::shared_ptr<void>> nodePtrs  = {};
    std::vector<std::thread> nodeThreads         = {};

    switch (iAppType)
    {
    case ECalibrationApplication::MULTI_SENSOR_CALIBRATION:
        pGui = std::make_shared<MultiSensorCalibrationGui>(iAppName, GUI_SUB_NAMESPACE);
        break;

    case ECalibrationApplication::EXTRINSIC_CAMERA_LIDAR_CALIBRATION:
        launchNode<GuidedCameraLidarTargetPlacementNode>(iAppName, options, pExec,
                                                         nodePtrs, nodeThreads);
        launchNode<ExtrinsicCameraLidarCalibration>(iAppName, options, pExec,
                                                    nodePtrs, nodeThreads);
        pGui = std::make_shared<CameraLidarCalibrationGui>(iAppName, GUI_SUB_NAMESPACE);
        break;

    case ECalibrationApplication::EXTRINSIC_CAMERA_REFERENCE_CALIBRATION:
        launchNode<GuidedCameraLidarTargetPlacementNode>(iAppName, options, pExec,
                                                         nodePtrs, nodeThreads);
        launchNode<ExtrinsicCameraReferenceCalibration>(iAppName, options, pExec,
                                                        nodePtrs, nodeThreads);
        pGui = std::make_shared<CameraReferenceCalibrationGui>(iAppName, GUI_SUB_NAMESPACE);
        break;

    case ECalibrationApplication::EXTRINSIC_LIDAR_LIDAR_CALIBRATION:
        launchNode<GuidedLidarLidarTargetPlacementNode>(iAppName, options, pExec,
                                                        nodePtrs, nodeThreads);
        launchNode<ExtrinsicLidarLidarCalibration>(iAppName, options, pExec,
                                                   nodePtrs, nodeThreads);
        pGui = std::make_shared<LidarLidarCalibrationGui>(iAppName, GUI_SUB_NAMESPACE);
        break;

    case ECalibrationApplication::EXTRINSIC_LIDAR_REFERENCE_CALIBRATION:
        launchNode<GuidedLidarLidarTargetPlacementNode>(iAppName, options, pExec,
                                                        nodePtrs, nodeThreads);
        launchNode<ExtrinsicLidarReferenceCalibration>(iAppName, options, pExec,
                                                       nodePtrs, nodeThreads);
        pGui = std::make_shared<LidarReferenceCalibrationGui>(iAppName, GUI_SUB_NAMESPACE);
        break;

    case ECalibrationApplication::EXTRINSIC_CAMERA_CAMERA_CALIBRATION:
        launchNode<ExtrinsicCameraCameraCalibration>(iAppName, options, pExec,
                                                     nodePtrs, nodeThreads);
        pGui = std::make_shared<CameraCameraCalibrationGui>(iAppName, GUI_SUB_NAMESPACE);
        break;
    }

    //--- initialize gui
    bool isSuccessful = (pGui != nullptr)
                          ? pGui->init(pExec, options)
                          : false;
    if (!isSuccessful)
    {
        std::cerr << "Something went wrong in the initialization of the GUI." << std::endl;
        return 1;
    }

    //--- run application
    int appRetVal = app.exec();
    pGui.reset();

    //--- shut down ROS so that spinning node threads return and can be joined
    rclcpp::shutdown();
    for (std::thread& thread : nodeThreads)
    {
        if (thread.joinable())
            thread.join();
    }

    return appRetVal;
}

} // namespace multisensor_calibration
