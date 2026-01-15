/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

// Std
#include <iostream>
#include <locale>
#include <memory>

// ROS
#include <rclcpp/rclcpp.hpp>

// QT
#include <QApplication>

// multisensor_calibration
#include "multisensor_calibration/calibration/ExtrinsicCameraLidarCalibration.h"
#include "multisensor_calibration/calibration/ExtrinsicCameraReferenceCalibration.h"
#include "multisensor_calibration/calibration/ExtrinsicLidarLidarCalibration.h"
#include "multisensor_calibration/calibration/ExtrinsicLidarReferenceCalibration.h"
#include "multisensor_calibration/calibration/ExtrinsicLidarVehicleCalibration.h"
#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/guidance/GuidedCameraLidarTargetPlacementNode.h"
#include "multisensor_calibration/guidance/GuidedLidarLidarTargetPlacementNode.h"
#include "multisensor_calibration/ui/CalibrationGuiBase.h"
#include "multisensor_calibration/ui/CameraLidarCalibrationGui.h"
#include "multisensor_calibration/ui/CameraReferenceCalibrationGui.h"
#include "multisensor_calibration/ui/GuiBase.h"
#include "multisensor_calibration/ui/LidarLidarCalibrationGui.h"
#include "multisensor_calibration/ui/LidarReferenceCalibrationGui.h"
#include "multisensor_calibration/ui/MultiSensorCalibrationGui.h"

#if !defined(TARGET_NAME)
#define TARGET_NAME ""
#endif

/**
 * @brief Main entry point for all calibration programs.
 *
 * This will load the Gui application, as well as the calibration and guidance nodes.
 * Depending on the preprocessor define TARGET, different configurations of the GUI and
 * nodes are initialized.
 */

int main(int argc, char** argv)
{
    std::setlocale(LC_ALL, "en_US.UTF-8");

    //--- initialize ROS
    rclcpp::init(argc, argv);

    //--- initialize executor
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> pExec =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    //--- initialize Qt
    QApplication app(argc, argv);

    //--- Load nodes and setup Gui
    std::shared_ptr<multisensor_calibration::GuiBase> pGui = nullptr;

#if TARGET == MULTI_SENSOR_CALIBRATION_TARGET

    /*
     * For the ROS node 'multi_sensor_calibration', only the calibration configurator is loaded.
     */

    pGui.reset(new multisensor_calibration::MultiSensorCalibrationGui(
      TARGET_NAME,
      multisensor_calibration::GUI_SUB_NAMESPACE));
#elif TARGET == EXTRINSIC_CAMERA_LIDAR_CALIBRATION_TARGET

    /*
     * For 'extrinsic_camera_lidar_calibration',
     *   - the ExtrinsicCameraLidarCalibration,
     *   - the GuidedCameraLidarTargetPlacementNode, as well as
     *   - the CameraLidarCalibrationGui
     * is loaded.
     */
#ifdef MULTI_THREADED
    auto guidanceThread = std::thread(
      [](rclcpp::NodeOptions options)
      {
          auto guidanceNode = std::make_shared<multisensor_calibration::GuidedCameraLidarTargetPlacementNode>(TARGET_NAME, options);
          rclcpp::spin(guidanceNode);
      },
      options);

    auto calibrationThread = std::thread(
      [](rclcpp::NodeOptions options)
      {
          auto calibrationNode = std::make_shared<multisensor_calibration::ExtrinsicCameraLidarCalibration>(TARGET_NAME, options);
          rclcpp::spin(calibrationNode);
      },
      options);
#else
    auto guidanceNode    = std::make_shared<multisensor_calibration::GuidedCameraLidarTargetPlacementNode>(TARGET_NAME, options);
    auto calibrationNode = std::make_shared<multisensor_calibration::ExtrinsicCameraLidarCalibration>(TARGET_NAME, options);
    pExec->add_node(guidanceNode);
    pExec->add_node(calibrationNode);
#endif

    pGui.reset(new multisensor_calibration::CameraLidarCalibrationGui(TARGET_NAME,
                                                                      multisensor_calibration::GUI_SUB_NAMESPACE));

#elif TARGET == EXTRINSIC_CAMERA_REFERENCE_CALIBRATION_TARGET

    /*
     * For 'extrinsic_camera_reference_calibration',
     *   - the ExtrinsicCameraReferenceCalibration,
     *   - the GuidedCameraLidarTargetPlacementNode, as well as
     *   - the CameraReferenceCalibrationGui
     * is loaded.
     */
#ifdef MULTI_THREADED
    auto guidanceThread = std::thread(
      [](rclcpp::NodeOptions options)
      {
          auto guidanceNode = std::make_shared<multisensor_calibration::GuidedCameraLidarTargetPlacementNode>(TARGET_NAME, options);
          rclcpp::spin(guidanceNode);
      },
      options);

    auto calibrationThread = std::thread(
      [](rclcpp::NodeOptions options)
      {
          auto calibrationNode = std::make_shared<multisensor_calibration::ExtrinsicCameraReferenceCalibration>(TARGET_NAME, options);
          rclcpp::spin(calibrationNode);
      },
      options);
#else
    auto guidanceNode    = std::make_shared<multisensor_calibration::GuidedCameraLidarTargetPlacementNode>(TARGET_NAME, options);
    auto calibrationNode = std::make_shared<multisensor_calibration::ExtrinsicCameraReferenceCalibration>(TARGET_NAME, options);
    pExec->add_node(guidanceNode);
    pExec->add_node(calibrationNode);
#endif

    pGui.reset(new multisensor_calibration::CameraReferenceCalibrationGui(TARGET_NAME,
                                                                          multisensor_calibration::GUI_SUB_NAMESPACE));
#elif TARGET == EXTRINSIC_LIDAR_LIDAR_CALIBRATION_TARGET

    /*
     * For 'extrinsic_camera_lidar_calibration',
     *   - the ExtrinsicLidarLidarCalibration,
     *   - the GuidedLidarLidarTargetPlacementNode, as well as
     *   - the LidarLidarCalibrationGui
     * is loaded.
     */
#ifdef MULTI_THREADED
    auto guidanceThread = std::thread(
      [](rclcpp::NodeOptions options)
      {
          auto guidanceNode = std::make_shared<multisensor_calibration::GuidedLidarLidarTargetPlacementNode>(TARGET_NAME, options);
          rclcpp::spin(guidanceNode);
      },
      options);

    auto calibrationThread = std::thread(
      [](rclcpp::NodeOptions options)
      {
          auto calibrationNode = std::make_shared<multisensor_calibration::ExtrinsicLidarLidarCalibration>(TARGET_NAME, options);
          rclcpp::spin(calibrationNode);
      },
      options);
#else
    auto guidanceNode    = std::make_shared<multisensor_calibration::GuidedLidarLidarTargetPlacementNode>(TARGET_NAME, options);
    auto calibrationNode = std::make_shared<multisensor_calibration::ExtrinsicLidarLidarCalibration>(TARGET_NAME, options);
    pExec->add_node(guidanceNode);
    pExec->add_node(calibrationNode);
#endif

    pGui.reset(new multisensor_calibration::LidarLidarCalibrationGui(TARGET_NAME,
                                                                     multisensor_calibration::GUI_SUB_NAMESPACE));
#elif TARGET == EXTRINSIC_LIDAR_REFERENCE_CALIBRATION_TARGET

    /*
     * For 'extrinsic_lidar_reference_calibration',
     *   - the ExtrinsicLidarReferenceCalibration,
     *   - the GuidedLidarLidarTargetPlacementNode, as well as
     *   - the LidarLidarCalibrationGui
     * is loaded.
     */
#ifdef MULTI_THREADED
    auto guidanceThread = std::thread(
      [](rclcpp::NodeOptions options)
      {
          auto guidanceNode = std::make_shared<multisensor_calibration::GuidedLidarLidarTargetPlacementNode>(TARGET_NAME, options);
          rclcpp::spin(guidanceNode);
      },
      options);

    auto calibrationThread = std::thread(
      [](rclcpp::NodeOptions options)
      {
          auto calibrationNode = std::make_shared<multisensor_calibration::ExtrinsicLidarReferenceCalibration>(TARGET_NAME, options);
          rclcpp::spin(calibrationNode);
      },
      options);
#else
    auto guidanceNode    = std::make_shared<multisensor_calibration::GuidedLidarLidarTargetPlacementNode>(TARGET_NAME, options);
    auto calibrationNode = std::make_shared<multisensor_calibration::ExtrinsicLidarReferenceCalibration>(TARGET_NAME, options);
    pExec->add_node(guidanceNode);
    pExec->add_node(calibrationNode);
#endif

    pGui.reset(new multisensor_calibration::LidarReferenceCalibrationGui(TARGET_NAME,
                                                                         multisensor_calibration::GUI_SUB_NAMESPACE));

#elif TARGET == STEREO_CAMERA_CALIBRATION

    /*
     * For 'extrinsic_camera_camera_calibration',
     *   - the ExtrinsicCameraCameraCalibration,
     * is loaded.
     */
#ifdef MULTI_THREADED
    auto calibrationThread = std::thread(
      [](rclcpp::NodeOptions options)
      {
          auto calibrationNode = std::make_shared<multisensor_calibration::ExtrinsicCameraCameraCalibration>(TARGET_NAME, options);
          rclcpp::spin(calibrationNode);
      },
      options);
#else
    auto calibrationNode = std::make_shared<multisensor_calibration::ExtrinsicCameraCameraCalibration>(TARGET_NAME, options);
    pExec->add_node(calibrationNode);
#endif

    /* @TODO: Camera-Camera GUI */
    pGui.reset(new multisensor_calibration::CameraLidarCalibrationGui(TARGET_NAME,
                                                                      multisensor_calibration::GUI_SUB_NAMESPACE));

#else
    std::cerr << "No valid TARGET passed as compiler define!" << std::endl;
    return 1;
#endif

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

#ifdef MULTI_THREADED
#if target != MULTI_SENSOR_CALIBRATION_TARGET
    guidanceThread.join();
    calibrationThread.join();
#endif
#endif

    return appRetVal;
}
