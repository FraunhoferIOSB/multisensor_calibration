/***********************************************************************
*
* Copyright (c) Fraunhofer Institute of Optronics,
* System Technologies and Image Exploitation IOSB
*
**********************************************************************/
/***********************************************************************
*
* Entry-point dispatch for the calibration GUI applications.
*
**********************************************************************/

#ifndef MULTISENSORCALIBRATION_UI_CALIBRATIONAPPLICATION_H
#define MULTISENSORCALIBRATION_UI_CALIBRATIONAPPLICATION_H

// Std
#include <string>

namespace multisensor_calibration
{

/// Available calibration GUI applications.
enum class ECalibrationApplication
{
    MULTI_SENSOR_CALIBRATION,
    EXTRINSIC_CAMERA_LIDAR_CALIBRATION,
    EXTRINSIC_CAMERA_REFERENCE_CALIBRATION,
    EXTRINSIC_LIDAR_LIDAR_CALIBRATION,
    EXTRINSIC_LIDAR_REFERENCE_CALIBRATION,
    EXTRINSIC_CAMERA_CAMERA_CALIBRATION
};

/**
 * @brief Runs the calibration application of the given type: initializes ROS and Qt,
 * launches the corresponding calibration and guidance nodes as well as the GUI, and
 * blocks until the application exits.
 *
 * This is compiled once into the library so that the per-application executables
 * reduce to a trivial main().
 *
 * @param[in] iAppType Application to run.
 * @param[in] iAppName Application/node name, typically the executable name.
 * @param[in] argc, argv Command line arguments as passed to main().
 *
 * @return Application exit code.
 */
int runCalibrationApplication(ECalibrationApplication iAppType,
                              const std::string& iAppName,
                              int argc, char** argv);

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_UI_CALIBRATIONAPPLICATION_H
