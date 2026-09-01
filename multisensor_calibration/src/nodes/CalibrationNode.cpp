/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/
/***********************************************************************
*
* Main entry point for all calibration executables. The application to run
* is selected via the compile definition CALIBRATION_APP_TYPE.
*
**********************************************************************/

#include "multisensor_calibration/ui/CalibrationApplication.h"

#if !defined(TARGET_NAME)
#define TARGET_NAME ""
#endif

#if !defined(CALIBRATION_APP_TYPE)
#error "CALIBRATION_APP_TYPE must be defined to a value of ECalibrationApplication."
#endif

int main(int argc, char** argv)
{
    return multisensor_calibration::runCalibrationApplication(
      multisensor_calibration::ECalibrationApplication::CALIBRATION_APP_TYPE,
      TARGET_NAME, argc, argv);
}
