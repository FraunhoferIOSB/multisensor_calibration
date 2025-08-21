/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/sensor_data_processing/ReferenceDataProcessor3d.h"

namespace multisensor_calibration
{

//==================================================================================================
ReferenceDataProcessor3d::ReferenceDataProcessor3d(const std::string& iLoggerName,
                                                   const std::string& iSensorName,
                                                   const fs::path& iCalibTargetFilePath) :
  DataProcessor3d(iLoggerName, iSensorName, iCalibTargetFilePath)
{
}

//==================================================================================================
ReferenceDataProcessor3d::~ReferenceDataProcessor3d()
{
}

//==================================================================================================
ReferenceDataProcessor3d::EProcessingResult ReferenceDataProcessor3d::processData(
  const pcl::PointCloud<InputPointType>& iPointCloud,
  const EProcessingLevel& iProcLevel)
{
    UNUSED_VAR(iPointCloud);
    UNUSED_VAR(iProcLevel);

    return SUCCESS;
}

} // namespace multisensor_calibration