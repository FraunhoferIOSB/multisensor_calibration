// Copyright (c) 2024 - 2025 Fraunhofer IOSB and contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef MULTISENSORCALIBRATION_REFERENCEDATAPROCESSOR3D_H
#define MULTISENSORCALIBRATION_REFERENCEDATAPROCESSOR3D_H

// ROS
#include <rclcpp/rclcpp.hpp>

// multisensor_calibration
#include "../common/common.h"
#include "DataProcessor3d.h"

namespace multisensor_calibration
{

/**
 * @ingroup calibration
 * @brief Processor to compute and publish reference cloud of the calibration
 * target based on measured marker corners.
 */
class ReferenceDataProcessor3d : public DataProcessor3d
{

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Default constructor is deleted.
     *
     */
    ReferenceDataProcessor3d() = delete;

    /**
     * @brief Initialization constructor, providing the node name, the sensor name as well
     * as the path to the calibration target configuration file.
     */
    ReferenceDataProcessor3d(const std::string& iLoggerName,
                             const std::string& iSensorName,
                             const fs::path& iCalibTargetFilePath);

    /**
     * @brief Destructor
     */
    virtual ~ReferenceDataProcessor3d();

    /**
     * @brief Callback method to process the point cloud data.
     *
     * In this, first, the point cloud is transformed into a reference coordinate system if
     * according transformation is set with setDataTransform(). After that a preprocessing filter
     * is applied if it has been set using setPreprocFilter(). After the point cloud has been
     * transformed and filtered, the point cloud normals are estimated and the region growing is
     * applied. If the processing level is set to PREVIEW, only the cluster candidates are computed
     * and published. If the processing level is set to TARGET_DETECTION, the target pose is
     * detected and fit into the cloud using RANSAC as described above.
     *
     * @param iPointCloud Point cloud as pcl::PointCloud
     * @param[in] iProcLevel Level at which the data is to be processed, i.e. PREVIEW or
     * TARGET_DETECTION
     * @return Result of the processing.
     */
    EProcessingResult processData(const pcl::PointCloud<InputPointType>& iPointCloud,
                                  const EProcessingLevel& iProcLevel) override;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_REFERENCE3DDATAPROCESSOR_H