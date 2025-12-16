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

#ifndef MULTISENSORCALIBRATION_EXTRINSIC3D3DCALIBRATIONBASE_H
#define MULTISENSORCALIBRATION_EXTRINSIC3D3DCALIBRATIONBASE_H

// ROS
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>

// PCL
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>

// small_gicp
#include <small_gicp/registration/registration_helper.hpp>

// multisensor_calibration
#include "ExtrinsicCalibrationBase.h"

namespace multisensor_calibration
{

/**
 * @ingroup calibration
 * @brief Base class for all extrinsic 3D-3D calibration routines.
 *
 * This subclasses multisensor_calibration::ExtrinsicCalibrationBase.
 *
 * @tparam SrcDataProcessorT Class to process data from source sensor.
 * @tparam RefDataProcessorT Class to process data from reference sensor.
 */
template <class SrcDataProcessorT, class RefDataProcessorT>
class Extrinsic3d3dCalibrationBase
  : public ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>
{

    //--- METHOD DECLARATION ---/

  public:
    /**
     * @brief Default constructor is deleted.
     */
    Extrinsic3d3dCalibrationBase() = delete;

    /**
     * @brief Initialization constructor.
     *
     * @param[in] type Type of calibration
     */
    Extrinsic3d3dCalibrationBase(ECalibrationType type);

    /**
     * @brief Destructor
     */
    virtual ~Extrinsic3d3dCalibrationBase() = default;

  protected:
    /**
     * @brief Compute extrinsic pose from 3D correspondences.
     *
     * @tparam PointT Point type of input point clouds.
     * @param[in] pSrcCloud Point cloud for which the rigid transformation is to be computed.
     * @param[in] pRefCloud Point cloud used as reference.
     * @param[in] pointCorrespondences List of correspondences between point clouds.
     * @return Extrinsic sensor pose from pSrcCloud to pRefCloud.
     */
    template <typename PointT>
    lib3d::Extrinsics computeExtrinsicsFromPointCorrespondences(
      const typename pcl::PointCloud<PointT>::Ptr pSrcCloud,
      const typename pcl::PointCloud<PointT>::Ptr pRefCloud,
      const pcl::Correspondences& pointCorrespondences);

    /**
     * @brief @brief Method to run ICP between fused clouds of the calibration targets from source LiDAR
     * and of the calibration targets from reference LiDAR. The ICP is initialized with the
     * pose guess stored in sensorExtrinsics_.
     *
     * The transformation estimated by the ICP is only stored, if the RMSE after the ICP is smaller
     * than before.
     *
     * @tparam PointT Point type of input point clouds.
     * @param[in] pSrcCloud Point cloud which is to be registered with ICP.
     * @param[in] pRefCloud Point cloud against which SpSrcCloud is to be registered.
     * @param[in] icpVariant ICP variant to use (Default: GICP)
     * @param[in] maxCorrespondenceDistance Maximium distance between the point correspondences
     * @param[in] rotationToleranceDegree Accepted rotation tolerance in degrees
     * @param[in] translationTolerance Accepted translation tolerance
     * @param[in] downsamplingResolution Resolution that is used for downsampling prior to ICP
     *
     * @return Returns the minimum of RMSE before and after the ICP.
     */
    template <typename PointT>
    double runIcp(const typename pcl::PointCloud<PointT>::Ptr pSrcCloud,
                  const typename pcl::PointCloud<PointT>::Ptr pRefCloud,
                  const small_gicp::RegistrationSetting::RegistrationType& icpVariant =
                    small_gicp::RegistrationSetting::GICP,
                  const double& maxCorrespondenceDistance = 0.1,
                  const double& rotationToleranceDegree   = 0.5,
                  const double& translationTolerance      = 0.001,
                  const double& downsamplingResolution    = 0.01);

    using CalibrationBase::logger_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_EXTRINSIC3D3DCALIBRATIONBASE_H
