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

#pragma once

// ROS
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>

// multisensor_calibration
#include "ExtrinsicCalibrationBase.h"
#include <multisensor_calibration_interface/srv/camera_intrinsics.hpp>

#include <multisensor_calibration/common/lib3D/core/intrinsics.hpp>

namespace multisensor_calibration
{

/**
 * @ingroup calibration
 * @brief Base class for extrinsic 2D–2D calibration routines.
 *
 * @tparam SrcDataProcessorT Data processor for the source sensor
 * @tparam RefDataProcessorT Data processor for the reference sensor
 */
template <class SrcDataProcessorT, class RefDataProcessorT>
class Extrinsic2d2dCalibrationBase
  : public ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>
{
    //==============================================================================
    // CONSTRUCTION / DESTRUCTION
    //==============================================================================
  public:
    Extrinsic2d2dCalibrationBase() = delete;

    explicit Extrinsic2d2dCalibrationBase(ECalibrationType type);

    virtual ~Extrinsic2d2dCalibrationBase();

    //==============================================================================
    // METHODS
    //==============================================================================
  protected:
    double runStereoCalib(
      const std::vector<std::vector<cv::Point3f>>& iMarkerPointsRelative,
      const std::vector<std::vector<cv::Point2f>>& iSrcCamObs,
      const std::vector<std::vector<cv::Point2f>>& iRefCamObs,
      lib3d::Intrinsics const& ioSrcCameraIntrinsics,
      lib3d::Intrinsics const& ioRefCameraIntrinsics,
      bool refineIntrinsics,
      lib3d::Extrinsics& oNewSensorExtrinsics) const;
};

} // namespace multisensor_calibration
