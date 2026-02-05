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
// ARE DISCLAIMED IN ANY EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/common/lib3D/core/extrinsics.hpp"
#include "multisensor_calibration/sensor_data_processing/CameraDataProcessor.h"
#include "multisensor_calibration/sensor_data_processing/SensorDataProcessorBase.h"
#include <cstddef>
#include <multisensor_calibration/calibration/Extrinsic2d2dCalibrationBase.h>
#include <numeric>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace multisensor_calibration
{

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
Extrinsic2d2dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::Extrinsic2d2dCalibrationBase(ECalibrationType type) :
  ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>(type)
{
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
Extrinsic2d2dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::~Extrinsic2d2dCalibrationBase()
{
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
double Extrinsic2d2dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::runStereoMatching(
  const std::vector<std::vector<cv::Point3f>>& iMarkerPointsRelative,
  const std::vector<std::vector<cv::Point2f>>& iSrcCamObs,
  const std::vector<std::vector<cv::Point2f>>& iRefCamObs,
  lib3d::Intrinsics const& ioSrcCameraIntrinsics,
  lib3d::Intrinsics const& ioRefCameraIntrinsics,
  float inlierMaxRpjError,
  bool refineIntrinsics,
  lib3d::Extrinsics& oNewSensorExtrinsics) const
{
    UNUSED_VAR(inlierMaxRpjError);
    UNUSED_VAR(refineIntrinsics);

    auto srcIntrinsics = ioSrcCameraIntrinsics.getK_as3x3();
    auto srcDistCoeff  = ioSrcCameraIntrinsics.getDistortionCoeffs();

    auto refIntrinsics = ioRefCameraIntrinsics.getK_as3x3();
    auto refDistCoeff  = ioRefCameraIntrinsics.getDistortionCoeffs();

    cv::Size imageSize(ioSrcCameraIntrinsics.getWidth(), ioSrcCameraIntrinsics.getHeight());

    std::vector<lib3d::Extrinsics> iterationsExtrinsics;

    cv::Mat rotation, translation, E, F;
    std::vector<double> error;
    double meanError = cv::stereoCalibrate(iMarkerPointsRelative,
                        iSrcCamObs,
                        iRefCamObs,
                        srcIntrinsics, srcDistCoeff,
                        refIntrinsics, refDistCoeff,
                        imageSize,
                        rotation,
                        translation,
                        E, F,
                        cv::CALIB_FIX_INTRINSIC | cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_FIX_FOCAL_LENGTH | cv::CALIB_ZERO_TANGENT_DIST);

    oNewSensorExtrinsics = lib3d::Extrinsics(rotation, translation, lib3d::Extrinsics::REF_2_LOCAL);

    return meanError;
}

template class Extrinsic2d2dCalibrationBase<CameraDataProcessor, CameraDataProcessor>;

} // namespace multisensor_calibration
