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

#include <multisensor_calibration/calibration/Extrinsic2d2dCalibrationBase.h>

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
std::pair<double, int> Extrinsic2d2dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::runStereoMatching(
  const std::vector<cv::Point2f>::const_iterator iSrcCamObsBegin,
  const std::vector<cv::Point2f>::const_iterator iSrcCamObsEnd,
  const std::vector<cv::Point2f>::const_iterator iRefCamObsBegin,
  const std::vector<cv::Point2f>::const_iterator iRefCamObsEnd,
  lib3d::Intrinsics& ioSrcCameraIntrinsics,
  lib3d::Intrinsics& ioRefCameraIntrinsics,
  float inlierMaxRpjError,
  bool useIntrinsics,
  lib3d::Extrinsics& oNewSensorExtrinsics,
  const std::vector<uint>& indices) const
{
    double meanError = 0.0;
    int inlierCount  = 0;

    /* @TODO update sensorExtrinsics_ */

    return {meanError, inlierCount};
}

} // namespace multisensor_calibration
