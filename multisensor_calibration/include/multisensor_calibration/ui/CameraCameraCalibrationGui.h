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

// Std
#include <memory>

// multisensor_calibration
#include "CalibrationGuiBase.h"
#include "ImageViewDialog.h"
#include "Rviz3dViewDialog.h"
#include "multisensor_calibration/visualizers/PointCloud2ImageNode.h"

namespace multisensor_calibration
{

/**
 * @ingroup ui
 * @brief GUI for a guided extrinsic camera-LiDAR calibration.
 *
 * This subclasses the CalibrationGuiBase and will instantiate
 *  - the CalibrationControlWindow,
 *  - an ImageViewDialog to guid the user where to place the target in the camera image,
 *  - an ImageViewDialog to show results of the CameraDataProcessor, i.e. the detections of the
 *  target in the camera images (source), as well as
 *  - a RViz3dViewDialog to show the results of the LidarDataProcessor, i.e. the detections of the
 *  target in the cloud data (reference) of the LiDAR.
 *
 */
class CameraCameraCalibrationGui : public CalibrationGuiBase
{

    //--- METHOD DECLARATION ---/
  public:
    /**
     * @brief Default constructor
     *
     * @param[in] iAppTitle Application title.
     * @param[in] iGuiSubNamespace Sub namespace of the gui.
     */
    CameraCameraCalibrationGui(const std::string& iAppTitle, const std::string& iGuiSubNamespace);

    /**
     * @brief Destructor
     */
    virtual ~CameraCameraCalibrationGui();

  protected:
    /**
     * @brief Initialize the contents to the GUI elements, i.e. assign the GUI elements to assign
     * to the appropriate topics.
     */
    void initializeGuiContents() override;

    /**
     * @brief Method to load visualizer.
     */
    void loadVisualizer() override;

    /**
     * @overload
     */
    bool setupGuiElements() override;

  private:
    //--- MEMBER DECLARATION ---/

  private:
    /// Node handle
    using GuiBase::pNode_;

    /// calibration meta data
    using CalibrationGuiBase::pCalibrationMetaData_;

    /// Member pointer to dialog object used to visualize the placement guidance for the calibration
    /// target.
    std::shared_ptr<ImageViewDialog> pPlacementGuidanceDialog_;

    /// Member pointer to dialog object used to visualize the target detection in the camera image.
    std::shared_ptr<ImageViewDialog> pCameraTargetDialog_;

    /// Member pointer to dialog object used to visualize the reference cloud from the LiDAR.
    std::shared_ptr<Rviz3dViewDialog> pLidarTargetDialog_;

    /// Member pointer to dialog object used to visualize the calibration by means of sensor fusion.
    std::shared_ptr<ImageViewDialog> pFusionDialog_;

    /// Pointer to visualizer
    std::shared_ptr<visualizers::PointCloud2ImageNode> pVisualizerNode_;
};

} // namespace multisensor_calibration
