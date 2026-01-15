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

#ifndef MULTISENSORCALIBRATION_MULTISENSORCALIBRATIONGUI_H
#define MULTISENSORCALIBRATION_MULTISENSORCALIBRATIONGUI_H

// Qt
#include <QObject>

// multisensor_calibration
#include "CalibrationConfigDialog.h"
#include "CalibrationGuiBase.h"
#include "GuiBase.h"
#include "multisensor_calibration/calibration/CalibrationBase.h"
#include "multisensor_calibration/guidance/GuidanceBase.h"

#include <type_traits>

namespace multisensor_calibration
{

/**
 * @ingroup ui
 * @brief GUI for the multi-sensor-calibration.
 *
 * This subclasses the GuiBase and will first instantiate the CalibrationConfigDialog and then
 * instantiate the appropriate calibration according to the values given in the calibration
 * configurator.
 *
 */
class MultiSensorCalibrationGui : public GuiBase
{
    Q_OBJECT

    //--- METHOD DECLARATION ---//
  public:
    /**
     * @brief Default constructor.
     *
     * @param[in] iAppTitle Application title.
     * @param[in] iGuiSubNamespace Sub namespace of the gui.
     */
    MultiSensorCalibrationGui(const std::string& iAppTitle, const std::string& iGuiSubNamespace);

    /**
     * @brief Destructor
     */
    virtual ~MultiSensorCalibrationGui();

    /**
     * @brief Method to call the initialization routine. At the end of the routine the spin timer
     * is started to start the ROS spin loop.
     */
    bool init(const std::shared_ptr<rclcpp::Executor>& ipExec,
              const rclcpp::NodeOptions& iNodeOpts = rclcpp::NodeOptions()) override;

  private:
    /**
     * @brief Run extrinsic camera lidar calibration.
     */
    void runExtrinsicCameraLidarCalibration();

    /**
     * @brief Run extrinsic camera reference calibration.
     */
    void runExtrinsicCameraReferenceCalibration();

    /**
     * @brief Run extrinsic lidar lidar calibration.
     */
    void runExtrinsicLidarLidarCalibration();

    /**
     * @brief Run extrinsic lidar reference calibration.
     */
    void runExtrinsicLidarReferenceCalibration();


    /**
     * @brief Run camera-camera calibration
     */
    void runExtrinsicCameraCameraCalibration();

    template <typename CalibrationType, typename GuidanceType, typename GuiType>
    typename std::enable_if<
      std::is_base_of<CalibrationGuiBase, GuiType>::value &&
        std::is_base_of<CalibrationBase, CalibrationType>::value &&
        std::is_base_of<GuidanceBase, GuidanceBase>::value,
      void>::type
    runExtrinsicCalibration();

  private slots:

    /**
     * @brief Handle accepted close of config dialog.
     *
     */
    void handleConfigDialogAccepted();

    /**
     * @brief Handle termination or ROS loop.
     */
    void handleRosLoopTerminated();

    //--- MEMBER DECLARATION ---//

  protected:
    /// Pointer to object of CalibrationConfigDialog
    std::shared_ptr<CalibrationConfigDialog> pCalibConfigDialog_;

    /// Pointer to object of calibration gui
    std::shared_ptr<CalibrationGuiBase> pCalibrationGui_;

    /// Pointer to object of calibration
    std::shared_ptr<CalibrationBase> pCalibration_;
    std::thread calibrationThread_;

    /// Pointer to object of guidance
    std::shared_ptr<GuidanceBase> pGuidance_;
    std::thread guidanceThread_;

    /// Node options
    rclcpp::NodeOptions nodeOptions_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_MULTISENSORCALIBRATIONGUI_H
