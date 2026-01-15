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

#ifndef MULTISENSORCALIBRATION_UI_CALIBRATIONCONFIGDIALOG_H
#define MULTISENSORCALIBRATION_UI_CALIBRATIONCONFIGDIALOG_H

// Qt
#include <QAbstractButton>
#include <QDialog>
#include <QDir>
#include <QLabel>
#include <QSettings>
#include <Qt>

// ROS
#include <rcl_interfaces/msg/log.hpp>

// multisensor_calibration
#include "../common/common.h"
#include "ExtrinsicCameraLidarConfigWidget.h"
#include "ExtrinsicCameraCameraConfigWidget.h"
#include "ExtrinsicCameraReferenceConfigWidget.h"
#include "ExtrinsicLidarLidarConfigWidget.h"
#include "ExtrinsicLidarReferenceConfigWidget.h"
#include "InstallWorkspaceDialog.h"

namespace multisensor_calibration
{

namespace Ui
{
class CalibrationConfigDialog;
}

/**
 * @ingroup ui
 * @brief Calibration configuration dialog.
 *
 * This is the main dialog which is instantiated, when running the 'multi_sensor_calibration' node.
 *
 */
class CalibrationConfigDialog : public QDialog
{
    Q_OBJECT

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Construct a new Calibration Config Dialog object.
     *
     * @param[in] parent Parent Widget
     */
    CalibrationConfigDialog(QWidget* parent = nullptr);

    /**
     * @brief Destroy the Calibration Config Dialog object.
     */
    ~CalibrationConfigDialog();

    /**
     * @brief Get selected calibration type.
     */
    ECalibrationType selectedCalibrationType() const;

    /**
     * @brief Get key-value list of calibration options that are of type bool.
     */
    std::unordered_map<std::string, bool> getBoolTypedCalibrationOptions();

    /**
     * @brief Get key-value list of calibration options that are of type double.
     */
    std::unordered_map<std::string, double> getDoubleTypedCalibrationOptions();

    /**
     * @brief Get key-value list of calibration options that are of type int.
     */
    std::unordered_map<std::string, int> getIntTypedCalibrationOptions();

    /**
     * @brief Get key-value list of calibration options that are of type string.
     */
    std::unordered_map<std::string, std::string> getStringTypedCalibrationOptions();

  public slots:

    void accept() override;

  private:
    /**
     * @brief Load settings for configurator from file.
     */
    void loadConfiguratorSettings();

    /**
     * @brief Load settings for robot from file.
     */
    void loadRobotSettings();

    /**
     * @brief Populate combo box holding different workspace folders.
     *
     */
    void populateWsFolderComboBox();

    /**
     * @brief Reset calibration options to the settings which were used last.
     */
    void resetCalibrationOptions();

    void resizeEvent(QResizeEvent* event) override;

    /**
     * @brief Save settings to file.
     */
    void saveSettings();

    void showEvent(QShowEvent* event) override;

    /**
     * @brief Set and adjust content of label showing the path of the calibration root directory.
     */
    void updateRootDirLabelContents();

  private slots:

    /**
     * @brief Handle click of button within ButtonBox
     *
     */
    void handleButtonBoxClicked(QAbstractButton* button);

    /**
     * @brief Handle change of selected calibration type.
     */
    void handleCalibrationTypeChanged();

    /**
     * @brief Handle click on push button to install workspace templates.
     */
    void handleInstallWsPushButtonClicked();

    /**
     * @brief Handle acceptance of dialog that installs workspace templates.
     *
     */
    void handleInstallWsDialogAccepted();

    /**
     * @brief Handle click on push button to open new root directory for calibration.
     */
    void handleRootDirPushButtonClicked();

    /**
     * @brief Handle click on push button to select URDF model.
     */
    void handleSelectUrdfPushButtonClicked();

    /**
     * @brief Handle change of selected workspace folder.
     */
    void handleWsFolderChanged();

    //--- MEMBER DECLARATION ---//

  private:
    /// Pointer to UI
    Ui::CalibrationConfigDialog* ui;

    /// POinter to dialog to install workspace templates.
    InstallWorkspaceDialog* pInstallWsTemplateDialog;

    /// Pointer to configuration widget for extrinsic camera-LiDAR calibration.
    ExtrinsicCameraLidarConfigWidget* pCameraLidarConfigWidget;

    /// Pointer to configuration widget for extrinsic camera-camera calibration.
    ExtrinsicCameraCameraConfigWidget* pCameraCameraConfigWidget;

    /// Pointer to configuration widget for extrinsic camera-Reference calibration.
    ExtrinsicCameraReferenceConfigWidget* pCameraReferenceConfigWidget;

    /// Pointer to configuration widget for extrinsic LiDAR-LiDAR calibration.
    ExtrinsicLidarLidarConfigWidget* pLidarLidarConfigWidget;

    /// Pointer to configuration widget for extrinsic LiDAR-Reference calibration.
    ExtrinsicLidarReferenceConfigWidget* pLidarReferenceConfigWidget;

    /// Pointer to object holding settings of configurator.
    std::shared_ptr<QSettings> pConfiguratorSettings_;

    /// Pointer to object holding settings of robot.
    std::shared_ptr<QSettings> pRobotSettings_;

    /// Root directory of all calibrations, holding individual robot workspaces.
    QDir calibrationRootDir_;
};

} // namespace multisensor_calibration
#endif // MULTISENSORCALIBRATION_UI_CALIBRATIONCONFIGDIALOG_H
