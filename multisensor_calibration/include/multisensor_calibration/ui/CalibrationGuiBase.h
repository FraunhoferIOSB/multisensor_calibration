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

#ifndef MULTISENSORCALIBRATION_CALIBRATIONGUIBASE_H
#define MULTISENSORCALIBRATION_CALIBRATIONGUIBASE_H

// Std
#include <memory>
#include <string>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

// Qt
#include <QLabel>
#include <QObject>
#include <QProcess>
#include <QProgressDialog>
#include <QRect>
#include <QTimer>

// multisensor_calibration
#include "../common/common.h"
#include "CalibrationControlWindow.h"
#include "GuiBase.h"
#include <multisensor_calibration_interface/srv/calibration_meta_data.hpp>

namespace interf = multisensor_calibration_interface;

namespace multisensor_calibration
{

/**
 * @ingroup ui
 * @brief Base class of all calibration GUIs.
 *
 * This holds and orchestrate the different UI elements of the calibration gui, i.e. the
 * main window as well as the data view windows.
 *
 * This also couples the ROS event loop to the Qt event loop. In this, a QTimer is instantiated
 * which periodically triggers the spinning of the ROS loop.
 *
 */
class CalibrationGuiBase : public GuiBase
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
    CalibrationGuiBase(const std::string& iAppTitle, const std::string& iGuiSubNamespace);

    /**
     * @brief Destructor
     */
    virtual ~CalibrationGuiBase();

    /**
     * @brief Get the name of the calibrator node.
     *
     */
    std::string getCalibratorNodeName() const;

    /**
     * @brief Method to call the initialization routine.
     *
     * @param[in] ipExec Pointer to executor.
     * @param[in] iNodeOpts Options for ros node wihtin gui.
     */
    bool init(const std::shared_ptr<rclcpp::Executor>& ipExec,
              const rclcpp::NodeOptions& iNodeOpts = rclcpp::NodeOptions()) override;

    /**
     * @brief Hide progress dialog and mark UI as ready.
     */
    void hideProgressDialog();

    /**
     * @brief Show progress dialog and mark UI as busy.
     *
     * @param[in] iBusyText Text to display on progress dialog.
     */
    void showProgressDialog(const QString& iBusyText);

  signals:
    /**
     * @brief Signal emitted when calibration gui is closed.
     */
    void closed();

  protected:
    /**
     * @brief Method to initialize content of the GUI elements, i.e. connect them to the data
     * publishers.
     */
    virtual void initializeGuiContents();

    /**
     * @brief Method to initialize subscribers.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    virtual bool initializeSubscribers(rclcpp::Node* ipNode);

    /**
     * @brief Pure virtual method to load visualizer.
     */
    virtual void loadVisualizer() = 0;

    /**
     * @brief Callback function handling calibration results messages.
     *
     * @param[in] ipResultMsg Calibration result message.
     */
    virtual void onCalibrationResultReceived(
      const CalibrationResult_Message_T::ConstSharedPtr& ipResultMsg);

    /**
     * @brief Callback function to handle log messages subscribed to by logSubsc_
     *
     */
    void onLogMessageReceived(const rcl_interfaces::msg::Log::ConstSharedPtr pLogMsg);

    /**
     * @brief Method to setup gui elements.
     *
     * @return False, if not successful. In this case pCalibControlWindow is null.
     */
    virtual bool setupGuiElements();

  private slots:
    /**
     * @brief Method to get calibration meta data. This is connected to the calibMetaDataTimer_.
     */
    void getCalibrationMetaData();

    /**
     * @brief Method to handle click event of 'Open Calibration workspace' menu.
     */
    void onActionOpenCalibWsTriggered() const;

    /**
     * @brief Method to handle click event of 'Open Robot workspace' menu.
     */
    void onActionOpenRobotWsTriggered() const;

    /**
     * @brief Handle triggering of preference action.
     *
     */
    void onActionPreferencesTriggered();

    /**
     * @brief Method to handle click event of 'New / Reset' menu.
     */
    void onActionResetCalibTriggered();

    /**
     * @brief Method to handle click event of 'Import Observations from Directory...' menu.
     */
    void onActionImportObservationsTriggered();

    /**
     * @brief Method to handle click event of 'Add Observation' button.
     */
    void onCaptureTargetButtonClicked();

    /**
     * @brief Method to handle click event of 'Finalize Calibration' button.
     */
    void onFinalizeCalibrationButtonClicked();

    /**
     * @brief Method to handle click event of 'Remove Observation' button.
     */
    void onRemoveObservationButtonClicked();

    /**
     * @brief Method to handle click event of 'Visualize Calibration' button.
     */
    void onVisualizeCalibrationButtonClicked();

    //--- MEMBER DECLARATION ---//

  protected:
    /// Subscriber to log messages.
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr pLogSubsc_;

    /// Subscriber to calibration result messages.
    rclcpp::Subscription<CalibrationResult_Message_T>::SharedPtr pCalibResultSubsc_;

    /// Name of the calibrator node
    std::string calibratorNodeName_;

    /// Name of the guidance node
    std::string guidanceNodeName_;

    /// Name of the node to fuse the data and visualize calibration
    std::string visualizerNodeName_;

    /// Member variable holding calibration meta data.
    std::shared_ptr<interf::srv::CalibrationMetaData::Response> pCalibrationMetaData_;

    /// QTimer object to trigger service call to get calibration meta data. This needs to be a QTimer,
    /// since the event loop runs in Qt.
    QTimer calibMetaDataTimer_;

    /// Member rectangle holding the screen geometry. This is initialized at the beginning of setting
    /// up the GUI elements.
    QRect screenGeometry_;

    /// Height of titlebar of each window. This is initialized at the beginning of setting
    /// up the GUI elements.
    int titleBarHeight_;

    /// Pointer to object of CalibrationControlWindow
    std::shared_ptr<CalibrationControlWindow> pCalibControlWindow_;

    /// Pointer to object of QProgressDialog
    std::shared_ptr<QProgressDialog> pProgressDialog_;

    /// Pointer to process of rqt_reconfigure
    std::shared_ptr<QProcess> pRqtReconfigureProcess_;

    /// Flag to signal if the current calib. mode has a visualizer
    bool hasCalibVisualizer_ = true;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_CALIBRATIONGUIBASE_H
