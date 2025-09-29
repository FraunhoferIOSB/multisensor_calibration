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

#ifndef MULTISENSORCALIBRATION_UI_OBSERVATIONSVIEWDIALOG_H
#define MULTISENSORCALIBRATION_UI_OBSERVATIONSVIEWDIALOG_H

// std
#include <memory>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

// Qt
#include <QAbstractButton>
#include <QDialog>

namespace multisensor_calibration
{

class CalibrationGuiBase;

namespace Ui
{
class ObservationsViewDialog;
}

/**
 * @ingroup ui
 * @brief Dialog class to allow to enter observations through a table.
 */
class ObservationsViewDialog : public QDialog
{
    Q_OBJECT

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Constructor
     *
     * @param parent
     */
    ObservationsViewDialog(CalibrationGuiBase* calibrationGui,
                           QWidget* parent = nullptr);

    /**
     * @brief Destructor
     *
     */
    ~ObservationsViewDialog();

    /**
     * @brief Set the name of the sensor. Needed to call the correct service.
     */
    void setSensorName(const std::string& name);

    /**
     * @brief Initialize TF listener.
     *
     * @param[in] ipNode Pointer to node.
     */
    void initializeTfListener(rclcpp::Node* ipNode);

  private slots:

    /**
     * @brief handle QT signal emitted by QButtonBox whenever a button is pressed.
     *
     */
    void handleButtonBoxClicked(QAbstractButton* button);

    /**
     * @brief Handle QT signal emitted by QTableWidget whenever the data of the item in the cell
     * has changed.
     *
     * @param[in] row Row of the cell which has changed
     * @param[in] column Column of the cell which has changed
     */
    void handleTableWidgetCellChanged(int row, int column);

    void handleTableWidgetContextMenuRequest(const QPoint& pos);

    //--- MEMBER DECLARATION ---//

  private:
    /// Pointer to base class of calibration gui.
    CalibrationGuiBase* pCalibrationGui_;

    /// Pointer to UI
    Ui::ObservationsViewDialog* pUi_;

    /// TF buffer needed for listener
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;

    /// Transform listener to get transform between the two sensor frames.
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    /// Sensor name with which the output dialog is associated with.
    std::string sensorName_;
};

} // namespace multisensor_calibration
#endif // MULTISENSORCALIBRATION_UI_OBSERVATIONSVIEWDIALOG_H
