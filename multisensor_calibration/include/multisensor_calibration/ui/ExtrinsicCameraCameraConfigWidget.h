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
#include <map>
#include <memory>
#include <unordered_map>

// ROS
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

// Qt
#include <QComboBox>
#include <QDir>
#include <QSettings>
#include <QWidget>

namespace multisensor_calibration
{

namespace Ui
{
class ExtrinsicCameraCameraConfigWidget;
}

/**
 * @ingroup ui
 * @brief Widget holding the configuration options for the extrinsic camera calibration.
 * This is part of the calibration configurator of the multi_sensor_calibration node.
 */
class ExtrinsicCameraCameraConfigWidget : public QWidget
{
    Q_OBJECT

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Construct a new Extrinsic Camera Config Widget.
     *
     * @param[in] parent Parent Widget
     */
    ExtrinsicCameraCameraConfigWidget(QWidget* parent = nullptr);

    /**
     * @brief Destroy the Extrinsic Camera Config Widget.
     */
    ~ExtrinsicCameraCameraConfigWidget();

    /**
     * @brief Clear contents of options.
     */
    void clearCalibrationOptions();

    /**
     * @brief Set path to robot workspace folder, triggering the population of the widget contents.
     */
    void setRobotWorkspaceFolderPath(const QString& iFolderPath);

    /**
     * @brief Set the source sensor name.
     */
    void setSourceSensorName(const QString& iSensorName);

    /**
     * @brief Get the source sensor name.
     */
    QString getSourceSensorName() const;

    /**
     * @brief Set the reference sensor name.
     */
    void setReferenceSensorName(const QString& iSensorName);

    /**
     * @brief Get the source sensor name.
     */
    QString getReferenceSensorName() const;

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

    void setTopicList(std::map<std::string, std::vector<std::string>> topicList);

  private:
    /**
     * @brief Utility function to add string value uniquely to ComboBox.
     */
    void addStrUniquelyToComboBox(QComboBox* pComboBox, QString strVal) const;

    /**
     * @brief Loop over all entries in robot workspace directory (if existent) and populate
     * options from settings files.
     */
    void populateCalibrationOptions();

    /**
     * @brief Populate combo boxes holding TF names from available ros TFs.
     */
    void populateComboBoxesFromAvailableTfs();

    /**
     * @brief Populate combo boxes holding topic names from available ros topics.
     */
    void populateComboBoxesFromAvailableTopics();

    /**
     * @brief Set the calibration options from loaded settings file.
     *
     */
    void setCalibrationOptionsFromSettings();

  private slots:

    /**
     * @brief Handle a change of the selected sensors which are to be calibrated.
     *
     */
    void handleSelectedSensorsChanged();

    //--- MEMBER DECLARATION ---//

  private:
    /// Pointer to UI
    Ui::ExtrinsicCameraCameraConfigWidget* ui;

    /// Directory of robot workspace, holding robot specific calibration workspaces.
    QDir robotWorkspaceDir_;

    /// Map holding QSettings mapped to sensor pair
    std::map<std::string, std::shared_ptr<QSettings>> sensorPairSettingsMap_;

    /// Transform buffer and listener to get transform between the two sensor frames
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};

    std::map<std::string, std::vector<std::string>> topicList;
};

} // namespace multisensor_calibration
