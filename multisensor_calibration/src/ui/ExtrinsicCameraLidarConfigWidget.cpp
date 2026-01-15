/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/ExtrinsicCameraLidarConfigWidget.h"

// ROS
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

// Qt
#include <QDebug>

// multisensor_calibration
#include "multisensor_calibration/common/utils.hpp"
#include "multisensor_calibration/io/Workspace.h"
#include "ui_ExtrinsicCameraLidarConfigWidget.h"
#include <multisensor_calibration/common/common.h>

namespace multisensor_calibration
{

//==================================================================================================
ExtrinsicCameraLidarConfigWidget::ExtrinsicCameraLidarConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui(new Ui::ExtrinsicCameraLidarConfigWidget),
  tfBuffer_(std::make_unique<tf2_ros::Buffer>(rclcpp::Clock::make_shared())),
  tfListener_(std::make_shared<tf2_ros::TransformListener>(*tfBuffer_))
{
    //--- set up UI
    ui->setupUi(this);

    for (uint i = 0; i < IMG_STATE_2_STR.size(); ++i)
    {
        ui->imageStateComboBox->addItem(
          QString::fromStdString(
            IMG_STATE_2_STR.at(static_cast<EImageState>(i))));
    }
}

//==================================================================================================
ExtrinsicCameraLidarConfigWidget::~ExtrinsicCameraLidarConfigWidget()
{
    delete ui;
}

//==================================================================================================
void ExtrinsicCameraLidarConfigWidget::clearCalibrationOptions()
{
    sensorPairSettingsMap_.clear();

    ui->cameraNameComboBox->clear();
    ui->imageTopicComboBox->clear();
    ui->infoTopicComboBox->clear();
    ui->rightCameraNameComboBox->clear();
    ui->rightInfoComboBox->clear();
    ui->rectSuffixLineEdit->setText("_rect");
    ui->lidarNameComboBox->clear();
    ui->cloudTopicComboBox->clear();
    ui->baseFrameComboBox->clear();
}

//==================================================================================================
void ExtrinsicCameraLidarConfigWidget::setRobotWorkspaceFolderPath(const QString& iFolderPath)
{
    robotWorkspaceDir_.setPath(iFolderPath);

    //--- disconnect slots from combo box signals
    disconnect(ui->cameraNameComboBox,
               static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
               this,
               &ExtrinsicCameraLidarConfigWidget::handleSelectedSensorsChanged);
    disconnect(ui->lidarNameComboBox,
               static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
               this,
               &ExtrinsicCameraLidarConfigWidget::handleSelectedSensorsChanged);

    clearCalibrationOptions();
    populateCalibrationOptions();
    setCalibrationOptionsFromSettings();

    //--- connects slots to combo box signals
    connect(ui->cameraNameComboBox,
            static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this,
            &ExtrinsicCameraLidarConfigWidget::handleSelectedSensorsChanged);
    connect(ui->lidarNameComboBox,
            static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this,
            &ExtrinsicCameraLidarConfigWidget::handleSelectedSensorsChanged);
}

//==================================================================================================
void ExtrinsicCameraLidarConfigWidget::setSourceSensorName(const QString& iSensorName)
{
    ui->cameraNameComboBox->setCurrentText(iSensorName);
    handleSelectedSensorsChanged();
}

//==================================================================================================
QString ExtrinsicCameraLidarConfigWidget::getSourceSensorName() const
{
    return ui->cameraNameComboBox->currentText();
}

//==================================================================================================
void ExtrinsicCameraLidarConfigWidget::setReferenceSensorName(const QString& iSensorName)
{
    ui->lidarNameComboBox->setCurrentText(iSensorName);
    handleSelectedSensorsChanged();
}

//==================================================================================================
QString ExtrinsicCameraLidarConfigWidget::getReferenceSensorName() const
{
    return ui->lidarNameComboBox->currentText();
}

//==================================================================================================
std::unordered_map<std::string, bool>
ExtrinsicCameraLidarConfigWidget::getBoolTypedCalibrationOptions()
{
    return {
      {"is_stereo_camera", ui->isStereoGroupBox->isChecked()},
      {"use_initial_guess", ui->initialGuessCheckBox->isChecked()},
      {"save_observations", ui->observationsCheckBox->isChecked()},
      {"use_exact_sync", (ui->syncPolicyComboBox->currentIndex() == 0)}};
}

//==================================================================================================
std::unordered_map<std::string, double>
ExtrinsicCameraLidarConfigWidget::getDoubleTypedCalibrationOptions()
{
    return {};
}

//==================================================================================================
std::unordered_map<std::string, int>
ExtrinsicCameraLidarConfigWidget::getIntTypedCalibrationOptions()
{
    return {
      {"sync_queue_size", ui->queueSizeSpinBox->value()}};
}

//==================================================================================================
std::unordered_map<std::string, std::string>
ExtrinsicCameraLidarConfigWidget::getStringTypedCalibrationOptions()
{
    std::string path = ament_index_cpp::get_package_share_directory("multisensor_calibration");

    return {
      {"camera_sensor_name", ui->cameraNameComboBox->currentText().toStdString()},
      {"camera_image_topic", ui->imageTopicComboBox->currentText().toStdString()},
      {"camera_info_topic", ui->infoTopicComboBox->currentText().toStdString()},
      {"image_state", ui->imageStateComboBox->currentText().toStdString()},
      {"right_camera_sensor_name", ui->rightCameraNameComboBox->currentText().toStdString()},
      {"right_camera_info_topic", ui->rightInfoComboBox->currentText().toStdString()},
      {"rect_suffix", ui->rectSuffixLineEdit->text().toStdString()},
      {"lidar_sensor_name", ui->lidarNameComboBox->currentText().toStdString()},
      {"lidar_cloud_topic", ui->cloudTopicComboBox->currentText().toStdString()},
      {"base_frame_id", (ui->baseFrameGroupBox->isChecked())
                          ? ui->baseFrameComboBox->currentText().toStdString()
                          : ""},
      {"target_config_file", path + "/cfg/" +
                               ui->calibTargetFileLineEdit->text().toStdString()},
    };
}

//==================================================================================================
void ExtrinsicCameraLidarConfigWidget::addStrUniquelyToComboBox(
  QComboBox* pComboBox, QString strVal) const
{
    if (pComboBox->findText(strVal) == -1)
        pComboBox->addItem(strVal);
}

//==================================================================================================
void ExtrinsicCameraLidarConfigWidget::populateCalibrationOptions()
{
    populateComboBoxesFromAvailableTopics();

    populateComboBoxesFromAvailableTfs();

    // TODO: change when supporting other Calibration targets
    ui->calibTargetFileLineEdit->setText("TargetWithCirclesAndAruco.yaml");

    if (!robotWorkspaceDir_.exists())
        return;

    //--- populate from settings files
    //--- loop over subdirectories in root workspace dir and evaluate appropriate settings files
    for (QString entry : robotWorkspaceDir_.entryList(QDir::AllDirs | QDir::NoDotAndDotDot,
                                                      QDir::Name))
    {
        QString fullDirectoryPath = robotWorkspaceDir_.absolutePath() + QDir::separator() + entry;

        //--- if entry is an appropriate workspace folder, populate options from settings file
        if (ExtrinsicCameraLidarCalibWorkspace::isValid(fullDirectoryPath.toStdString()))
        {
            QString settingsFilePath = fullDirectoryPath + QDir::separator() +
                                       QString::fromStdString(SETTINGS_FILE_NAME);
            std::shared_ptr<QSettings> pSettings =
              std::make_shared<QSettings>(settingsFilePath, QSettings::Format::IniFormat);

            //--- camera options
            QString camSensorName = pSettings->value("camera/sensor_name", "").toString();
            if (!camSensorName.isEmpty())
                addStrUniquelyToComboBox(ui->cameraNameComboBox, camSensorName);

            QString tmpStrVal = pSettings->value("camera/image_topic", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->imageTopicComboBox, tmpStrVal);

            tmpStrVal = pSettings->value("camera/info_topic", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->infoTopicComboBox, tmpStrVal);

            tmpStrVal = pSettings->value("camera/right_sensor_name", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->rightCameraNameComboBox, tmpStrVal);

            tmpStrVal = pSettings->value("camera/right_info_topic", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->rightInfoComboBox, tmpStrVal);

            //--- lidar options
            QString lidarSensorName = pSettings->value("lidar/sensor_name", "").toString();
            if (!lidarSensorName.isEmpty())
                addStrUniquelyToComboBox(ui->lidarNameComboBox, lidarSensorName);

            tmpStrVal = pSettings->value("lidar/cloud_topic", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->cloudTopicComboBox, tmpStrVal);

            //--- calibration options
            tmpStrVal = pSettings->value("calibration/base_frame_id", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->baseFrameComboBox, tmpStrVal);

            if (!camSensorName.isEmpty() && !lidarSensorName.isEmpty())
            {
                QString key = camSensorName + "_" + lidarSensorName;
                key.replace(" ", "_");
                sensorPairSettingsMap_[key.toStdString()] = pSettings;
            }
        }
    }
}

//==================================================================================================
void ExtrinsicCameraLidarConfigWidget::populateComboBoxesFromAvailableTfs()
{
    //--- populate combo boxes from available tf
    std::vector<std::string> frameIds;
    tfBuffer_->_getFrameStrings(frameIds);
    for (std::string id : frameIds)
        addStrUniquelyToComboBox(ui->baseFrameComboBox, QString::fromStdString(id));
}

//==================================================================================================
void ExtrinsicCameraLidarConfigWidget::populateComboBoxesFromAvailableTopics()
{
    //--- populate combo boxes from available ros topics
    auto topicInfos = utils::getTopicList();
    for (auto topicInfo : topicInfos)
    {
        if (std::find(topicInfo.second.begin(), topicInfo.second.end(), "sensor_msgs/msg/Image") !=
            topicInfo.second.end())
        {
            QString topicName = QString::fromStdString(topicInfo.first);
            addStrUniquelyToComboBox(ui->imageTopicComboBox, topicName);
        }
        else if (std::find(topicInfo.second.begin(), topicInfo.second.end(), "sensor_msgs/msg/CameraInfo") !=
                 topicInfo.second.end())
        {
            QString topicName = QString::fromStdString(topicInfo.first);
            addStrUniquelyToComboBox(ui->infoTopicComboBox, topicName);
            addStrUniquelyToComboBox(ui->rightInfoComboBox, topicName);
        }
        else if (std::find(topicInfo.second.begin(), topicInfo.second.end(), "sensor_msgs/msg/PointCloud2") !=
                 topicInfo.second.end())
        {
            QString topicName = QString::fromStdString(topicInfo.first);
            addStrUniquelyToComboBox(ui->cloudTopicComboBox, topicName);
        }
    }
}

//==================================================================================================
void ExtrinsicCameraLidarConfigWidget::setCalibrationOptionsFromSettings()
{
    //--- construct key to access settings map
    QString camSensorName   = ui->cameraNameComboBox->currentText();
    QString lidarSensorName = ui->lidarNameComboBox->currentText();
    if (camSensorName.isEmpty() || lidarSensorName.isEmpty())
        return;

    QString key = camSensorName + "_" + lidarSensorName;
    key.replace(" ", "_");
    if (sensorPairSettingsMap_.find(key.toStdString()) == sensorPairSettingsMap_.end())
        return;

    // Pointer to settings object
    std::shared_ptr<QSettings> pSettings = sensorPairSettingsMap_[key.toStdString()];

    //--- fill ui elements of calibration options

    ui->imageTopicComboBox->setCurrentText(pSettings->value("camera/image_topic").toString());
    ui->infoTopicComboBox->setCurrentText(pSettings->value("camera/info_topic").toString());
    ui->imageStateComboBox->setCurrentIndex(pSettings->value("camera/image_state").toInt());
    ui->isStereoGroupBox->setChecked(pSettings->value("camera/is_stereo_camera").toBool());
    ui->rightCameraNameComboBox->setCurrentText(
      pSettings->value("camera/right_sensor_name").toString());
    ui->rightInfoComboBox->setCurrentText(
      pSettings->value("camera/right_info_topic").toString());
    ui->rectSuffixLineEdit->setText(pSettings->value("camera/rect_suffix", "_rect").toString());

    ui->cloudTopicComboBox->setCurrentText(pSettings->value("lidar/cloud_topic").toString());

    QString baseFrameIdVal = pSettings->value("calibration/base_frame_id").toString();
    if (!baseFrameIdVal.isEmpty())
    {
        ui->baseFrameGroupBox->setChecked(true);
        ui->baseFrameComboBox->setCurrentText(baseFrameIdVal);
    }
    else
    {
        ui->baseFrameGroupBox->setChecked(false);
    }
    // TODO: change when supporting other Calibration targets
    // ui->calibTargetFileLineEdit->setText(
    //   pSettings->value("calibration/target_config_file").toString());

    ui->observationsCheckBox->setChecked(
      pSettings->value("calibration/save_observations").toBool());
    ui->initialGuessCheckBox->setChecked(
      pSettings->value("calibration/use_initial_guess").toBool());

    ui->syncPolicyComboBox->setCurrentIndex((pSettings->value("misc/use_exact_sync").toBool())
                                              ? 0
                                              : 1);
    ui->queueSizeSpinBox->setValue(pSettings->value("misc/sync_queue_size").toInt());
}

//==================================================================================================
void ExtrinsicCameraLidarConfigWidget::handleSelectedSensorsChanged()
{
    setCalibrationOptionsFromSettings();
}

} // namespace multisensor_calibration
