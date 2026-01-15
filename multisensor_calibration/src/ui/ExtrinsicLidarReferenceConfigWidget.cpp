/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/ExtrinsicLidarReferenceConfigWidget.h"

// ROS
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

// Qt
#include <QDebug>

// multisensor_calibration
#include "multisensor_calibration/common/utils.hpp"
#include "multisensor_calibration/io/Workspace.h"
#include "ui_ExtrinsicLidarReferenceConfigWidget.h"

namespace multisensor_calibration
{

//==================================================================================================
ExtrinsicLidarReferenceConfigWidget::ExtrinsicLidarReferenceConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui(new Ui::ExtrinsicLidarReferenceConfigWidget),
  tfBuffer_(std::make_unique<tf2_ros::Buffer>(rclcpp::Clock::make_shared())),
  tfListener_(std::make_shared<tf2_ros::TransformListener>(*tfBuffer_))
{
    //--- set up UI
    ui->setupUi(this);
}

//==================================================================================================
ExtrinsicLidarReferenceConfigWidget::~ExtrinsicLidarReferenceConfigWidget()
{
    delete ui;
}

//==================================================================================================
void ExtrinsicLidarReferenceConfigWidget::clearCalibrationOptions()
{
    sensorPairSettingsMap_.clear();

    ui->srcNameComboBox->clear();
    ui->srcCloudTopicComboBox->clear();
    ui->refNameComboBox->clear();
    ui->refFrameIdComboBox->clear();
    ui->baseFrameComboBox->clear();
}

//==================================================================================================
void ExtrinsicLidarReferenceConfigWidget::setRobotWorkspaceFolderPath(const QString& iFolderPath)
{
    robotWorkspaceDir_.setPath(iFolderPath);

    //--- disconnect slots from combo box signals
    disconnect(ui->srcNameComboBox,
               static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
               this,
               &ExtrinsicLidarReferenceConfigWidget::handleSelectedSensorsChanged);
    disconnect(ui->refNameComboBox,
               static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
               this,
               &ExtrinsicLidarReferenceConfigWidget::handleSelectedSensorsChanged);

    clearCalibrationOptions();
    populateCalibrationOptions();
    setCalibrationOptionsFromSettings();

    addStrUniquelyToComboBox(ui->refNameComboBox, "reference");

    //--- connects slots to combo box signals
    connect(ui->srcNameComboBox,
            static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this,
            &ExtrinsicLidarReferenceConfigWidget::handleSelectedSensorsChanged);
    connect(ui->refNameComboBox,
            static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this,
            &ExtrinsicLidarReferenceConfigWidget::handleSelectedSensorsChanged);
}

//==================================================================================================
void ExtrinsicLidarReferenceConfigWidget::setSourceSensorName(const QString& iSensorName)
{
    ui->srcNameComboBox->setCurrentText(iSensorName);
    handleSelectedSensorsChanged();
}

//==================================================================================================
QString ExtrinsicLidarReferenceConfigWidget::getSourceSensorName() const
{
    return ui->srcNameComboBox->currentText();
}

//==================================================================================================
void ExtrinsicLidarReferenceConfigWidget::setReferenceName(const QString& iName)
{
    ui->refNameComboBox->setCurrentText(iName);
    handleSelectedSensorsChanged();
}

//==================================================================================================
QString ExtrinsicLidarReferenceConfigWidget::getReferenceName() const
{
    return ui->refNameComboBox->currentText();
}

//==================================================================================================
std::unordered_map<std::string, bool>
ExtrinsicLidarReferenceConfigWidget::getBoolTypedCalibrationOptions()
{
    return {
      {"save_observations", ui->observationsCheckBox->isChecked()}};
}

//==================================================================================================
std::unordered_map<std::string, double>
ExtrinsicLidarReferenceConfigWidget::getDoubleTypedCalibrationOptions()
{
    return {};
}

//==================================================================================================
std::unordered_map<std::string, int>
ExtrinsicLidarReferenceConfigWidget::getIntTypedCalibrationOptions()
{
    return {};
}

//==================================================================================================
std::unordered_map<std::string, std::string>
ExtrinsicLidarReferenceConfigWidget::getStringTypedCalibrationOptions()
{
    std::string path = ament_index_cpp::get_package_share_directory("multisensor_calibration");
    return {
      {"src_lidar_sensor_name", ui->srcNameComboBox->currentText().toStdString()},
      {"src_lidar_cloud_topic", ui->srcCloudTopicComboBox->currentText().toStdString()},
      {"reference_name", ui->refNameComboBox->currentText().toStdString()},
      {"reference_frame_id", ui->refFrameIdComboBox->currentText().toStdString()},
      {"base_frame_id", (ui->baseFrameGroupBox->isChecked())
                          ? ui->baseFrameComboBox->currentText().toStdString()
                          : ""},
      {"target_config_file", path + "/cfg/" +
                               ui->calibTargetFileLineEdit->text().toStdString()},
    };
}

//==================================================================================================
void ExtrinsicLidarReferenceConfigWidget::addStrUniquelyToComboBox(
  QComboBox* pComboBox, QString strVal) const
{
    if (pComboBox->findText(strVal) == -1)
        pComboBox->addItem(strVal);
}

//==================================================================================================
void ExtrinsicLidarReferenceConfigWidget::populateCalibrationOptions()
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
        if (ExtrinsicLidarReferenceCalibWorkspace::isValid(fullDirectoryPath.toStdString()))
        {
            QString settingsFilePath = fullDirectoryPath + QDir::separator() +
                                       QString::fromStdString(SETTINGS_FILE_NAME);
            std::shared_ptr<QSettings> pSettings =
              std::make_shared<QSettings>(settingsFilePath, QSettings::Format::IniFormat);

            //--- source lidar options
            QString srcLidarName = pSettings->value("source_lidar/sensor_name", "").toString();
            if (!srcLidarName.isEmpty())
                addStrUniquelyToComboBox(ui->srcNameComboBox, srcLidarName);

            QString tmpStrVal = pSettings->value("source_lidar/cloud_topic", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->srcCloudTopicComboBox, tmpStrVal);

            //--- reference options
            QString refName = pSettings->value("reference/name", "").toString();
            if (!refName.isEmpty())
                addStrUniquelyToComboBox(ui->refNameComboBox, refName);

            QString refFrameId = pSettings->value("reference/frame_id", "").toString();
            if (!refFrameId.isEmpty())
                addStrUniquelyToComboBox(ui->refFrameIdComboBox, refFrameId);

            //--- calibration options
            tmpStrVal = pSettings->value("calibration/base_frame_id", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->baseFrameComboBox, tmpStrVal);

            if (!srcLidarName.isEmpty() && !refName.isEmpty())
            {
                QString key = srcLidarName + "_" + refName;
                key.replace(" ", "_");
                sensorPairSettingsMap_[key.toStdString()] = pSettings;
            }
        }
    }
}

//==================================================================================================
void ExtrinsicLidarReferenceConfigWidget::populateComboBoxesFromAvailableTfs()
{
    //--- populate combo boxes from available tf
    std::vector<std::string> frameIds;
    tfBuffer_->_getFrameStrings(frameIds);
    for (std::string id : frameIds)
    {
        addStrUniquelyToComboBox(ui->baseFrameComboBox, QString::fromStdString(id));
        addStrUniquelyToComboBox(ui->refFrameIdComboBox, QString::fromStdString(id));
    }
}

//==================================================================================================
void ExtrinsicLidarReferenceConfigWidget::populateComboBoxesFromAvailableTopics()
{

    //--- populate combo boxes from available ros topics
    auto topicInfos = utils::getTopicList();
    for (auto topicInfo : topicInfos)
    {
        if (std::find(topicInfo.second.begin(), topicInfo.second.end(), "sensor_msgs/msg/PointCloud2") !=
            topicInfo.second.end())
        {
            QString topicName = QString::fromStdString(topicInfo.first);
            addStrUniquelyToComboBox(ui->srcCloudTopicComboBox, topicName);
        }
    }
}

//==================================================================================================
void ExtrinsicLidarReferenceConfigWidget::setCalibrationOptionsFromSettings()
{
    //--- construct key to access settings map
    QString srcLidarName = ui->srcNameComboBox->currentText();
    QString refName      = ui->refNameComboBox->currentText();
    if (srcLidarName.isEmpty() || refName.isEmpty())
        return;

    QString key = srcLidarName + "_" + refName;
    key.replace(" ", "_");
    if (sensorPairSettingsMap_.find(key.toStdString()) == sensorPairSettingsMap_.end())
        return;

    // Pointer to settings object
    std::shared_ptr<QSettings> pSettings = sensorPairSettingsMap_[key.toStdString()];

    //--- fill ui elements of calibration options

    ui->srcCloudTopicComboBox->setCurrentText(
      pSettings->value("source_lidar/cloud_topic").toString());

    ui->refFrameIdComboBox->setCurrentText(
      pSettings->value("reference/frame_id").toString());

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
}

//==================================================================================================
void ExtrinsicLidarReferenceConfigWidget::handleSelectedSensorsChanged()
{
    setCalibrationOptionsFromSettings();
}

} // namespace multisensor_calibration
