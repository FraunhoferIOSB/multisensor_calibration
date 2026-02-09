/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/ExtrinsicCameraCameraConfigWidget.h"

// ROS
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

// Qt
#include <QDebug>

// multisensor_calibration
#include "multisensor_calibration/common/utils.hpp"
#include "multisensor_calibration/io/Workspace.h"
#include "ui_ExtrinsicCameraCameraConfigWidget.h"
#include <multisensor_calibration/common/common.h>

namespace multisensor_calibration
{

//==================================================================================================
ExtrinsicCameraCameraConfigWidget::ExtrinsicCameraCameraConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui(new Ui::ExtrinsicCameraCameraConfigWidget),
  tfBuffer_(std::make_unique<tf2_ros::Buffer>(rclcpp::Clock::make_shared())),
  tfListener_(std::make_shared<tf2_ros::TransformListener>(*tfBuffer_))
{
    //--- set up UI
    ui->setupUi(this);

    for (uint i = 0; i < IMG_STATE_2_STR.size(); ++i)
    {
        ui->refImageStateComboBox->addItem(
          QString::fromStdString(
            IMG_STATE_2_STR.at(static_cast<EImageState>(i))));
        ui->srcImageStateComboBox->addItem(
          QString::fromStdString(
            IMG_STATE_2_STR.at(static_cast<EImageState>(i))));
    }
}

//==================================================================================================
ExtrinsicCameraCameraConfigWidget::~ExtrinsicCameraCameraConfigWidget()
{
    delete ui;
}

//==================================================================================================
void ExtrinsicCameraCameraConfigWidget::clearCalibrationOptions()
{
    sensorPairSettingsMap_.clear();

    ui->srcCamNameComboBox->clear();
    ui->srcImageTopicComboBox->clear();
    ui->srcInfoTopicComboBox->clear();

    ui->refCamNameComboBox->clear();
    ui->refImageTopicComboBox->clear();
    ui->refInfoTopicComboBox->clear();

    ui->baseFrameComboBox->clear();
}

//==================================================================================================
void ExtrinsicCameraCameraConfigWidget::setRobotWorkspaceFolderPath(const QString& iFolderPath)
{
    robotWorkspaceDir_.setPath(iFolderPath);

    //--- disconnect slots from combo box signals
    disconnect(ui->srcCamNameComboBox,
               static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
               this,
               &ExtrinsicCameraCameraConfigWidget::handleSelectedSensorsChanged);
    disconnect(ui->refCamNameComboBox,
               static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
               this,
               &ExtrinsicCameraCameraConfigWidget::handleSelectedSensorsChanged);

    clearCalibrationOptions();
    populateCalibrationOptions();
    setCalibrationOptionsFromSettings();

    //--- connects slots to combo box signals
    connect(ui->srcCamNameComboBox,
            static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this,
            &ExtrinsicCameraCameraConfigWidget::handleSelectedSensorsChanged);
    connect(ui->refCamNameComboBox,
            static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this,
            &ExtrinsicCameraCameraConfigWidget::handleSelectedSensorsChanged);
}

//==================================================================================================
void ExtrinsicCameraCameraConfigWidget::setSourceSensorName(const QString& iSensorName)
{
    ui->srcCamNameComboBox->setCurrentText(iSensorName);
    handleSelectedSensorsChanged();
}

//==================================================================================================
QString ExtrinsicCameraCameraConfigWidget::getSourceSensorName() const
{
    return ui->srcCamNameComboBox->currentText();
}

//==================================================================================================
void ExtrinsicCameraCameraConfigWidget::setReferenceSensorName(const QString& iSensorName)
{
    ui->refCamNameComboBox->setCurrentText(iSensorName);
    handleSelectedSensorsChanged();
}

//==================================================================================================
QString ExtrinsicCameraCameraConfigWidget::getReferenceSensorName() const
{
    return ui->refCamNameComboBox->currentText();
}

//==================================================================================================
std::unordered_map<std::string, bool>
ExtrinsicCameraCameraConfigWidget::getBoolTypedCalibrationOptions()
{
    return {
      {"is_stereo_camera", ui->calibrateIntrinsics->isChecked()},
      {"use_initial_guess", ui->initialGuessCheckBox->isChecked()},
      {"save_observations", ui->observationsCheckBox->isChecked()},
      {"use_exact_sync", (ui->syncPolicyComboBox->currentIndex() == 0)}};
}

//==================================================================================================
std::unordered_map<std::string, double>
ExtrinsicCameraCameraConfigWidget::getDoubleTypedCalibrationOptions()
{
    return {};
}

//==================================================================================================
std::unordered_map<std::string, int>
ExtrinsicCameraCameraConfigWidget::getIntTypedCalibrationOptions()
{
    return {
      {"sync_queue_size", ui->queueSizeSpinBox->value()}};
}

//==================================================================================================
std::unordered_map<std::string, std::string>
ExtrinsicCameraCameraConfigWidget::getStringTypedCalibrationOptions()
{
    std::string path = ament_index_cpp::get_package_share_directory("multisensor_calibration");

    return {
      {"src_camera_sensor_name", ui->srcCamNameComboBox->currentText().toStdString()},
      {"src_camera_image_topic", ui->srcImageTopicComboBox->currentText().toStdString()},
      {"src_camera_info_topic", ui->srcInfoTopicComboBox->currentText().toStdString()},
      {"src_image_state", ui->srcImageStateComboBox->currentText().toStdString()},

      {"ref_camera_sensor_name", ui->refCamNameComboBox->currentText().toStdString()},
      {"ref_camera_image_topic", ui->refImageTopicComboBox->currentText().toStdString()},
      {"ref_camera_info_topic", ui->refInfoTopicComboBox->currentText().toStdString()},
      {"ref_image_state", ui->refImageStateComboBox->currentText().toStdString()},

      {"base_frame_id", (ui->baseFrameGroupBox->isChecked())
                          ? ui->baseFrameComboBox->currentText().toStdString()
                          : ""},
      {"target_config_file", path + "/cfg/" +
                               ui->calibTargetFileLineEdit->text().toStdString()},
    };
}

//==================================================================================================
void ExtrinsicCameraCameraConfigWidget::addStrUniquelyToComboBox(
  QComboBox* pComboBox, QString strVal) const
{
    if (pComboBox->findText(strVal) == -1)
        pComboBox->addItem(strVal);
}

//==================================================================================================
void ExtrinsicCameraCameraConfigWidget::populateCalibrationOptions()
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
        if (ExtrinsicCameraCameraCalibWorkspace::isValid(fullDirectoryPath.toStdString()))
        {
            QString settingsFilePath = fullDirectoryPath + QDir::separator() +
                                       QString::fromStdString(SETTINGS_FILE_NAME);
            std::shared_ptr<QSettings> pSettings =
              std::make_shared<QSettings>(settingsFilePath, QSettings::Format::IniFormat);

            //--- camera options
            QString srcCamSensorName = pSettings->value("source_camera/sensor_name", "").toString();
            if (!srcCamSensorName.isEmpty())
                addStrUniquelyToComboBox(ui->srcCamNameComboBox, srcCamSensorName);

            QString tmpStrVal = pSettings->value("source_camera/image_topic", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->srcImageTopicComboBox, tmpStrVal);

            tmpStrVal = pSettings->value("source_camera/info_topic", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->srcInfoTopicComboBox, tmpStrVal);

            QString refCamSensorName = pSettings->value("reference_camera/sensor_name", "").toString();
            if (!refCamSensorName.isEmpty())
                addStrUniquelyToComboBox(ui->refCamNameComboBox, refCamSensorName);

            tmpStrVal = pSettings->value("reference_camera/image_topic", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->refImageTopicComboBox, tmpStrVal);

            tmpStrVal = pSettings->value("reference_camera/info_topic", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->refInfoTopicComboBox, tmpStrVal);

            //--- calibration options
            tmpStrVal = pSettings->value("calibration/base_frame_id", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->baseFrameComboBox, tmpStrVal);

            if (!refCamSensorName.isEmpty() && !srcCamSensorName.isEmpty())
            {
                QString key = srcCamSensorName + "_" + refCamSensorName;
                key.replace(" ", "_");
                sensorPairSettingsMap_[key.toStdString()] = pSettings;
            }
        }
    }
}

//==================================================================================================
void ExtrinsicCameraCameraConfigWidget::populateComboBoxesFromAvailableTfs()
{
    //--- populate combo boxes from available tf
    std::vector<std::string> frameIds;
    tfBuffer_->_getFrameStrings(frameIds);
    for (std::string id : frameIds)
        addStrUniquelyToComboBox(ui->baseFrameComboBox, QString::fromStdString(id));
}

//==================================================================================================

void ExtrinsicCameraCameraConfigWidget::populateComboBoxesFromAvailableTopics()
{
    //--- populate combo boxes from available ros topics

    for (auto topicInfo : topicList)
    {
        if (std::find(topicInfo.second.begin(), topicInfo.second.end(), "sensor_msgs/msg/Image") !=
            topicInfo.second.end())
        {
            QString topicName = QString::fromStdString(topicInfo.first);
            addStrUniquelyToComboBox(ui->srcImageTopicComboBox, topicName);
            addStrUniquelyToComboBox(ui->refImageTopicComboBox, topicName);
        }
        else if (std::find(topicInfo.second.begin(), topicInfo.second.end(), "sensor_msgs/msg/CameraInfo") !=
                 topicInfo.second.end())
        {
            QString topicName = QString::fromStdString(topicInfo.first);
            addStrUniquelyToComboBox(ui->srcInfoTopicComboBox, topicName);
            addStrUniquelyToComboBox(ui->refInfoTopicComboBox, topicName);
        }
    }
}

//==================================================================================================
void ExtrinsicCameraCameraConfigWidget::setCalibrationOptionsFromSettings()
{
    //--- construct key to access settings map
    QString srcSensorName = ui->srcCamNameComboBox->currentText();
    QString refSensorName = ui->refCamNameComboBox->currentText();
    if (srcSensorName.isEmpty() || refSensorName.isEmpty())
        return;

    QString key = srcSensorName + "_" + refSensorName;
    key.replace(" ", "_");
    if (sensorPairSettingsMap_.find(key.toStdString()) == sensorPairSettingsMap_.end())
        return;

    // Pointer to settings object
    std::shared_ptr<QSettings> pSettings = sensorPairSettingsMap_[key.toStdString()];

    //--- fill ui elements of calibration options

    ui->srcImageTopicComboBox->setCurrentText(pSettings->value("source_camera/image_topic").toString());
    ui->srcInfoTopicComboBox->setCurrentText(pSettings->value("source_camera/info_topic").toString());
    ui->srcImageStateComboBox->setCurrentIndex(pSettings->value("source_camera/image_state").toInt());

    ui->refImageTopicComboBox->setCurrentText(pSettings->value("reference_camera/image_topic").toString());
    ui->refInfoTopicComboBox->setCurrentText(pSettings->value("reference_camera/info_topic").toString());
    ui->refImageStateComboBox->setCurrentIndex(pSettings->value("reference_camera/image_state").toInt());

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

    ui->initialGuessCheckBox->setChecked(false);
    ui->initialGuessCheckBox->setEnabled(false);
    // ui->initialGuessCheckBox->setChecked(
    //   pSettings->value("calibration/use_initial_guess").toBool());

    ui->calibrateIntrinsics->setChecked(false);
    ui->calibrateIntrinsics->setEnabled(false);
    // ui->calibrateIntrinsics->setChecked(
    //   pSettings->value("intrinsic_calibration").toBool());

    ui->queueSizeSpinBox->setValue(pSettings->value("misc/sync_queue_size").toInt());
}

//==================================================================================================
void ExtrinsicCameraCameraConfigWidget::handleSelectedSensorsChanged()
{
    setCalibrationOptionsFromSettings();
}

//==================================================================================================

void ExtrinsicCameraCameraConfigWidget::setTopicList(std::map<std::string, std::vector<std::string>> topicList)
{
    this->topicList = topicList;
}

} // namespace multisensor_calibration
