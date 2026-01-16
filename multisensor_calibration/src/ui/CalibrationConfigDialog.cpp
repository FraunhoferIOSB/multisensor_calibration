/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/CalibrationConfigDialog.h"

// Qt
#include <QDebug>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QMessageBox>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/common/utils.hpp"
#include "multisensor_calibration/io/Workspace.h"
#include "ui_CalibrationConfigDialog.h"

QString ROOT_DIR_LABEL_TXT = "<html><head/><body><p>"
                             "<span style=\" font-size:10pt; font-style:italic;\">"
                             "%1"
                             "</span>"
                             "</p></body></html>";

namespace multisensor_calibration
{

//==================================================================================================
CalibrationConfigDialog::CalibrationConfigDialog(QWidget* parent) :
  QDialog(parent),
  ui(new Ui::CalibrationConfigDialog),
  pInstallWsTemplateDialog(new InstallWorkspaceDialog(this)),
  pCameraLidarConfigWidget(new ExtrinsicCameraLidarConfigWidget(this)),
  pCameraCameraConfigWidget(new ExtrinsicCameraCameraConfigWidget(this)),
  pCameraReferenceConfigWidget(new ExtrinsicCameraReferenceConfigWidget(this)),
  pLidarLidarConfigWidget(new ExtrinsicLidarLidarConfigWidget(this)),
  pLidarReferenceConfigWidget(new ExtrinsicLidarReferenceConfigWidget(this)),
  pConfiguratorSettings_(nullptr),
  pRobotSettings_(nullptr)
{
    //--- set up UI
    ui->setupUi(this);
    ui->calibrationGroupBox->layout()->addWidget(pCameraLidarConfigWidget);
    pCameraLidarConfigWidget->setParent(ui->calibrationGroupBox);
    ui->calibrationGroupBox->layout()->addWidget(pCameraCameraConfigWidget);
    pCameraCameraConfigWidget->setParent(ui->calibrationGroupBox);
    ui->calibrationGroupBox->layout()->addWidget(pCameraReferenceConfigWidget);
    pCameraReferenceConfigWidget->setParent(ui->calibrationGroupBox);
    ui->calibrationGroupBox->layout()->addWidget(pLidarLidarConfigWidget);
    pLidarLidarConfigWidget->setParent(ui->calibrationGroupBox);
    ui->calibrationGroupBox->layout()->addWidget(pLidarReferenceConfigWidget);
    pLidarReferenceConfigWidget->setParent(ui->calibrationGroupBox);
    ui->titleIconLabel->setPixmap(QIcon(":/icons/icons8-sensor-100.png").pixmap(50, 50));
    this->setWindowIcon(QIcon(":/icons/icons8-sensor-100_filled.png"));

    ui->rootDirPushButton->setIcon(QIcon(":/icons/icons8-opened-folder-26.png"));
    ui->installWsPushButton->setIcon(QIcon(":/icons/icons8-add-list-30.png"));
    ui->selectUrdfPushButton->setIcon(QIcon(":/icons/icons8-add-file-26.png"));

    ui->buttonBox->button(QDialogButtonBox::Ok)->setText("Run");

    //--- fill calibration type combo box
    for (uint i = 0; i < CALIB_TYPE_2_STR.size(); ++i)
    {
        ui->calibTypeComboBox->addItem(
          QString::fromStdString(CALIB_TYPE_2_STR.at(static_cast<ECalibrationType>(i))));
    }

    //--- initialize settings
    loadConfiguratorSettings();

    //--- connect signal and slots
    connect(ui->rootDirPushButton, &QPushButton::clicked, this,
            &CalibrationConfigDialog::handleRootDirPushButtonClicked);
    connect(ui->installWsPushButton, &QPushButton::clicked, this,
            &CalibrationConfigDialog::handleInstallWsPushButtonClicked);
    connect(pInstallWsTemplateDialog, &QDialog::accepted,
            this, &CalibrationConfigDialog::handleInstallWsDialogAccepted);
    connect(ui->selectUrdfPushButton, &QPushButton::clicked, this,
            &CalibrationConfigDialog::handleSelectUrdfPushButtonClicked);
    connect(ui->wsFolderComboBox,
            static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this,
            &CalibrationConfigDialog::handleWsFolderChanged);
    connect(ui->calibTypeComboBox,
            static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this,
            &CalibrationConfigDialog::handleCalibrationTypeChanged);
    connect(ui->buttonBox, &QDialogButtonBox::clicked,
            this, &CalibrationConfigDialog::handleButtonBoxClicked);
}

//==================================================================================================
CalibrationConfigDialog::~CalibrationConfigDialog()
{
    delete ui;
}

//==================================================================================================
std::unordered_map<std::string, bool> CalibrationConfigDialog::getBoolTypedCalibrationOptions()
{
    std::unordered_map<std::string, bool> calibOptions;

    //--- get Options from widget
    ECalibrationType currentCalibType =
      static_cast<ECalibrationType>(ui->calibTypeComboBox->currentIndex());

    if (currentCalibType == EXTRINSIC_CAMERA_LIDAR_CALIBRATION)
    {
        calibOptions = pCameraLidarConfigWidget->getBoolTypedCalibrationOptions();
    }
    else if (currentCalibType == EXTRINSIC_CAMERA_REFERENCE_CALIBRATION)
    {
        calibOptions = pCameraReferenceConfigWidget->getBoolTypedCalibrationOptions();
    }
    else if (currentCalibType == EXTRINSIC_LIDAR_LIDAR_CALIBRATION)
    {
        calibOptions = pLidarLidarConfigWidget->getBoolTypedCalibrationOptions();
    }
    else if (currentCalibType == EXTRINSIC_LIDAR_REFERENCE_CALIBRATION)
    {
        calibOptions = pLidarReferenceConfigWidget->getBoolTypedCalibrationOptions();
    }
    else if (currentCalibType == STEREO_CAMERA_CALIBRATION)
    {
        calibOptions = pCameraCameraConfigWidget->getBoolTypedCalibrationOptions();
    }

    return calibOptions;
}

//==================================================================================================
std::unordered_map<std::string, double> CalibrationConfigDialog::getDoubleTypedCalibrationOptions()
{
    std::unordered_map<std::string, double> calibOptions;

    //--- get Options from widget
    ECalibrationType currentCalibType =
      static_cast<ECalibrationType>(ui->calibTypeComboBox->currentIndex());

    if (currentCalibType == EXTRINSIC_CAMERA_LIDAR_CALIBRATION)
    {
        calibOptions = pCameraLidarConfigWidget->getDoubleTypedCalibrationOptions();
    }
    else if (currentCalibType == EXTRINSIC_CAMERA_REFERENCE_CALIBRATION)
    {
        calibOptions = pCameraReferenceConfigWidget->getDoubleTypedCalibrationOptions();
    }
    else if (currentCalibType == EXTRINSIC_LIDAR_LIDAR_CALIBRATION)
    {
        calibOptions = pLidarLidarConfigWidget->getDoubleTypedCalibrationOptions();
    }
    else if (currentCalibType == EXTRINSIC_LIDAR_REFERENCE_CALIBRATION)
    {
        calibOptions = pLidarReferenceConfigWidget->getDoubleTypedCalibrationOptions();
    }
    else if (currentCalibType == STEREO_CAMERA_CALIBRATION)
    {
        calibOptions = pCameraCameraConfigWidget->getDoubleTypedCalibrationOptions();
    }

    return calibOptions;
}

//==================================================================================================
std::unordered_map<std::string, int> CalibrationConfigDialog::getIntTypedCalibrationOptions()
{
    std::unordered_map<std::string, int> calibOptions;

    //--- get Options from widget
    ECalibrationType currentCalibType =
      static_cast<ECalibrationType>(ui->calibTypeComboBox->currentIndex());

    if (currentCalibType == EXTRINSIC_CAMERA_LIDAR_CALIBRATION)
    {
        calibOptions = pCameraLidarConfigWidget->getIntTypedCalibrationOptions();
    }
    else if (currentCalibType == EXTRINSIC_CAMERA_REFERENCE_CALIBRATION)
    {
        calibOptions = pCameraReferenceConfigWidget->getIntTypedCalibrationOptions();
    }
    else if (currentCalibType == EXTRINSIC_LIDAR_LIDAR_CALIBRATION)
    {
        calibOptions = pLidarLidarConfigWidget->getIntTypedCalibrationOptions();
    }
    else if (currentCalibType == EXTRINSIC_LIDAR_REFERENCE_CALIBRATION)
    {
        calibOptions = pLidarReferenceConfigWidget->getIntTypedCalibrationOptions();
    }
    else if (currentCalibType == STEREO_CAMERA_CALIBRATION)
    {
        calibOptions = pCameraCameraConfigWidget->getIntTypedCalibrationOptions();
    }

    return calibOptions;
}

//==================================================================================================
std::unordered_map<std::string, std::string>
CalibrationConfigDialog::getStringTypedCalibrationOptions()
{
    std::unordered_map<std::string, std::string> calibOptions;

    //--- get Options from widget
    ECalibrationType currentCalibType =
      static_cast<ECalibrationType>(ui->calibTypeComboBox->currentIndex());

    if (currentCalibType == EXTRINSIC_CAMERA_LIDAR_CALIBRATION)
    {
        calibOptions = pCameraLidarConfigWidget->getStringTypedCalibrationOptions();
    }
    else if (currentCalibType == EXTRINSIC_CAMERA_REFERENCE_CALIBRATION)
    {
        calibOptions = pCameraReferenceConfigWidget->getStringTypedCalibrationOptions();
    }
    else if (currentCalibType == EXTRINSIC_LIDAR_LIDAR_CALIBRATION)
    {
        calibOptions = pLidarLidarConfigWidget->getStringTypedCalibrationOptions();
    }
    else if (currentCalibType == EXTRINSIC_LIDAR_REFERENCE_CALIBRATION)
    {
        calibOptions = pLidarReferenceConfigWidget->getStringTypedCalibrationOptions();
    }
    else if (currentCalibType == STEREO_CAMERA_CALIBRATION)
    {
        calibOptions = pCameraCameraConfigWidget->getStringTypedCalibrationOptions();
    }

    //--- add robot ws path
    calibOptions.insert({"robot_ws_path",
                         QString(calibrationRootDir_.absolutePath() +
                                 QDir::separator() +
                                 ui->wsFolderComboBox->currentText())
                           .toStdString()});

    return calibOptions;
}

//==================================================================================================
void CalibrationConfigDialog::accept()
{
    // Workspace folder name
    QString wsFolderName = ui->wsFolderComboBox->currentText();

    //--- if workspace folder name is empty, show warning and return
    if (wsFolderName.isEmpty())
    {
        QMessageBox::critical(this, this->windowTitle(),
                              "Please select or enter a folder name for the robot workspace.");
        return;
    }

    // Robot workspace
    RobotWorkspace robotWs = RobotWorkspace((calibrationRootDir_.absolutePath() +
                                             QDir::separator() +
                                             wsFolderName)
                                              .toStdString(),
                                            rclcpp::get_logger("RobotWorkspace"));

    //--- check if robot workspace is valid, i.e. already available.
    if (!robotWs.isValid())
    {
        //--- ask if sure to create new robot workspace
        int ret =
          QMessageBox::question(this, this->windowTitle(),
                                QString("You are about to create a new robot workspace ('%1'). "
                                        "Are you sure to continue?")
                                  .arg(wsFolderName));
        if (ret == QMessageBox::No)
            return;

        //--- if yes, create workspace and print warning if something went wrong
        if (!robotWs.load(true, true))
        {
            QMessageBox::critical(this, this->windowTitle(),
                                  QString("Something went wrong during "
                                          "the initialization of the robot workspace '%1'")
                                    .arg(wsFolderName));
            return;
        }
    }
    else
    {
        if (!robotWs.load(false, false))
        {
            QMessageBox::critical(this, this->windowTitle(),
                                  QString("Something went wrong during "
                                          "the loading of the robot workspace '%1'")
                                    .arg(wsFolderName));
            return;
        }
    }

    //--- if all checks passed, save settings and accept dialog.
    robotWs.settingsPtr()->setValue("robot/name", ui->robotNameLineEdit->text());
    robotWs.settingsPtr()->setValue("robot/urdf_model_path", ui->urdfLineEdit->text());
    robotWs.settingsPtr()->sync();

    this->saveSettings();

    QDialog::accept();
}

//==================================================================================================
ECalibrationType CalibrationConfigDialog::selectedCalibrationType() const
{
    return static_cast<ECalibrationType>(ui->calibTypeComboBox->currentIndex());
}

//==================================================================================================
void CalibrationConfigDialog::loadConfiguratorSettings()
{
    pConfiguratorSettings_.reset(new QSettings("multisensor_calibration",
                                               "multi_sensor_calibration"));

    resetCalibrationOptions();
}

//==================================================================================================
void CalibrationConfigDialog::loadRobotSettings()
{
    //--- get currently selected workspace folder and initialize settings file
    QString selectedFolder   = ui->wsFolderComboBox->currentText();
    QString settingsFilePath = calibrationRootDir_.absolutePath() +
                               QDir::separator() +
                               selectedFolder +
                               QDir::separator() +
                               QString::fromStdString(SETTINGS_FILE_NAME);
    pRobotSettings_.reset(new QSettings(settingsFilePath, QSettings::Format::IniFormat));

    //--- enter information into ui fields
    ui->robotNameLineEdit->setText(pRobotSettings_->value("robot/name", "")
                                     .toString());
    ui->urdfLineEdit->setText(pRobotSettings_->value("robot/urdf_model_path", "")
                                .toString());
}

//==================================================================================================
void CalibrationConfigDialog::populateWsFolderComboBox()
{
    ui->wsFolderComboBox->clear();

    for (QString entry : calibrationRootDir_.entryList(QDir::AllDirs | QDir::NoDotAndDotDot,
                                                       QDir::Name))
    {
        QString fullDirectoryPath = calibrationRootDir_.absolutePath() + QDir::separator() + entry;

        if (RobotWorkspace::isValid(fullDirectoryPath.toStdString()))
            ui->wsFolderComboBox->addItem(entry);
    }
}

//==================================================================================================
void CalibrationConfigDialog::resetCalibrationOptions()
{
    if (pConfiguratorSettings_ == nullptr)
        pConfiguratorSettings_.reset(new QSettings("multisensor_calibration",
                                                   "multi_sensor_calibration"));

    //--- root dir path
    QString rootDirPath = pConfiguratorSettings_->value("calibration_root_dir",
                                                        QDir::homePath() + QDir::separator() +
                                                          "multisensor_calibration")
                            .toString();
    calibrationRootDir_.setPath(rootDirPath);
    updateRootDirLabelContents();
    populateWsFolderComboBox();

    //--- robot settings
    QString lastRobotSelected = pConfiguratorSettings_->value("last_robot_ws", "")
                                  .toString();
    if (!lastRobotSelected.isEmpty())
    {
        int idx = ui->wsFolderComboBox->findText(lastRobotSelected);
        if (idx >= 0)
            ui->wsFolderComboBox->setCurrentIndex(idx);
    }
    handleWsFolderChanged();

    //--- calib type settings
    uint lastCalibTypeSelected = pConfiguratorSettings_->value("last_calibration_type", 0)
                                   .toUInt();
    ui->calibTypeComboBox->setCurrentIndex(lastCalibTypeSelected);
    handleCalibrationTypeChanged();

    //--- calib settings
    QString lastSrcSensorSelected = pConfiguratorSettings_->value("last_src_sensor_name", "")
                                      .toString();
    QString lastRefSensorSelected = pConfiguratorSettings_->value("last_ref_sensor_name", "")
                                      .toString();
    if (lastCalibTypeSelected == EXTRINSIC_CAMERA_LIDAR_CALIBRATION)
    {
        if (!lastSrcSensorSelected.isEmpty())
            pCameraLidarConfigWidget->setSourceSensorName(lastSrcSensorSelected);
        if (!lastRefSensorSelected.isEmpty())
            pCameraLidarConfigWidget->setReferenceSensorName(lastRefSensorSelected);
    }
    else if (lastCalibTypeSelected == EXTRINSIC_CAMERA_REFERENCE_CALIBRATION)
    {
        if (!lastSrcSensorSelected.isEmpty())
            pCameraReferenceConfigWidget->setSourceSensorName(lastSrcSensorSelected);
        if (!lastRefSensorSelected.isEmpty())
            pCameraReferenceConfigWidget->setReferenceName(lastRefSensorSelected);
    }
    else if (lastCalibTypeSelected == EXTRINSIC_LIDAR_LIDAR_CALIBRATION)
    {
        if (!lastSrcSensorSelected.isEmpty())
            pLidarLidarConfigWidget->setSourceSensorName(lastSrcSensorSelected);
        if (!lastRefSensorSelected.isEmpty())
            pLidarLidarConfigWidget->setReferenceSensorName(lastRefSensorSelected);
    }
    else if (lastCalibTypeSelected == EXTRINSIC_LIDAR_REFERENCE_CALIBRATION)
    {
        if (!lastSrcSensorSelected.isEmpty())
            pLidarReferenceConfigWidget->setSourceSensorName(lastSrcSensorSelected);
        if (!lastRefSensorSelected.isEmpty())
            pLidarReferenceConfigWidget->setReferenceName(lastRefSensorSelected);
    }
    else if (lastCalibTypeSelected == STEREO_CAMERA_CALIBRATION)
    {
        if (!lastSrcSensorSelected.isEmpty())
            pCameraCameraConfigWidget->setSourceSensorName(lastSrcSensorSelected);
        if (!lastRefSensorSelected.isEmpty())
            pCameraCameraConfigWidget->setReferenceSensorName(lastRefSensorSelected);
    }
}

//==================================================================================================
void CalibrationConfigDialog::resizeEvent(QResizeEvent* event)
{
    QDialog::resizeEvent(event);

    updateRootDirLabelContents();
}

//==================================================================================================
void CalibrationConfigDialog::saveSettings()
{
    pConfiguratorSettings_->setValue("calibration_root_dir", calibrationRootDir_.absolutePath());
    pConfiguratorSettings_->setValue("last_robot_ws", ui->wsFolderComboBox->currentText());

    ECalibrationType currentCalibType =
      static_cast<ECalibrationType>(ui->calibTypeComboBox->currentIndex());
    pConfiguratorSettings_->setValue("last_calibration_type", static_cast<uint>(currentCalibType));

    if (currentCalibType == EXTRINSIC_CAMERA_LIDAR_CALIBRATION)
    {
        pConfiguratorSettings_->setValue("last_src_sensor_name",
                                         pCameraLidarConfigWidget->getSourceSensorName());
        pConfiguratorSettings_->setValue("last_ref_sensor_name",
                                         pCameraLidarConfigWidget->getReferenceSensorName());
    }
    else if (currentCalibType == EXTRINSIC_CAMERA_REFERENCE_CALIBRATION)
    {
        pConfiguratorSettings_->setValue("last_src_sensor_name",
                                         pCameraReferenceConfigWidget->getSourceSensorName());
        pConfiguratorSettings_->setValue("last_ref_sensor_name",
                                         pCameraReferenceConfigWidget->getReferenceName());
    }
    else if (currentCalibType == EXTRINSIC_LIDAR_LIDAR_CALIBRATION)
    {
        pConfiguratorSettings_->setValue("last_src_sensor_name",
                                         pLidarLidarConfigWidget->getSourceSensorName());
        pConfiguratorSettings_->setValue("last_ref_sensor_name",
                                         pLidarLidarConfigWidget->getReferenceSensorName());
    }
    else if (currentCalibType == EXTRINSIC_LIDAR_REFERENCE_CALIBRATION)
    {
        pConfiguratorSettings_->setValue("last_src_sensor_name",
                                         pLidarReferenceConfigWidget->getSourceSensorName());
        pConfiguratorSettings_->setValue("last_ref_sensor_name",
                                         pLidarReferenceConfigWidget->getReferenceName());
    }
    else if (currentCalibType == STEREO_CAMERA_CALIBRATION)
    {
        pConfiguratorSettings_->setValue("last_src_sensor_name",
                                         pCameraCameraConfigWidget->getSourceSensorName());
        pConfiguratorSettings_->setValue("last_ref_sensor_name",
                                         pCameraCameraConfigWidget->getReferenceSensorName());
    }
}

//==================================================================================================
void CalibrationConfigDialog::showEvent(QShowEvent* event)
{
    QDialog::showEvent(event);

    updateRootDirLabelContents();

    this->adjustSize();
    this->setMinimumHeight(this->size().height());
}

//==================================================================================================
void CalibrationConfigDialog::updateRootDirLabelContents()
{
    int textLength = ui->rootDirLabel->fontMetrics()
                       .boundingRect(calibrationRootDir_.absolutePath())
                       .width();
    if (textLength < ui->rootDirLabel->size().width())
    {
        ui->rootDirLabel->setText(ROOT_DIR_LABEL_TXT.arg(calibrationRootDir_.absolutePath()));
    }
    else
    {
        QString placeholder = QString(QDir::separator()) + "{...}";

        QStringList pathElements = calibrationRootDir_.absolutePath().split(QDir::separator());
        QString labelText        = placeholder + QDir::separator() + pathElements.back();
        pathElements.pop_back();

        textLength = ui->rootDirLabel->fontMetrics().boundingRect(labelText).width();
        while ((textLength < ui->rootDirLabel->size().width()) &&
               !pathElements.isEmpty())
        {
            if (pathElements.front().isEmpty())
            {
                pathElements.pop_front();
                continue;
            }

            ui->rootDirLabel->setText(ROOT_DIR_LABEL_TXT.arg(labelText));

            labelText = labelText.replace(placeholder,
                                          QDir::separator() + pathElements.front() + placeholder);
            pathElements.pop_front();

            textLength = ui->rootDirLabel->fontMetrics().boundingRect(labelText).width();
        }
    }

    ui->rootDirLabel->setToolTip("Calibration Root Directory: " +
                                 calibrationRootDir_.absolutePath());
}

//==================================================================================================
void CalibrationConfigDialog::handleButtonBoxClicked(QAbstractButton* button)
{
    if (ui->buttonBox->buttonRole(button) == QDialogButtonBox::ResetRole)
    {
        this->resetCalibrationOptions();
    }
}

//==================================================================================================
void CalibrationConfigDialog::handleInstallWsPushButtonClicked()
{
    pInstallWsTemplateDialog->setCalibrationRootDirPath(calibrationRootDir_.absolutePath());
    pInstallWsTemplateDialog->show();
}

//==================================================================================================
void CalibrationConfigDialog::handleInstallWsDialogAccepted()
{
    populateWsFolderComboBox();
}

//==================================================================================================
void CalibrationConfigDialog::handleCalibrationTypeChanged()
{
    ECalibrationType currentCalibType =
      static_cast<ECalibrationType>(ui->calibTypeComboBox->currentIndex());

    switch (currentCalibType)
    {
    default:
    case EXTRINSIC_CAMERA_LIDAR_CALIBRATION:
    {
        pCameraReferenceConfigWidget->setVisible(false);
        pLidarLidarConfigWidget->setVisible(false);
        pLidarReferenceConfigWidget->setVisible(false);
        pCameraLidarConfigWidget->setVisible(true);
        pCameraCameraConfigWidget->setVisible(false);
    }
    break;

    case EXTRINSIC_CAMERA_REFERENCE_CALIBRATION:
    {
        pCameraLidarConfigWidget->setVisible(false);
        pLidarLidarConfigWidget->setVisible(false);
        pLidarReferenceConfigWidget->setVisible(false);
        pCameraReferenceConfigWidget->setVisible(true);
        pCameraCameraConfigWidget->setVisible(false);
    }
    break;

    case EXTRINSIC_LIDAR_LIDAR_CALIBRATION:
    {
        pCameraLidarConfigWidget->setVisible(false);
        pCameraReferenceConfigWidget->setVisible(false);
        pLidarReferenceConfigWidget->setVisible(false);
        pLidarLidarConfigWidget->setVisible(true);
        pCameraCameraConfigWidget->setVisible(false);
    }
    break;

    case EXTRINSIC_LIDAR_REFERENCE_CALIBRATION:
    {
        pCameraLidarConfigWidget->setVisible(false);
        pCameraReferenceConfigWidget->setVisible(false);
        pLidarLidarConfigWidget->setVisible(false);
        pLidarReferenceConfigWidget->setVisible(true);
        pCameraCameraConfigWidget->setVisible(false);
    }
    break;

    case STEREO_CAMERA_CALIBRATION:
    {
        pCameraLidarConfigWidget->setVisible(false);
        pCameraReferenceConfigWidget->setVisible(false);
        pLidarLidarConfigWidget->setVisible(false);
        pLidarReferenceConfigWidget->setVisible(false);
        pCameraCameraConfigWidget->setVisible(true);
    }
    break;
    }

    this->adjustSize();
    this->setMinimumHeight(this->size().height());
}

//==================================================================================================
void CalibrationConfigDialog::handleRootDirPushButtonClicked()
{
    QString newRootDir = QFileDialog::getExistingDirectory(
      this, "Select calibration root directory ...",
      ((calibrationRootDir_.exists()) ? calibrationRootDir_.absolutePath() : QDir::homePath()));

    if (!newRootDir.isEmpty())
    {
        calibrationRootDir_.setPath(newRootDir);

        updateRootDirLabelContents();
        populateWsFolderComboBox();
    }
}

//==================================================================================================
void CalibrationConfigDialog::handleSelectUrdfPushButtonClicked()
{
    QString currentUrdfFilePath = ui->urdfLineEdit->text();

    //--- compute parent directory of current urdf file path
    //--- if empty, set to robot directory
    //--- if relative, set to parent directory relative to robot directory
    //--- else set to parent directory
    QDir urdfFileParentDir;
    if (currentUrdfFilePath.isEmpty())
    {
        urdfFileParentDir = calibrationRootDir_;
        urdfFileParentDir.cd(ui->wsFolderComboBox->currentText());
    }
    else if (QFileInfo(currentUrdfFilePath).isRelative())
    {
        urdfFileParentDir = calibrationRootDir_;
        urdfFileParentDir.cd(ui->wsFolderComboBox->currentText());
        urdfFileParentDir.cd(QFileInfo(currentUrdfFilePath).dir().path());
    }
    else
    {
        urdfFileParentDir = QFileInfo(currentUrdfFilePath).dir();
    }

    QString newFile = QFileDialog::getOpenFileName(
      this, "Select URDF Model file ...", urdfFileParentDir.path(), "URDF Model Files (*.urdf)");

    if (!newFile.isEmpty())
        ui->urdfLineEdit->setText(newFile);
}

//==================================================================================================
void CalibrationConfigDialog::handleWsFolderChanged()
{
    loadRobotSettings();

    auto topicList = utils::getTopicList();

    QString robotWsFolderPath = calibrationRootDir_.absolutePath() +
                                QDir::separator() +
                                ui->wsFolderComboBox->currentText();

    pCameraLidarConfigWidget->setTopicList(topicList);
    pCameraReferenceConfigWidget->setTopicList(topicList);
    pLidarLidarConfigWidget->setTopicList(topicList);
    pLidarReferenceConfigWidget->setTopicList(topicList);
    pCameraCameraConfigWidget->setTopicList(topicList);

    pCameraLidarConfigWidget->setRobotWorkspaceFolderPath(robotWsFolderPath);
    pCameraReferenceConfigWidget->setRobotWorkspaceFolderPath(robotWsFolderPath);
    pLidarLidarConfigWidget->setRobotWorkspaceFolderPath(robotWsFolderPath);
    pLidarReferenceConfigWidget->setRobotWorkspaceFolderPath(robotWsFolderPath);
    pCameraCameraConfigWidget->setRobotWorkspaceFolderPath(robotWsFolderPath);
}

} // namespace multisensor_calibration
