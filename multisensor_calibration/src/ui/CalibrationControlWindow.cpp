/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/CalibrationControlWindow.h"

// Qt
#include <QCloseEvent>
#include <QDesktopServices>
#include <QMessageBox>

// multisensor_calibration
#include "ui_CalibrationControlWindow.h"

namespace multisensor_calibration
{

//==================================================================================================
CalibrationControlWindow::CalibrationControlWindow(QWidget* parent) :
  QMainWindow(parent),
  ui(new Ui::CalibrationControlWindow),
  pAboutDialog(new AboutDialog(this))
{
    ui->setupUi(this);
    this->setWindowIcon(QIcon(":/icons/icons8-sensor-100_filled"));

    //--- connections
    connect(ui->actionDocumentation, &QAction::triggered,
            this, &CalibrationControlWindow::onActionDocumentationTriggered);
    connect(ui->actionAbout, &QAction::triggered,
            this, &CalibrationControlWindow::onActionAboutTriggered);
    connect(this, &CalibrationControlWindow::newLogMessage, ui->teLog, &QPlainTextEdit::appendHtml);
}

//==================================================================================================
CalibrationControlWindow::~CalibrationControlWindow()
{
    delete ui;
    pAboutDialog->deleteLater();
}

//==================================================================================================
void CalibrationControlWindow::attachPlacementGuidanceDialog(QDialog* pDialog,
                                                             Qt::CheckState buttonInitialCheckState)
{
    if (pDialog == nullptr)
        return;

    attachDialogToPushButton(pDialog, ui->pbPlacementGuidance, buttonInitialCheckState);
}

//==================================================================================================
void CalibrationControlWindow::attachReferenceDialog(QDialog* pDialog,
                                                     Qt::CheckState buttonInitialCheckState)
{
    if (pDialog == nullptr)
        return;

    attachDialogToPushButton(pDialog, ui->pbRefOutput, buttonInitialCheckState);
}

//==================================================================================================
void CalibrationControlWindow::attachSourceDialog(QDialog* pDialog,
                                                  Qt::CheckState buttonInitialCheckState)
{
    if (pDialog == nullptr)
        return;

    attachDialogToPushButton(pDialog, ui->pbSrcOutput, buttonInitialCheckState);
}

//==================================================================================================
void CalibrationControlWindow::clearLogMessages()
{
    ui->teLog->clear();
}

//==================================================================================================
void CalibrationControlWindow::closeEvent(QCloseEvent* closeEvent)
{
    //--- confirm close
    QMessageBox::StandardButton closeConfirm =
      QMessageBox::question(this, "Close Confirmation",
                            "Do you really want to exit the calibration process?",
                            QMessageBox::Yes | QMessageBox::No);

    if (closeConfirm == QMessageBox::Yes)
    {
        closeEvent->accept();
        emit closed();
    }
    else
    {
        closeEvent->ignore();
    }
}

//==================================================================================================
void CalibrationControlWindow::showEvent(QShowEvent* showEvent)
{
    QMainWindow::showEvent(showEvent);
}

//==================================================================================================
QAction* CalibrationControlWindow::actionResetCalibrationPtr()
{
    return ui->actionNew_Reset;
}

//==================================================================================================
QAction* CalibrationControlWindow::actionOpenPreferencesPtr()
{
    return ui->actionPreferences;
}

//==================================================================================================
QAction* CalibrationControlWindow::actionOpenCalibWsPtr()
{
    return ui->actionCalibration_Workspace;
}

//==================================================================================================
QAction* CalibrationControlWindow::actionOpenRobotWsPtr()
{
    return ui->actionRobot_Workspace;
}

//==================================================================================================
QAction* CalibrationControlWindow::actionAddObservationPtr()
{
    return ui->actionAdd_Observation;
}

//==================================================================================================
QAction* CalibrationControlWindow::actionImportObservationPtr()
{
    return ui->actionImport_Observations_from_Directory;
}

//==================================================================================================
QPushButton* CalibrationControlWindow::pbCaptureTargetPtr()
{
    return ui->pbCapture;
}

//==================================================================================================
QPushButton* CalibrationControlWindow::pbFinalizeCalibrationPtr()
{
    return ui->pbFinalize;
}

//==================================================================================================
QPushButton* CalibrationControlWindow::pbRemoveObservationPtr()
{
    return ui->pbRemove;
}

//==================================================================================================
QPushButton* CalibrationControlWindow::pbVisCalibrationPtr()
{
    return ui->pbVisCalibration;
}

//==================================================================================================
void CalibrationControlWindow::printLogMessage(
  const rcl_interfaces::msg::Log::ConstSharedPtr pLogMsg)
{
    QString plainTxtMessage = QString::fromStdString(pLogMsg->msg);
    QString textColor       = "black";

    //--- update prefix based on log level
    switch (pLogMsg->level)
    {
    case rcl_interfaces::msg::Log::DEBUG:
        plainTxtMessage.prepend("[DEBUG]: ");
        textColor = "darkblue";
        break;

    default:
    case rcl_interfaces::msg::Log::INFO:
        plainTxtMessage.prepend("[ INFO]: ");
        textColor = "black";
        break;

    case rcl_interfaces::msg::Log::WARN:
        plainTxtMessage.prepend("[ WARN]: ");
        textColor = "olive";
        break;

    case rcl_interfaces::msg::Log::ERROR:
        plainTxtMessage.prepend("[ERROR]: ");
        textColor = "darkred";
        break;

    case rcl_interfaces::msg::Log::FATAL:
        plainTxtMessage.prepend("[FATAL]: ");
        textColor = "red";
        break;
    }

    QString htmlTextMessage = QString("<p style=\"color:%1;white-space:pre\">%2</p>")
                                .arg(textColor, plainTxtMessage);

    emit(newLogMessage(htmlTextMessage));
}

//==================================================================================================
void CalibrationControlWindow::attachDialogToPushButton(
  QDialog*& pDialog,
  QPushButton*& pButton,
  Qt::CheckState buttonInitialCheckState) const
{
    //--- connect clicked signal of button, this will show the dialog and disable the push button
    connect(pButton, &QPushButton::clicked,
            [=](const bool isChecked)
            {
                if (isChecked)
                {
                    pDialog->show();
                    pButton->setEnabled(false);
                }
            });

    //--- connect rejection signal, i.e. close signal of dialog, this will uncheck an reenable the
    //--- push button
    connect(pDialog, &QDialog::rejected,
            [=]()
            {
                pButton->setChecked(false);
                pButton->setEnabled(true);
            });

    //--- check button if applicable
    if (buttonInitialCheckState == Qt::Checked ||
        buttonInitialCheckState == Qt::PartiallyChecked)
    {
        pButton->setEnabled(false);
        pButton->setChecked(true);
    }
    else
    {
        pButton->setEnabled(true);
        pButton->setChecked(false);
    }
}

//==================================================================================================
void CalibrationControlWindow::onActionDocumentationTriggered() const
{
    QDesktopServices::openUrl(QUrl("https://fraunhoferiosb.github.io/multisensor_calibration/"));
}

//==================================================================================================
void CalibrationControlWindow::onActionAboutTriggered() const
{
    pAboutDialog->show();
}

} // namespace multisensor_calibration
