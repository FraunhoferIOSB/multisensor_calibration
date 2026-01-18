/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/CalibrationGuiBase.h"

// Std
#include <cassert>
#include <regex>

// Qt
#include <QAction>
#include <QApplication>
#include <QDesktopServices>
#include <QDesktopWidget>
#include <QDirIterator>
#include <QFileDialog>
#include <QLocale>
#include <QMenu>
#include <QMessageBox>
#include <QScreen>
#include <QStyle>
#include <QUrl>

// ROS
#include <rclcpp/client.hpp>
#include <rclcpp/service.hpp>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/sensor_data_processing/DataProcessor3d.h"
#include <multisensor_calibration/common/utils.hpp>
#include <multisensor_calibration_interface/srv/capture_calib_target.hpp>
#include <multisensor_calibration_interface/srv/finalize_calibration.hpp>
#include <multisensor_calibration_interface/srv/import_marker_observations.hpp>
#include <multisensor_calibration_interface/srv/remove_last_observation.hpp>
#include <multisensor_calibration_interface/srv/reset_calibration.hpp>

using namespace std::chrono_literals;

namespace multisensor_calibration
{

//==================================================================================================
CalibrationGuiBase::CalibrationGuiBase(const std::string& iAppTitle,
                                       const std::string& iGuiSubNamespace) :
  GuiBase(iAppTitle, iGuiSubNamespace),
  calibratorNodeName_(""),
  guidanceNodeName_(""),
  visualizerNodeName_(""),
  pCalibControlWindow_(nullptr),
  pProgressDialog_(nullptr),
  pRqtReconfigureProcess_(nullptr)
{
    //--- initialize calibration meta data timer
    calibMetaDataTimer_.setInterval(1000);
    calibMetaDataTimer_.setSingleShot(false);
    connect(&calibMetaDataTimer_, &QTimer::timeout, this, &CalibrationGuiBase::getCalibrationMetaData);
}

//==================================================================================================
CalibrationGuiBase::~CalibrationGuiBase()
{
}

//==================================================================================================
std::string CalibrationGuiBase::getCalibratorNodeName() const
{
    return calibratorNodeName_;
}

//==================================================================================================
bool CalibrationGuiBase::init(const std::shared_ptr<rclcpp::Executor>& ipExec,
                              const rclcpp::NodeOptions& iNodeOpts)
{
    if (!GuiBase::init(ipExec, iNodeOpts))
        return false;

    //--- set component names
    calibratorNodeName_ = appTitle_;
    guidanceNodeName_   = appTitle_ + "_" + GUIDANCE_SUB_NAMESPACE;
    visualizerNodeName_ = appTitle_ + "_" + VISUALIZER_SUB_NAMESPACE;

    //--- initialize subscribers
    isInitialized_ &= initializeSubscribers(pNode_.get());

    //--- setup GUI
    isInitialized_ &= setupGuiElements();

    //--- start ros event loop
    if (isInitialized_)
    {
        calibMetaDataTimer_.start();
    }

    //--- activate visualization push button in gui
    if (pCalibControlWindow_)
        pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled((pExecutor_ != nullptr));

    return true;
}

//==================================================================================================
void CalibrationGuiBase::hideProgressDialog()
{
    //--- update mouse cursor
    QApplication::restoreOverrideCursor();

    //--- update progress dialog
    if (pProgressDialog_)
        pProgressDialog_->reset();
    QApplication::processEvents();
}

//==================================================================================================
void CalibrationGuiBase::initializeGuiContents()
{

    //--- add submenue to import observations
    QString importObsToolTip = "Import observations of the calibration target from directory to the "
                               "corresponding sensor. This will remove all previously added/captured "
                               "observations.";
    QMenu* pImportObsSubMenu = new QMenu(pCalibControlWindow_.get());
    pImportObsSubMenu->setToolTip(importObsToolTip);

    // function to add action to import observations for given sensor name
    auto addImportAction = [&](const std::string& sensorName)
    {
        QAction* pImportObsAction = new QAction(
          QString("to '%1'").arg(QString::fromStdString(sensorName)),
          pCalibControlWindow_.get());
        pImportObsAction->setToolTip(importObsToolTip);
        pImportObsAction->setData(
          QVariant(QString::fromStdString(sensorName)));
        connect(pImportObsAction, &QAction::triggered,
                this, &CalibrationGuiBase::onActionImportObservationsTriggered);

        pImportObsSubMenu->addAction(pImportObsAction);
    };

    //--- only add importation for source sensor if it is not a camera sensor, as this is currently
    //--- not implemented.
    if (pCalibrationMetaData_->calib_type != static_cast<int>(EXTRINSIC_CAMERA_LIDAR_CALIBRATION) &&
        pCalibrationMetaData_->calib_type != static_cast<int>(EXTRINSIC_CAMERA_REFERENCE_CALIBRATION))
    {
        addImportAction(pCalibrationMetaData_->src_sensor_name);
    }

    addImportAction(pCalibrationMetaData_->ref_sensor_name);

    pCalibControlWindow_->actionImportObservationPtr()->setMenu(pImportObsSubMenu);
}

//==================================================================================================
bool CalibrationGuiBase::initializeSubscribers(rclcpp::Node* ipNode)
{
    //--- subscribe to log messages
    pLogSubsc_ = ipNode->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", 10,
      std::bind(&CalibrationGuiBase::onLogMessageReceived, this, std::placeholders::_1));

    //--- subscriber to calib result
    pCalibResultSubsc_ = ipNode->create_subscription<CalibrationResult_Message_T>(
      calibratorNodeName_ + "/" + CALIB_RESULT_TOPIC_NAME, 1,
      std::bind(&CalibrationGuiBase::onCalibrationResultReceived, this, std::placeholders::_1));

    return true;
}

//==================================================================================================
void CalibrationGuiBase::onCalibrationResultReceived(
  const CalibrationResult_Message_T::ConstSharedPtr& ipResultMsg)
{
    UNUSED_VAR(ipResultMsg);

    hideProgressDialog();
}

//==================================================================================================
void CalibrationGuiBase::onLogMessageReceived(
  const rcl_interfaces::msg::Log::ConstSharedPtr pLogMsg)
{
    //--- if name does not equal appTitle_, i.e. if the log message is from another process, return
    if (pLogMsg->name.find(appTitle_) == std::string::npos)
        return;

    //--- print log message in text box of calibration control window
    if (pCalibControlWindow_)
        pCalibControlWindow_->printLogMessage(pLogMsg);

    if (pLogMsg->level < rcl_interfaces::msg::Log::WARN)
        return;
    //--- strip new line of message in order to prepare for regex match
    std::string newlineStripedMsgStr = pLogMsg->msg;
    newlineStripedMsgStr.erase(std::remove(newlineStripedMsgStr.begin(),
                                           newlineStripedMsgStr.end(),
                                           '\n'),
                               newlineStripedMsgStr.cend());

    //--- if log level is higher than WARN and if log message begins with "[<appTitle_>...]",
    //--- additionally print message in QMessagePox
    if (std::regex_match(newlineStripedMsgStr, std::regex("(\\[" + appTitle_ + "(.*)\\])(.*)")))
    {
        //--- strip message from leading '[...]'
        QString strippedMsg = QString::fromStdString(
          pLogMsg->msg.substr(pLogMsg->msg.find_first_of(']') + 2));

        switch (pLogMsg->level)
        {
        default:
        case rcl_interfaces::msg::Log::WARN:
        {
            QMessageBox::information(pCalibControlWindow_.get(), pCalibControlWindow_->windowTitle(),
                                     strippedMsg);
        }
        break;

        case rcl_interfaces::msg::Log::ERROR:
        {
            QMessageBox::warning(pCalibControlWindow_.get(), pCalibControlWindow_->windowTitle(),
                                 strippedMsg);
        }
        break;

        case rcl_interfaces::msg::Log::FATAL:
        {
            QMessageBox::critical(pCalibControlWindow_.get(), pCalibControlWindow_->windowTitle(),
                                  strippedMsg);
        }
        break;
        }
    }
}

//==================================================================================================
bool CalibrationGuiBase::setupGuiElements()
{
    //--- get screen geometry to later place the individual windows
    screenGeometry_ = QApplication::primaryScreen()->availableGeometry();
    screenGeometry_.setHeight(screenGeometry_.height() - 100); // X11: availableGeometry also returns space which is reserved by window manager.

    //--- get height of titlebar
    titleBarHeight_ = QApplication::style()->pixelMetric(QStyle::PM_TitleBarHeight);

    //--- CalibrationControlWindow at the Top-Left Corner of Display
    pCalibControlWindow_ = std::make_shared<CalibrationControlWindow>();
    pCalibControlWindow_->setWindowTitle(QString::fromStdString(appTitle_.substr(1)));
    pCalibControlWindow_->move(screenGeometry_.topLeft());
    pCalibControlWindow_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                       (screenGeometry_.height() / 2) - titleBarHeight_ - 1);
    pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled((pExecutor_ != nullptr));
    pCalibControlWindow_->show();

    //--- connect close signals
    connect(pCalibControlWindow_.get(), &CalibrationControlWindow::closed,
            this, &CalibrationGuiBase::closed);

    //--- connect push buttons to slots
    connect(pCalibControlWindow_->actionOpenCalibWsPtr(), &QAction::triggered,
            this, &CalibrationGuiBase::onActionOpenCalibWsTriggered);
    connect(pCalibControlWindow_->actionOpenRobotWsPtr(), &QAction::triggered,
            this, &CalibrationGuiBase::onActionOpenRobotWsTriggered);
    connect(pCalibControlWindow_->actionResetCalibrationPtr(), &QAction::triggered,
            this, &CalibrationGuiBase::onActionResetCalibTriggered);
    connect(pCalibControlWindow_->actionOpenPreferencesPtr(), &QAction::triggered,
            this, &CalibrationGuiBase::onActionPreferencesTriggered);
    connect(pCalibControlWindow_->pbCaptureTargetPtr(), &QPushButton::clicked,
            this, &CalibrationGuiBase::onCaptureTargetButtonClicked);
    connect(pCalibControlWindow_->pbFinalizeCalibrationPtr(), &QPushButton::clicked,
            this, &CalibrationGuiBase::onFinalizeCalibrationButtonClicked);
    connect(pCalibControlWindow_->pbRemoveObservationPtr(), &QPushButton::clicked,
            this, &CalibrationGuiBase::onRemoveObservationButtonClicked);
    connect(pCalibControlWindow_->pbVisCalibrationPtr(), &QPushButton::clicked,
            this, &CalibrationGuiBase::onVisualizeCalibrationButtonClicked);

    //--- Infinite progress dialog
    pProgressDialog_ = std::make_shared<QProgressDialog>(pCalibControlWindow_.get());
    pProgressDialog_->setWindowTitle("Please Wait!");
    pProgressDialog_->setCancelButton(nullptr);
    pProgressDialog_->setWindowModality(Qt::ApplicationModal);
    pProgressDialog_->setMinimumWidth(300);
    pProgressDialog_->setRange(0, 0); // set to infinite
    pProgressDialog_->setValue(0);

    return (pCalibControlWindow_ != nullptr);
}

//==================================================================================================
void CalibrationGuiBase::showProgressDialog(const QString& iBusyText)
{
    //--- update progress dialog
    if (pProgressDialog_)
    {
        pProgressDialog_->setLabelText(iBusyText);
        pProgressDialog_->show();
    }

    //--- update mouse cursor
    QApplication::setOverrideCursor(Qt::BusyCursor);
    QApplication::processEvents();
}

//==================================================================================================
void CalibrationGuiBase::getCalibrationMetaData()
{
    //--- get calibration meta data
    auto pMetaDataClient =
      pNode_->create_client<interf::srv::CalibrationMetaData>(calibratorNodeName_ +
                                                              "/" + REQUEST_META_DATA_SRV_NAME);

    bool isServiceAvailable = false;
    const int MAX_TRIES     = 10;
    int cntr                = 0;

    while (!isServiceAvailable && cntr < MAX_TRIES)
    {
        isServiceAvailable = pMetaDataClient->wait_for_service(500ms);
        cntr++;
    }

    if (isServiceAvailable)
    {

        auto request  = std::make_shared<interf::srv::CalibrationMetaData::Request>();
        auto response = pMetaDataClient->async_send_request(request);

        if (pExecutor_->spin_until_future_complete(response) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            pCalibrationMetaData_ = response.get();

            //--- if calibration metadata is complete stop timer
            if (pCalibrationMetaData_->is_complete)
            {
                calibMetaDataTimer_.stop();
                initializeGuiContents();
            }
        }
        else
        {
            RCLCPP_ERROR(pNode_->get_logger(),
                         "Failure in getting calibration meta data.\n"
                         "Check if calibration node is initialized!");
        }
    }
    else
    {
        RCLCPP_ERROR(pNode_->get_logger(),
                     "Service to get calibration meta data is not available.\n"
                     "Check if calibration node is initialized!");
    }
}

//==================================================================================================
void CalibrationGuiBase::onActionOpenCalibWsTriggered() const
{
    QDesktopServices::openUrl(
      QUrl(QString("file://%1")
             .arg(QString::fromStdString(pCalibrationMetaData_->calib_ws_path))));
}

//==================================================================================================
void CalibrationGuiBase::onActionOpenRobotWsTriggered() const
{
    QDesktopServices::openUrl(
      QUrl(QString("file://%1")
             .arg(QString::fromStdString(pCalibrationMetaData_->robot_ws_path))));
}

//==================================================================================================
void CalibrationGuiBase::onActionPreferencesTriggered()
{
    if (!pRqtReconfigureProcess_)
    {
        pRqtReconfigureProcess_ = std::make_shared<QProcess>(this);
        pRqtReconfigureProcess_->setProgram("ros2");
        pRqtReconfigureProcess_->setArguments({"run", "rqt_reconfigure", "rqt_reconfigure"});
    }

    if (pRqtReconfigureProcess_ && pRqtReconfigureProcess_->state() == QProcess::NotRunning)
    {
        //--- Start process
        pRqtReconfigureProcess_->start();
    }
    if (pRqtReconfigureProcess_ && pRqtReconfigureProcess_->state() == QProcess::Running)
    {
        //--- Kill process to bring window to the front

        pRqtReconfigureProcess_->kill();

        bool isFinished = false;
        do
        {
            QCoreApplication::processEvents();
            isFinished = pRqtReconfigureProcess_->waitForFinished(500);
        } while (!isFinished);

        pRqtReconfigureProcess_->start();
    }
}

//==================================================================================================
void CalibrationGuiBase::onActionResetCalibTriggered()
{
    // Function to do service call
    auto doServiceCall = [&](const std::string& serviceName)
    {
        //--- call service to reset calibration
        auto pResetClient = pNode_->create_client<interf::srv::ResetCalibration>(serviceName);

        bool isServiceAvailable = false;
        const int MAX_TRIES     = 10;
        int cntr                = 0;

        while (!isServiceAvailable && cntr < MAX_TRIES)
        {
            isServiceAvailable = pResetClient->wait_for_service(500ms);
            cntr++;
        }

        if (isServiceAvailable)
        {
            auto request  = std::make_shared<interf::srv::ResetCalibration::Request>();
            auto response = pResetClient->async_send_request(request);

            auto retCode = utils::doWhileWaiting(
              pExecutor_, response, [&]()
              { QCoreApplication::processEvents(); },
              100);

            assert(response.valid() && "Response is not valid");
            if (retCode != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(pNode_->get_logger(),
                             "Failure in calling service '%s'.", serviceName.c_str());
                return;
            }

            auto data = response.get();
            if (!data->is_accepted)
            {
                RCLCPP_ERROR(pNode_->get_logger(),
                             "Failed to reset (service '%s'). %s",
                             serviceName.c_str(), data->msg.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(pNode_->get_logger(),
                         "Service '%s' is not available.", serviceName.c_str());
        }
    };

    //--- show progress dialog
    showProgressDialog("Reset calibration ...");

    //--- reset asynchronously
    doServiceCall(guidanceNodeName_ + "/" + RESET_SRV_NAME);
    doServiceCall(calibratorNodeName_ + "/" + RESET_SRV_NAME);

    pCalibControlWindow_->clearLogMessages();

    hideProgressDialog();
}

//==================================================================================================
void CalibrationGuiBase::onActionImportObservationsTriggered()
{
    //--- get sensor name from the data of the action
    QString sensorName = "";
    QAction* pAction   = dynamic_cast<QAction*>(sender());
    if (pAction)
        sensorName = pAction->data().toString();
    else
        return;

    //--- open file dialog to choose directory
    QString directoryPath = QFileDialog::getExistingDirectory(
      pCalibControlWindow_.get(),
      QString("Open directory to import observations to '%1'...").arg(sensorName),
      QString::fromStdString(pCalibrationMetaData_->calib_ws_path));

    if (directoryPath.isEmpty())
        return;

    //--- get contents
    QDir observationParentDir(directoryPath);
    QStringList subdirList =
      observationParentDir.entryList(QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot);

    //--- check if subdirs exist and the iteration index starts of with 1
    if (subdirList.empty() ||
        !QFileInfo::exists(observationParentDir.absolutePath() + QDir::separator() +
                           "1" + QDir::separator() +
                           sensorName + QString::fromStdString(OBSERVATIONS_FILE_SUFFIX)))
    {
        QMessageBox::critical(pCalibControlWindow_.get(), "No observations found",
                              QString("There has been an error in trying to import observations from "
                                      "directory '%1'. Make sure that the given directory holds a number of "
                                      "subdirectories with the calibration iteration number "
                                      "(starting with '1') as directory name, e.g. 1, 2, 3, ... . "
                                      "And each subdirectory should hold a file named "
                                      "<sensor_name>%2. Take a look at the "
                                      "saved observations in the calibration workspace.")
                                .arg(directoryPath)
                                .arg(QString::fromStdString(OBSERVATIONS_FILE_SUFFIX)));
    }
    else
    {
        // service message to import marker observations
        auto importMarkerObsSrvReq =
          std::make_shared<interf::srv::ImportMarkerObservations::Request>();

        //--- read observation data from individual directories
        for (int i = 1; i <= subdirList.size(); i++)
        {
            // path to file holding the corner observation of the specific calibration iteration
            fs::path observationFilePath = fs::path(
              (observationParentDir.absolutePath() + QDir::separator() +
               QString::number(i) + QDir::separator() +
               sensorName + QString::fromStdString(OBSERVATIONS_FILE_SUFFIX))
                .toStdString());

            //--- if file does not exist, continue
            if (!fs::exists(observationFilePath))
                continue;

            //--- read marker observations from file
            std::vector<uint> markerIds;
            std::vector<std::array<cv::Point3f, 4>> markerCorners;
            DataProcessor3d::readMarkerObservationsFromFile(observationFilePath,
                                                            markerIds, markerCorners);

            //--- add marker observations to service message
            interf::msg::MarkerObservations markerObsMsg;
            for (uint j = 0; j < markerIds.size(); j++)
            {
                markerObsMsg.marker_ids.push_back(markerIds[j]);

                markerObsMsg.marker_top_left_point.push_back(geometry_msgs::msg::Point());
                markerObsMsg.marker_top_left_point.back().x = markerCorners[j][0].x;
                markerObsMsg.marker_top_left_point.back().y = markerCorners[j][0].y;
                markerObsMsg.marker_top_left_point.back().z = markerCorners[j][0].z;
            }

            importMarkerObsSrvReq->observation_list.push_back(markerObsMsg);
        }

        // Function to do service call
        auto doServiceCall = [&](const std::string& serviceName,
                                 interf::srv::ImportMarkerObservations::Request::SharedPtr request)
        {
            //--- call service to import marker observations
            auto pCaptureClient = pNode_->create_client<interf::srv::ImportMarkerObservations>(serviceName);

            bool isServiceAvailable = false;
            const int MAX_TRIES     = 10;
            int cntr                = 0;

            while (!isServiceAvailable && cntr < MAX_TRIES)
            {
                isServiceAvailable = pCaptureClient->wait_for_service(500ms);
                cntr++;
            }

            if (isServiceAvailable)
            {
                auto response = pCaptureClient->async_send_request(request);

                auto retCode = utils::doWhileWaiting(
                  pExecutor_, response, [&]()
                  { QCoreApplication::processEvents(); },
                  100);

                assert(response.valid() && "Response is not valid");
                if (retCode != rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_ERROR(pNode_->get_logger(),
                                 "Failure in calling service '%s'.", serviceName.c_str());
                    return;
                }

                auto data = response.get();
                if (!data->is_accepted)
                {
                    RCLCPP_ERROR(pNode_->get_logger(),
                                 "Failed to import marker observations. %s",
                                 data->msg.c_str());
                }
            }
            else
            {
                RCLCPP_ERROR(pNode_->get_logger(),
                             "Service '%s' is not available.", serviceName.c_str());
            }
        };

        //--- show progress dialog
        showProgressDialog(QString("Import observations to '%1' ...").arg(sensorName));

        //--- call service asynchronously
        doServiceCall(calibratorNodeName_ + "/" +
                        sensorName.toStdString() +
                        "/" +
                        IMPORT_MARKER_OBS_SRV_NAME,
                      importMarkerObsSrvReq);

        hideProgressDialog();
    }
}

//==================================================================================================
void CalibrationGuiBase::onCaptureTargetButtonClicked()
{
    // Function to do service call
    auto doServiceCall = [&](const std::string& serviceName)
    {
        //--- call service to capture target
        auto pCaptureClient = pNode_->create_client<interf::srv::CaptureCalibTarget>(serviceName);

        bool isServiceAvailable = false;
        const int MAX_TRIES     = 10;
        int cntr                = 0;

        while (!isServiceAvailable && cntr < MAX_TRIES)
        {
            isServiceAvailable = pCaptureClient->wait_for_service(500ms);
            cntr++;
        }

        if (isServiceAvailable)
        {
            auto request  = std::make_shared<interf::srv::CaptureCalibTarget::Request>();
            auto response = pCaptureClient->async_send_request(request);

            auto retCode = utils::doWhileWaiting(
              pExecutor_, response, [&]()
              { QCoreApplication::processEvents(); },
              100);

            assert(response.valid() && "Response is not valid");
            if (retCode != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(pNode_->get_logger(),
                             "Failure in calling service '%s'.", serviceName.c_str());
                return;
            }

            auto data = response.get();
            if (!data->is_accepted)
            {
                RCLCPP_ERROR(pNode_->get_logger(),
                             "Failed to capture target. %s",
                             data->msg.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(pNode_->get_logger(),
                         "Service '%s' is not available.", serviceName.c_str());
        }
    };

    //--- show progress dialog
    showProgressDialog("Capturing target ...");

    doServiceCall(calibratorNodeName_ + "/" + CAPTURE_TARGET_SRV_NAME);

    hideProgressDialog();
}

//==================================================================================================
void CalibrationGuiBase::onFinalizeCalibrationButtonClicked()
{

    auto doServiceCall = [&](const std::string& serviceName)
    {
        //--- call service to finalize calibration
        auto pFinalizeClient = pNode_->create_client<interf::srv::FinalizeCalibration>(serviceName);

        bool isServiceAvailable = false;
        const int MAX_TRIES     = 10;
        int cntr                = 0;

        while (!isServiceAvailable && cntr < MAX_TRIES)
        {
            isServiceAvailable = pFinalizeClient->wait_for_service(500ms);
            cntr++;
        }

        if (isServiceAvailable)
        {
            auto request  = std::make_shared<interf::srv::FinalizeCalibration::Request>();
            auto response = pFinalizeClient->async_send_request(request);

            auto retCode = utils::doWhileWaiting(pExecutor_, response, [&]()
                                                 { QCoreApplication::processEvents(); }, 100);

            assert(response.valid() && "Response is not valid");
            if (retCode != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(pNode_->get_logger(),
                             "Failure in calling service '%s'.", serviceName.c_str());
                return;
            }

            auto data = response.get();
            if (!data->is_accepted)
            {
                RCLCPP_ERROR(pNode_->get_logger(),
                             "Failed to finalize calibration. %s",
                             data->msg.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(pNode_->get_logger(),
                         "Service '%s' is not available.", serviceName.c_str());
        }
    };

    //--- show progress dialog
    showProgressDialog("Finalizing calibration ...");

    //--- call service asynchronously
    doServiceCall(calibratorNodeName_ + "/" + FINALIZE_CALIBRATION_SRV_NAME);

    hideProgressDialog();
}

//==================================================================================================
void CalibrationGuiBase::onRemoveObservationButtonClicked()
{
    auto doServiceCall = [&](const std::string& serviceName)
    {
        //--- call service to finalize calibration
        auto pRemoveObsClient =
          pNode_->create_client<interf::srv::RemoveLastObservation>(serviceName);

        bool isServiceAvailable = false;
        const int MAX_TRIES     = 10;
        int cntr                = 0;

        while (!isServiceAvailable && cntr < MAX_TRIES)
        {
            isServiceAvailable = pRemoveObsClient->wait_for_service(500ms);
            cntr++;
        }

        if (isServiceAvailable)
        {
            auto request  = std::make_shared<interf::srv::RemoveLastObservation::Request>();
            auto response = pRemoveObsClient->async_send_request(request);

            auto retCode = utils::doWhileWaiting(pExecutor_, response, [&]()
                                                 { QCoreApplication::processEvents(); }, 100);

            assert(response.valid() && "Response is not valid");
            if (retCode != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(pNode_->get_logger(),
                             "Failure in calling service '%s'.", serviceName.c_str());
                return;
            }

            auto data = response.get();
            if (!data->is_accepted)
            {
                RCLCPP_ERROR(pNode_->get_logger(),
                             "Failed to remove last observation. %s",
                             data->msg.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(pNode_->get_logger(),
                         "Service '%s' is not available.", serviceName.c_str());
        }
    };

    //--- show progress dialog
    showProgressDialog("Removing last observation ...");

    //--- call service asynchronously
    doServiceCall(calibratorNodeName_ + "/" + REMOVE_OBSERVATION_SRV_NAME);

    hideProgressDialog();
}

//==================================================================================================
void CalibrationGuiBase::onVisualizeCalibrationButtonClicked()
{
    loadVisualizer();
}

} // namespace multisensor_calibration
