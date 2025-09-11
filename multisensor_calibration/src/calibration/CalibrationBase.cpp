/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/calibration/CalibrationBase.h"

// Std
#include <functional>
#include <string>

// Qt
#include <QFile>

// PCL
#include <pcl/conversions.h>

namespace multisensor_calibration
{

using namespace std::chrono_literals;

//==================================================================================================
CalibrationBase::CalibrationBase(ECalibrationType type) :
  type_(type),
  isInitialized_(false),
  logger_(rclcpp::get_logger("")),
  tfBuffer_(nullptr),
  tfListener_(nullptr),
  pCaptureSrv_(nullptr),
  pFinalizeSrv_(nullptr),
  pResetSrv_(nullptr),
  pRobotWs_(nullptr),
  robotName_(""),
  isUrdfModelAvailable_(false),
  urdfModelPath_(""),
  urdfModel_(),
  saveObservationsToWs_(false),
  calibTargetFilePath_(""),
  captureCalibrationTarget_(false),
  calibrationItrCnt_(1)
{
}

//==================================================================================================
CalibrationBase::~CalibrationBase()
{
}

//==================================================================================================
bool CalibrationBase::initializeAndStartSensorCalibration(rclcpp::Node* ipNode)
{
    bool isSuccessful = true;

    //--- load robot workspace
    isSuccessful &= loadRobotWorkspace();

    //--- load urdf model, if available
    if (isUrdfModelAvailable_)
        isSuccessful &= loadRobotUrdfModel();

    //--- load calibration workspace
    isSuccessful &= loadCalibrationWorkspace();

    if (!isSuccessful)
    {
        return false;
    }

    //--- initialize sensor data processing
    isSuccessful &= initializeDataProcessors();

    //--- if not successful, print warning and return
    if (!isSuccessful)
    {
        RCLCPP_ERROR(logger_, "Error in the initialization of the sensor data processing!");
        return false;
    }

    //--- initialize subscribers
    isSuccessful &= initializeSubscribers(ipNode);

    //--- if not successful, print warning and return
    if (!isSuccessful)
    {
        RCLCPP_ERROR(logger_, "Error in the initialization of subscribers!");
        return false;
    }

    //--- initialize publishers
    isSuccessful &= initializePublishers(ipNode);

    //--- if not successful, print warning and return
    if (!isSuccessful)
    {
        RCLCPP_ERROR(logger_, "Error in the initialization of publishers!");
        return false;
    }

    RCLCPP_INFO(logger_, "Successfully initialized processing of sensor data.");

    return true;
}

//==================================================================================================
void CalibrationBase::initializeTfListener(rclcpp::Node* ipNode)
{
    tfBuffer_   = std::make_unique<tf2_ros::Buffer>(ipNode->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
}

//==================================================================================================
bool CalibrationBase::initializeServices(rclcpp::Node* ipNode)
{
    //--- capture target service
    pCaptureSrv_ = ipNode->create_service<interf::srv::CaptureCalibTarget>(
      "~/" + CAPTURE_TARGET_SRV_NAME,
      std::bind(&CalibrationBase::onRequestTargetCapture, this,
                std::placeholders::_1, std::placeholders::_2));

    //--- finalize calibration
    pFinalizeSrv_ = ipNode->create_service<interf::srv::FinalizeCalibration>(
      "~/" + FINALIZE_CALIBRATION_SRV_NAME,
      std::bind(&CalibrationBase::onRequestCalibrationFinalization, this,
                std::placeholders::_1, std::placeholders::_2));

    //--- reset service
    pResetSrv_ = ipNode->create_service<interf::srv::ResetCalibration>(
      "~/" + RESET_SRV_NAME,
      std::bind(&CalibrationBase::onReset, this,
                std::placeholders::_1, std::placeholders::_2));

    return true;
}

//==================================================================================================
bool CalibrationBase::initializeWorkspaceObjects()
{
    //--- initialize robot workspace
    pRobotWs_ = std::make_shared<RobotWorkspace>(robotWsPath_, logger_);

    return (pRobotWs_ != nullptr);
}

//==================================================================================================
bool CalibrationBase::isFrameIdInUrdfModel(const std::string& iFrameId) const
{
    bool isSensorInModel = (urdfModel_.getLink(iFrameId) != nullptr);

    return isSensorInModel;
}

//==================================================================================================
bool CalibrationBase::saveCalibrationSettingsToWorkspace()
{
    QSettings* pCalibSettings = pCalibrationWs_->settingsPtr();
    if (!pCalibSettings)
        return false;

    //--- target_config fil
    std::string fileName = calibTargetFilePath_.filename().string();
    pCalibSettings->setValue("calibration/target_config_file",
                             QString::fromStdString(fileName));

    //--- initial guess
    pCalibSettings->setValue("calibration/save_observations",
                             QVariant::fromValue(saveObservationsToWs_));

    //--- copy calibration target file
    QFile calibTargetFile(QString::fromStdString(calibTargetFilePath_.c_str()));
    QString newFilePath = QString::fromStdString(pCalibrationWs_->getPath()) +
                          fs::path::preferred_separator +
                          calibTargetFile.fileName()
                            .split(fs::path::preferred_separator)
                            .back();
    if (QFile(newFilePath).exists())
        QFile::remove(newFilePath);

    bool isSuccesful = true;
    isSuccesful &= calibTargetFile.copy(newFilePath);
    isSuccesful &= calibTargetFile.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner |
                                                  QFileDevice::ReadGroup | QFileDevice::WriteGroup |
                                                  QFileDevice::ReadOther);

    return isSuccesful;
}

//==================================================================================================
void CalibrationBase::setupLaunchParameters(rclcpp::Node* ipNode) const
{
    //--- robot workspace path
    auto robotWsPathDesc = rcl_interfaces::msg::ParameterDescriptor{};
    robotWsPathDesc.description =
      "Path to the folder holding the robot workspace. This path will be created if it does not "
      "yet exist.\n"
      "Default: \"\"";
    robotWsPathDesc.read_only = true;
    ipNode->declare_parameter<std::string>("robot_ws_path", "", robotWsPathDesc);

    //--- path to target configuration file
    auto targetConfigDesc = rcl_interfaces::msg::ParameterDescriptor{};
    targetConfigDesc.description =
      "Path to the file holding the configuration of the calibration target. "
      "E.g. \"$(find multisensor_calibration)/config/TargetWithCirclesAndAruco.yaml\"\n"
      "Default: \"\"";
    targetConfigDesc.read_only = true;
    ipNode->declare_parameter<std::string>("target_config_file", "", targetConfigDesc);

    //--- save observation to workspace
    auto saveObservationsDesc = rcl_interfaces::msg::ParameterDescriptor{};
    saveObservationsDesc.description =
      "Option to save recorded observations that have been used for the calibration to the "
      "workspace.\n"
      "Default: true";
    ipNode->declare_parameter<bool>("save_observations", true, saveObservationsDesc);
}

//==================================================================================================
bool CalibrationBase::readLaunchParameters(const rclcpp::Node* ipNode)
{
    //--- robot workspace path
    std::string robotWsPathStr = ipNode->get_parameter("robot_ws_path").as_string();
    if (robotWsPathStr.empty())
    {
        RCLCPP_ERROR(logger_, "None or empty path string passed to 'robot_ws_path'. "
                              "Please provide valid path to robot workspace.");
        return false;
    }
    robotWsPath_ = fs::absolute(robotWsPathStr);

    //--- path to target configuration file
    std::string targetFileStr = ipNode->get_parameter("target_config_file").as_string();
    if (targetFileStr.empty() || !fs::exists(targetFileStr))
    {
        RCLCPP_ERROR(logger_, "Target configuration file path is empty or does not consist: %s",
                     targetFileStr.c_str());
        return false;
    }
    calibTargetFilePath_ = fs::absolute(targetFileStr);

    //--- save observation to workspace
    saveObservationsToWs_ = ipNode->get_parameter("save_observations").as_bool();

    return true;
}

//==================================================================================================
rcl_interfaces::msg::SetParametersResult CalibrationBase::
  handleDynamicParameterChange(const std::vector<rclcpp::Parameter>& iParameters)
{
    RCLCPP_DEBUG(logger_, "%s", __PRETTY_FUNCTION__);

    // Return value that holds information on success.
    rcl_interfaces::msg::SetParametersResult retVal;
    retVal.successful = true;

    //-- loop through all parameters and set information accordingly
    for (rclcpp::Parameter param : iParameters)
    {
        if (this->setDynamicParameter(param))
        {
            retVal.successful &= true;
            break;
        }
        else
        {
            retVal.successful &= false;
            retVal.reason = std::string(logger_.get_name()) + ": Parameter '" + param.get_name() +
                            "' was not found in parameter list.";
        }
    }

    return retVal;
}

//==================================================================================================
bool CalibrationBase::setDynamicParameter(const rclcpp::Parameter& iParameter)
{
    bool retVal = true;

    if (iParameter.get_name() == "save_observations")
        saveObservationsToWs_ = iParameter.get_value<bool>();
    else
        retVal = false;

    return retVal;
}

//==================================================================================================
template <typename T>
T CalibrationBase::readNumericLaunchParameter(const rclcpp::Node* ipNode,
                                              const std::string& iParamName,
                                              const T& iDefaultVal,
                                              const T& iMinVal,
                                              const T& iMaxVal) const
{
    T num = ipNode->get_parameter(iParamName).get_value<T>();
    if (num < iMinVal)
    {
        RCLCPP_WARN(logger_, "(%s < %i) Setting %s to default: %i",
                    iParamName.c_str(),
                    iMinVal, iParamName.c_str(), iDefaultVal);
        num = iDefaultVal;
    }
    else if (num > iMaxVal)
    {
        RCLCPP_WARN(logger_, "(%s > %i) Setting %s to default : %i ",
                    iParamName.c_str(),
                    iMaxVal, iParamName.c_str(), iDefaultVal);
        num = iDefaultVal;
    }

    return num;
}
template int CalibrationBase::readNumericLaunchParameter<int>(const rclcpp::Node*,
                                                              const std::string&,
                                                              const int&,
                                                              const int&,
                                                              const int&) const;

//==================================================================================================
std::string CalibrationBase::readStringLaunchParameter(const rclcpp::Node* ipNode,
                                                       const std::string& iParamName,
                                                       const std::string& iDefaultVal) const
{

    std::string str = ipNode->get_parameter(iParamName).as_string();

    //--- if passed string is empty and default value is not empty, it is assumed that the parameter
    //--- is not allowed to be empty.
    if (str.empty() && !iDefaultVal.empty())
    {
        RCLCPP_WARN(logger_, "Empty string passed to '%s'. Setting '%s' to default: %s",
                    iParamName.c_str(), iParamName.c_str(), iDefaultVal.c_str());
        str = iDefaultVal;
    }

    return str;
}

//==================================================================================================
void CalibrationBase::reset()
{
    calibrationItrCnt_ = 1;
}

//==================================================================================================
bool CalibrationBase::loadCalibrationWorkspace()
{
    //--- assert that the pointer to the calibration workspace is not null
    assert(pCalibrationWs_ != nullptr);

    //--- get settings object from workspace
    bool isSuccessful = pCalibrationWs_->load(true);

    //--- if not successful check to start again
    if (!isSuccessful)
    {
        RCLCPP_ERROR(logger_, "Loading of calibration workspace was not successful. "
                              "Path: %s.",
                     pCalibrationWs_->getPath().c_str());

        return false;
    }

    //--- save calibration settings to workspace
    isSuccessful = saveCalibrationSettingsToWorkspace();

    RCLCPP_INFO(logger_, "Successfully loaded calibration workspace. "
                         "Path: %s.",
                pCalibrationWs_->getPath().c_str());

    return true;
}

//==================================================================================================
bool CalibrationBase::loadRobotWorkspace()
{
    //--- assert that the pointer to the robot workspace is not null
    assert(pRobotWs_ != nullptr);

    //--- get settings object from workspace
    bool isSuccessful = pRobotWs_->load();

    //--- if not successful print error
    if (!isSuccessful)
    {
        RCLCPP_ERROR(logger_, "Loading of robot workspace was not successful. "
                              "Path: %s.",
                     pRobotWs_->getPath().c_str());

        return false;
    }

    //--- read and check contents of settings file
    isSuccessful = readRobotSettings();
    if (!isSuccessful)
    {
        return false;
    }

    RCLCPP_INFO(logger_, "Successfully loaded robot workspace. "
                         "Path: %s.",
                pRobotWs_->getPath().c_str());

    return true;
}

//==================================================================================================
bool CalibrationBase::loadRobotUrdfModel()
{
    //--- Read URDF model into XML Doc (needed for later manipulation)
    urdfModelDoc_.LoadFile(urdfModelPath_.string().c_str());

    //--- initialized model from file
    bool isSuccessful = urdfModel_.initFile(urdfModelPath_.string());

    //--- if not successful check to start again
    if (!isSuccessful)
    {
        RCLCPP_ERROR(logger_, "Error in reading URDF model from file. "
                              "Model file: %s",
                     urdfModelPath_.string().c_str());
        return false;
    }

    RCLCPP_INFO(logger_, "Successfully parsed URDF model from file. "
                         "Path: %s.",
                urdfModelPath_.c_str());

    return true;
}

//==================================================================================================
bool CalibrationBase::onRequestCalibrationFinalization(
  const std::shared_ptr<interf::srv::FinalizeCalibration::Request> ipReq,
  std::shared_ptr<interf::srv::FinalizeCalibration::Response> opRes)
{
    UNUSED_VAR(ipReq);

    if (!isInitialized_)
    {
        opRes->is_accepted = false;
        opRes->msg         = "Calibration not successful! "
                             "Node is not initialized.";

        RCLCPP_ERROR(logger_, "%s", opRes->msg.c_str());

        return false;
    }

    if (calibrationItrCnt_ <= 1)
    {
        opRes->is_accepted = false;
        opRes->msg         = "Calibration not successful! "
                             "Not enough observations.";

        RCLCPP_ERROR(logger_, "%s", opRes->msg.c_str());

        return false;
    }

    //--- if calibration counter is greater than 1, i.e. if at least one calibration iteration
    //--- has been performed, trigger finalization. otherwise, do not trigger

    //--- finalize calibration
    bool isSuccessful = finalizeCalibration();

    //--- save calibration
    isSuccessful &= saveCalibration();

    if (isSuccessful)
    {
        opRes->is_accepted = true;
        opRes->msg         = "Calibration successful.";

        RCLCPP_INFO(logger_, "%s", opRes->msg.c_str());
    }
    else
    {
        opRes->is_accepted = false;
        opRes->msg         = "Error in finalizing the calibration.";

        RCLCPP_ERROR(logger_, "%s", opRes->msg.c_str());
    }

    return opRes->is_accepted;
}

//==================================================================================================
bool CalibrationBase::onRequestTargetCapture(
  const std::shared_ptr<interf::srv::CaptureCalibTarget::Request> ipReq,
  std::shared_ptr<interf::srv::CaptureCalibTarget::Response> opRes)
{
    UNUSED_VAR(ipReq);

    if (!isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(dataProcessingMutex_);

    //--- store request
    captureCalibrationTarget_ = true;

    //--- write response
    opRes->msg = "Start looking for calibration target!";

    opRes->is_accepted = true;

    RCLCPP_INFO(logger_, "%s", opRes->msg.c_str());

    return true;
}

//==================================================================================================
bool CalibrationBase::onReset(
  const std::shared_ptr<interf::srv::ResetCalibration::Request> ipReq,
  std::shared_ptr<interf::srv::ResetCalibration::Response> opRes)
{
    UNUSED_VAR(ipReq);

    reset();

    opRes->is_accepted = true;
    opRes->msg         = "Calibration is reset.";

    return true;
}

//==================================================================================================
bool CalibrationBase::readRobotSettings()
{
    bool retVal = true;

    // Pointer to robot settings
    QSettings* pRobotSettings = pRobotWs_->settingsPtr();
    if (!pRobotSettings)
        return false;

    // Lambda function to read string parameter with given name (iParamName)..
    // If parameter string is empty, 'isValid' is set to false
    auto readStringParameter = [&](const std::string& iParamName, const bool& isEmptyAllowed,
                                   bool& isValid) -> std::string
    {
        std::string str =
          pRobotSettings->value(QString::fromStdString(iParamName), "").toString().toStdString();
        if (str.empty() && !isEmptyAllowed)
        {
            RCLCPP_ERROR(logger_, "Value provided for '%s' in settings file is empty. "
                                  "\nSettings file: %s"
                                  "\nPlease provide valid string. ",
                         iParamName.c_str(),
                         pRobotSettings->fileName().toStdString().c_str());
            isValid = false;
        }
        else
        {
            isValid = true;
        }

        return str;
    };

    //--- robot name
    robotName_ = readStringParameter("robot/name", false, retVal);

    //--- urdf model path
    std::string pathStr = readStringParameter("robot/urdf_model_path", true, retVal);
    fs::path tmpPath    = fs::path(pathStr);
    if (tmpPath.is_relative())
    {
        urdfModelPath_ = pRobotWs_->getPath();
        urdfModelPath_ /= tmpPath;
    }
    else
    {
        urdfModelPath_ = tmpPath;
    }
    isUrdfModelAvailable_ = (!pathStr.empty() && fs::exists(urdfModelPath_));
    if (!isUrdfModelAvailable_)
    {
        isUrdfModelAvailable_ = false;
        RCLCPP_INFO(logger_, "URDF Model is not available");
        if (!pathStr.empty())
        {
            RCLCPP_WARN(logger_,
                        "Please provide valid path (absolute or relative) to URDF model file. "
                        "URDF file: %s",
                        tmpPath.c_str());
        }
    }

    return retVal;
}

} // namespace multisensor_calibration