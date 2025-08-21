/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/io/Workspace.h"

// Std
#include <map>
#include <string>

// Qt
#include <QFile>

// multisensor_calibration
#include "multisensor_calibration/common/utils.hpp"

namespace multisensor_calibration
{

// Map to convert workspace type to string.
static std::map<EWorkspaceType, std::string> WORKSPACE_TYPE_2_STR =
  {{ROBOT_WS, "robot"},
   {INTRINSIC_CAMERA_WS, "intrinsic-camera-calibration"},
   {EXTRINSIC_CAMERA_LIDAR_WS, "extrinsic-camera-lidar-calibration"},
   {EXTRINSIC_CAMERA_REFERENCE_WS, "extrinsic-camera-reference-calibration"},
   {EXTRINSIC_LIDAR_LIDAR_WS, "extrinsic-lidar-lidar-calibration"},
   {EXTRINSIC_LIDAR_REFERENCE_WS, "extrinsic-lidar-reference-calibration"},
   {EXTRINSIC_LIDAR_VEHICLE_WS, "extrinsic-lidar-vehicle-calibration"}};

// Map to get workspace type from string.
static std::map<EWorkspaceType, std::string> WORKSPACE_TYPE_2_STR_TITLE =
  {{ROBOT_WS, "Robot"},
   {INTRINSIC_CAMERA_WS, "Intrinsic-Camera-Calibration"},
   {EXTRINSIC_CAMERA_LIDAR_WS, "Extrinsic-Camera-LiDAR-Calibration"},
   {EXTRINSIC_CAMERA_REFERENCE_WS, "Extrinsic-Camera-Reference-Calibration"},
   {EXTRINSIC_LIDAR_LIDAR_WS, "Extrinsic-LiDAR-LiDAR-Calibration"},
   {EXTRINSIC_LIDAR_REFERENCE_WS, "Extrinsic-LiDAR-Reference-Calibration"},
   {EXTRINSIC_LIDAR_VEHICLE_WS, "Extrinsic-LiDAR-Vehicle-Calibration"}};

//==================================================================================================
template <EWorkspaceType TypeT>
Workspace<TypeT>::Workspace(const std::string& iWsPath,
                            const rclcpp::Logger& iLogger) :
  wsPath_(iWsPath),
  pSettings_(nullptr),
  logger_(iLogger),
  settingsTplFileName_("")
{
}

//==================================================================================================
template <EWorkspaceType TypeT>
Workspace<TypeT>::Workspace(const Workspace& rhs) :
  wsPath_(rhs.wsPath_),
  logger_(rhs.logger_),
  settingsTplFileName_(rhs.settingsTplFileName_)
{
}

//==================================================================================================
template <EWorkspaceType TypeT>
Workspace<TypeT>::Workspace(Workspace&& rhs) :
  wsPath_(rhs.wsPath_),
  logger_(rhs.logger_),
  settingsTplFileName_(rhs.settingsTplFileName_)
{
}

//==================================================================================================
template <EWorkspaceType TypeT>
Workspace<TypeT>::~Workspace()
{
}

//==================================================================================================
template <EWorkspaceType TypeT>
Workspace<TypeT>& Workspace<TypeT>::operator=(const Workspace& rhs)
{
    wsPath_              = rhs.wsPath_;
    logger_              = rhs.logger_;
    settingsTplFileName_ = rhs.settingsTplFileName_;

    return *this;
}

//==================================================================================================
template <EWorkspaceType TypeT>
Workspace<TypeT>& Workspace<TypeT>::operator=(Workspace&& rhs)
{
    if (this != &rhs)
    {
        wsPath_              = rhs.wsPath_;
        logger_              = rhs.logger_;
        settingsTplFileName_ = rhs.settingsTplFileName_;
    }

    return *this;
}

//==================================================================================================
template <EWorkspaceType TypeT>
bool Workspace<TypeT>::createBackup() const
{
    using namespace std::chrono;

    //--- check if workspace exists and is not empty.
    //--- if so return with true, since no backup needs to be created.
    if (!fs::exists(wsPath_) ||
        (fs::exists(wsPath_) && fs::is_empty(wsPath_)))
        return true;

    //--- check if at least one required files exist
    bool requiredFileExist = false;
    for (auto file : requiredFilesForBackup_)
        requiredFileExist |= fs::exists(
          fs::path(wsPath_.string() + fs::path::preferred_separator + file));
    if (!requiredFilesForBackup_.empty() && !requiredFileExist)
        return true;

    //--- get last modification date of directory
    //-- https://stackoverflow.com/questions/56788745/how-to-convert-stdfilesystemfile-time-type-to-a-string-using-gcc-9
    std::time_t modTime =
      system_clock::to_time_t(
        time_point_cast<system_clock::duration>(last_write_time(wsPath_) -
                                                fs::file_time_type::clock::now() +
                                                system_clock::now()));

    //--- create backup directory
    std::stringstream backupPathStrS;
    backupPathStrS << fs::canonical(wsPath_).string()
                   << fs::path::preferred_separator
                   << WS_BACKUP_SUBDIR_NAME;
    fs::path backupRootDir = fs::path(backupPathStrS.str());

    backupPathStrS << fs::path::preferred_separator
                   << std::put_time(std::localtime(&modTime), "%F_%T");
    fs::path backupPath = fs::path(backupPathStrS.str());

    fs::create_directories(backupPath);

    //--- copy/move workspace content (excluding $WS_BACKUP_SUBDIR_NAME to backup)
    for (auto const& dirEntry : fs::directory_iterator{wsPath_})
    {
        if (fs::canonical(dirEntry.path()) == backupRootDir)
            continue;

        //--- copy
        if (dirEntry.is_directory())
        {
            std::string dirName =
              dirEntry.path().string().substr(dirEntry.path().string().find_last_of('/') + 1);
            fs::path backupSubPath = backupPath;
            backupSubPath.append(dirName);

            fs::create_directories(backupSubPath);
            fs::copy(dirEntry.path(), backupSubPath,
                     fs::copy_options::overwrite_existing | fs::copy_options::recursive);
        }
        else
        {
            fs::copy(dirEntry.path(), backupPath,
                     fs::copy_options::overwrite_existing);
        }

        //--- if not settings file, delete after copy
        if (fs::canonical(dirEntry.path()) !=
            fs::canonical(fs::path(wsPath_.string() + fs::path::preferred_separator +
                                   SETTINGS_FILE_NAME)))
            fs::remove_all(dirEntry.path());
    }

    return true;
}

//==================================================================================================
template <EWorkspaceType TypeT>
fs::path Workspace<TypeT>::getPath() const
{
    return wsPath_;
}

//==================================================================================================
template <EWorkspaceType TypeT>
bool Workspace<TypeT>::initialize()
{
    return Workspace<TypeT>::initializeNonExistingWorkspace(
      wsPath_,
      ":/settings_templates/" + settingsTplFileName_);
}

//==================================================================================================
template <EWorkspaceType TypeT>
bool Workspace<TypeT>::isValid() const
{
    return Workspace<TypeT>::isValid(wsPath_);
}

//==================================================================================================
template <EWorkspaceType TypeT>
bool Workspace<TypeT>::load(const bool forceMkWs,
                            const bool forceOverwrite)
{
    // Flag to indicate if initialization of workspace or the copying of the settings template is
    // successful.
    bool isInitializationSuccessful = true;

    // Path to settings file in workspace.
    std::string settingsFilePathStr = wsPath_.string().append(fs::path::preferred_separator +
                                                              SETTINGS_FILE_NAME);

    //--- check if robot workspace exists
    if (!fs::exists(wsPath_))
    {
        //--- if 'make workspace' is not to be forced
        if (!forceMkWs)
        {

            RCLCPP_ERROR(logger_, "Workspace does not exist. "
                                  "\nPath: %s",
                         wsPath_.c_str());
            return false;
        }
        //--- initialize workspace, if user wants to create it, or if to be forcefully created
        else
        {
            isInitializationSuccessful &=
              initializeNonExistingWorkspace(wsPath_,
                                             ":/settings_templates/" + settingsTplFileName_);
        }
    }
    //--- workspace exists and settings file does not exist, or if overwrite is to be enforced
    //--- copy settings template
    else if (!fs::exists(settingsFilePathStr) || forceOverwrite)
    {
        //--- copy template of settings file
        isInitializationSuccessful &=
          copySettingsTemplate(":/settings_templates/" + settingsTplFileName_,
                               wsPath_);
    }

    //--- check if initialization was successful
    if (!isInitializationSuccessful)
    {
        RCLCPP_ERROR(logger_, "Something went wrong in the initialization of the %s workspace. "
                              "Path: %s",
                     WORKSPACE_TYPE_2_STR[TypeT].c_str(), wsPath_.c_str());
        return false;
    }

    //--- load settings file and return
    pSettings_.reset(new QSettings(QString::fromStdString(settingsFilePathStr),
                                   QSettings::IniFormat));
    QString wsTypeStr = pSettings_->value("workspace/type", "").toString();

    //--- if workspace type inside settings file is not set, clear settings file and write
    //--- type.
    //--- else, check if value in settings file matches type of workspace.
    if (wsTypeStr.isEmpty())
    {
        pSettings_->clear();
        pSettings_->setValue("workspace/type", WORKSPACE_TYPE_2_STR[TypeT].c_str());
    }
    else if (wsTypeStr.toStdString() != WORKSPACE_TYPE_2_STR[TypeT])
    {
        RCLCPP_ERROR(logger_, "Settings file does not correspond to type of workspace. "
                              "Workspace Path: %s",
                     wsPath_.c_str());
        return false;
    }

    //--- check for status of settings object and return accordingly
    if (pSettings_->status() == QSettings::NoError)
        return true;
    else
        return false;
}

//==================================================================================================
template <EWorkspaceType TypeT>
QSettings* Workspace<TypeT>::settingsPtr()
{
    return pSettings_.get();
}

//==================================================================================================
template <EWorkspaceType TypeT>
bool Workspace<TypeT>::isValid(const fs::path& iDirectory)
{
    if (!fs::exists(iDirectory) || !fs::is_directory(iDirectory))
        return false;

    QString settingsFilePathStr = QString::fromStdString(iDirectory) +
                                  fs::path::preferred_separator +
                                  QString::fromStdString(SETTINGS_FILE_NAME);
    if (!fs::exists(settingsFilePathStr.toStdString()))
        return false;

    QSettings settings(settingsFilePathStr, QSettings::Format::IniFormat);
    if (settings.status() != QSettings::NoError)
        return false;

    QString wsTypeStr = settings.value("workspace/type", "").toString();

    return (wsTypeStr.toStdString() == WORKSPACE_TYPE_2_STR[TypeT]);
}

//==================================================================================================
template <EWorkspaceType TypeT>
bool Workspace<TypeT>::copySettingsTemplate(const std::string& iTplResourceStr,
                                            const std::string& iDirectory)
{
    bool isSuccessful = true;

    //--- copy template file from resource to directory
    QString settingsFilePathStr = QString::fromStdString(iDirectory) +
                                  fs::path::preferred_separator +
                                  QString::fromStdString(SETTINGS_FILE_NAME);
    isSuccessful &= QFile::copy(QString::fromStdString(iTplResourceStr), settingsFilePathStr);

    std::cout << "copy: " << ((isSuccessful) ? "true" : "false") << " " << iTplResourceStr << " " << iDirectory << std::endl;

    //--- set permissions of settings file
    isSuccessful &= QFile(settingsFilePathStr)
                      .setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner |
                                      QFileDevice::ReadGroup | QFileDevice::WriteGroup |
                                      QFileDevice::ReadOther);

    return isSuccessful;
}

//==================================================================================================
template <EWorkspaceType TypeT>
bool Workspace<TypeT>::initializeNonExistingWorkspace(const fs::path& iWorkspacePath,
                                                      const std::string& iTplResourceStr)
{
    bool isSuccessful = true;

    //--- create workspace directory
    isSuccessful &= fs::create_directories(iWorkspacePath);

    //--- copy template file from resource to directory
    isSuccessful &= copySettingsTemplate(iTplResourceStr, iWorkspacePath);

    return isSuccessful;
}

} // namespace multisensor_calibration

template class multisensor_calibration::Workspace<multisensor_calibration::ROBOT_WS>;
template class multisensor_calibration::Workspace<multisensor_calibration::INTRINSIC_CAMERA_WS>;
template class multisensor_calibration::Workspace<multisensor_calibration::EXTRINSIC_CAMERA_LIDAR_WS>;
template class multisensor_calibration::Workspace<multisensor_calibration::EXTRINSIC_CAMERA_REFERENCE_WS>;
template class multisensor_calibration::Workspace<multisensor_calibration::EXTRINSIC_LIDAR_LIDAR_WS>;
template class multisensor_calibration::Workspace<multisensor_calibration::EXTRINSIC_LIDAR_REFERENCE_WS>;
template class multisensor_calibration::Workspace<multisensor_calibration::EXTRINSIC_LIDAR_VEHICLE_WS>;
