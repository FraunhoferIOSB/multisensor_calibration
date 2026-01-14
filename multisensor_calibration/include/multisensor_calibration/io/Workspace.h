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

#ifndef MULTISENSORCALIBRATION_WORKSPACE_H
#define MULTISENSORCALIBRATION_WORKSPACE_H

// Std
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

// Qt
#include <QSettings>

// ROS
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

// multisensor_calibration
#include "../common/common.h"

namespace fs = std::filesystem;

namespace multisensor_calibration
{

/**
 * @ingroup workspace_handling
 * @brief Enum holding type of workspace.
 */
enum EWorkspaceType
{
    ROBOT_WS = 0,                  ///< Robot workspace type.
    INTRINSIC_CAMERA_WS,           ///< Intrinsic camera calibration workspace type.
    EXTRINSIC_CAMERA_LIDAR_WS,     ///< EXtrinsic camera-LiDAR calibration workspace type.
    EXTRINSIC_CAMERA_REFERENCE_WS, ///< EXtrinsic camera-reference calibration workspace type.
    EXTRINSIC_LIDAR_LIDAR_WS,      ///< EXtrinsic LiDAR-LiDAR calibration workspace type.
    EXTRINSIC_LIDAR_REFERENCE_WS,  ///< EXtrinsic LiDAR-reference calibration workspace type.
    EXTRINSIC_LIDAR_VEHICLE_WS,     ///< EXtrinsic LiDAR-Vehicle calibration workspace type.
    EXTRINSIC_CAMERA_CAMERA_WS     ///< EXtrinsic Camera-Camera calibration workspace type.
};

/**
 * @ingroup workspace_handling
 * @brief Abstract interface of workspace class.
 *
 */
class AbstractWorkspace
{
    //--- METHOD DECLARATION ---//

  public:
    virtual ~AbstractWorkspace()
    {
    }

    virtual bool createBackup() const                    = 0;
    virtual fs::path getPath() const                     = 0;
    virtual bool load(const bool forceMkWs      = false,
                      const bool forceOverwrite = false) = 0;
    virtual QSettings* settingsPtr()                     = 0;
};

/**
 * @ingroup workspace_handling
 * @brief Base workspace class, templated by the type of workspace.
 * @tparam TypeT Type of workspace.
 */
template <EWorkspaceType TypeT>
class Workspace : public AbstractWorkspace
{

    //--- METHOD DECLARATION ---//
  public:
    /**
     * @brief Default constructor (deleted)
     */
    Workspace() = delete;

    /**
     * @brief Initialization constructor. This will initialize the workspace object with given
     * variables. It will, however, not load the workspace.
     *
     * @param[in] iWsPath Absolute path to workspace.
     * @param[in] iLogger Logger object of node. Needed for information logging
     */
    Workspace(const std::string& iWsPath,
              const rclcpp::Logger& iLogger);

    /**
     * @brief Copy constructor
     */
    Workspace(const Workspace& rhs);

    /**
     * @brief Move constructor
     */
    Workspace(Workspace&& rhs);

    /**
     * @brief Destructor
     */
    virtual ~Workspace();

    /**
     * @brief Copy assignment operator.
     */
    Workspace& operator=(const Workspace& rhs);

    /**
     * @brief Move assignment operator.
     */
    Workspace& operator=(Workspace&& rhs);

    /**
     * @brief Create a backup of the workspace.
     *
     * @return True, if successful. False, otherwise.
     */
    bool createBackup() const override;

    /**
     * @brief Returns absolute path to workspace.
     */
    fs::path getPath() const override;

    /**
     * @brief Method to initialize non-existing workspace.
     *
     * This will create the directory and copy the given settings file
     * template from resource into the created directory.
     * @return Returns true if successful. False, otherwise.
     */
    bool initialize();

    /**
     * @brief Check if workspace object is valid.
     *
     * This will check if the directory exists and if it holds a corresponding type of workspace.
     * Latter is being checked based on the key inside the settings.ini file.
     */
    bool isValid() const;

    /**
     * @brief Load workspace. This will if check if workspace and settings file exist. If not,
     * it will initialize an empty workspace and ask the user to adjust the settings file. Finally,
     * the settings file is loaded into oSettingsObj,
     *
     * @param[in] forceMkWs If true, this will immediately create the workspace if does not exist.
     * If false (default), the user is notified that the workspace does not exist and is asked if
     * it is do be created.
     * @param[in] forceOverwrite If true, a new settings file is copied into the directory,
     * even if a previous settings file already exists.
     * If false (default), the already existing setting file (if available) is used.
     * @return True, if successful. In this case, the returned settings object is valid. If false,
     * the user has aborted or an error has occurred.
     */
    bool load(const bool forceMkWs      = false,
              const bool forceOverwrite = false) override;

    /**
     * @brief Return pointer to workspace settings.
     *
     * @note This might return a nullptr, if the workspace object has not yet been initialized by
     * Workspace::load().
     */
    QSettings* settingsPtr() override;

    //--- STATIC FUNCTION DECLARATION ---//
  public:
    /**
     * @brief Check if workspace at given path is valid.
     *
     * This will check if the directory exists and if it holds a corresponding type of workspace.
     * Latter is being checked based on the key inside the settings.ini file.
     */
    static bool isValid(const fs::path& iDirectory);

  private:
    /**
     * @brief Method to copy settings template file from resource into given directory.
     *
     * @param[in] iTplResourceStr Path to template file within resource.
     * @param[in] iDirectory Path to directory in which to copy the file.
     * @return Returns true if successful. False, otherwise.
     */
    static bool copySettingsTemplate(const std::string& iTplResourceStr,
                                     const std::string& iDirectory);

    /**
     * @brief Method to initialize non-existing workspace.
     *
     * This will create the directory referenced by \a iWorkspacePath and copy the given settings file
     * template from resource into the created directory.
     *
     * @param[in] iWorkspacePath Path of workspace to be initialized.
     * @param[in] iTplResourceStr Path to template file within resource.
     * @return Returns true if successful. False, otherwise.
     */
    static bool initializeNonExistingWorkspace(const fs::path& iWorkspacePath,
                                               const std::string& iTplResourceStr);

    //--- MEMBER DECLARATION ---//

  protected:
    /// Absolute path to workspace.
    fs::path wsPath_;

    /// Pointer to object settings.
    std::shared_ptr<QSettings> pSettings_;

    /// Logger object of node.
    rclcpp::Logger logger_;

    /// Name of settings template file.
    std::string settingsTplFileName_;

    /// List file that have to exist in directory for backup to be done.
    std::vector<std::string> requiredFilesForBackup_;
};

/**
 * @ingroup workspace_handling
 * @brief Class of a robot workspace.
 */
class RobotWorkspace : public Workspace<ROBOT_WS>
{
    //--- METHOD DECLARATION ---//
  public:
    RobotWorkspace() = delete;

    RobotWorkspace(const std::string& iWsPath,
                   const rclcpp::Logger& iLogger) :
      Workspace<ROBOT_WS>(iWsPath, iLogger)
    {
        settingsTplFileName_ = "robot_ws_settings_template.ini";
    }

    /**
     * @brief Destructor
     */
    virtual ~RobotWorkspace()
    {
    }
};

/**
 * @ingroup workspace_handling
 * @brief Class of an intrinsic camera calibration workspace
 *
 */
class IntrinsicCameraCalibWorkspace : public Workspace<INTRINSIC_CAMERA_WS>
{
    //--- METHOD DECLARATION ---//
  public:
    IntrinsicCameraCalibWorkspace() = delete;

    IntrinsicCameraCalibWorkspace(const std::string& iWsPath,
                                  const rclcpp::Logger& iLogger) :
      Workspace<INTRINSIC_CAMERA_WS>(iWsPath, iLogger)
    {
        requiredFilesForBackup_ = {CALIB_RESULTS_FILE_NAME};
        settingsTplFileName_    = "intrinsic_camera_calib_ws_settings_template.ini";
    }

    /**
     * @brief Destructor
     */
    virtual ~IntrinsicCameraCalibWorkspace()
    {
    }
};

/**
 * @ingroup workspace_handling
 * @brief Class of an extrinsic camera-lidar calibration workspace
 *
 */
class ExtrinsicCameraLidarCalibWorkspace : public Workspace<EXTRINSIC_CAMERA_LIDAR_WS>
{
    //--- METHOD DECLARATION ---//
  public:
    ExtrinsicCameraLidarCalibWorkspace() = delete;

    ExtrinsicCameraLidarCalibWorkspace(const std::string& iWsPath,
                                       const rclcpp::Logger& iLogger) :
      Workspace<EXTRINSIC_CAMERA_LIDAR_WS>(iWsPath, iLogger)
    {
        requiredFilesForBackup_ = {CALIB_RESULTS_FILE_NAME};
        settingsTplFileName_    = "extrinsic_camera_lidar_calib_ws_settings_template.ini";
    }

    /**
     * @brief Destructor
     */
    virtual ~ExtrinsicCameraLidarCalibWorkspace()
    {
    }
};

/**
 * @ingroup workspace_handling
 * @brief Class of an extrinsic camera-reference calibration workspace
 *
 */
class ExtrinsicCameraReferenceCalibWorkspace : public Workspace<EXTRINSIC_CAMERA_REFERENCE_WS>
{
    //--- METHOD DECLARATION ---//
  public:
    ExtrinsicCameraReferenceCalibWorkspace() = delete;

    ExtrinsicCameraReferenceCalibWorkspace(const std::string& iWsPath,
                                           const rclcpp::Logger& iLogger) :
      Workspace<EXTRINSIC_CAMERA_REFERENCE_WS>(iWsPath, iLogger)
    {
        requiredFilesForBackup_ = {CALIB_RESULTS_FILE_NAME};
        settingsTplFileName_    = "extrinsic_camera_reference_calib_ws_settings_template.ini";
    }

    /**
     * @brief Destructor
     */
    virtual ~ExtrinsicCameraReferenceCalibWorkspace()
    {
    }
};

/**
 * @ingroup workspace_handling
 * @brief Class of an extrinsic lidar-lidar calibration workspace
 *
 */
class ExtrinsicLidarLidarCalibWorkspace : public Workspace<EXTRINSIC_LIDAR_LIDAR_WS>
{
    //--- METHOD DECLARATION ---//
  public:
    ExtrinsicLidarLidarCalibWorkspace() = delete;

    ExtrinsicLidarLidarCalibWorkspace(const std::string& iWsPath,
                                      const rclcpp::Logger& iLogger) :
      Workspace<EXTRINSIC_LIDAR_LIDAR_WS>(iWsPath, iLogger)
    {
        requiredFilesForBackup_ = {CALIB_RESULTS_FILE_NAME};
        settingsTplFileName_    = "extrinsic_lidar_lidar_calib_ws_settings_template.ini";
    }

    /**
     * @brief Destructor
     */
    virtual ~ExtrinsicLidarLidarCalibWorkspace()
    {
    }
};

/**
 * @ingroup workspace_handling
 * @brief Class of an extrinsic lidar-reference calibration workspace
 *
 */
class ExtrinsicLidarReferenceCalibWorkspace : public Workspace<EXTRINSIC_LIDAR_REFERENCE_WS>
{
    //--- METHOD DECLARATION ---//
  public:
    ExtrinsicLidarReferenceCalibWorkspace() = delete;

    ExtrinsicLidarReferenceCalibWorkspace(const std::string& iWsPath,
                                          const rclcpp::Logger& iLogger) :
      Workspace<EXTRINSIC_LIDAR_REFERENCE_WS>(iWsPath, iLogger)
    {
        requiredFilesForBackup_ = {CALIB_RESULTS_FILE_NAME};
        settingsTplFileName_    = "extrinsic_lidar_reference_calib_ws_settings_template.ini";
    }

    /**
     * @brief Destructor
     */
    virtual ~ExtrinsicLidarReferenceCalibWorkspace()
    {
    }
};


/**
 * @ingroup workspace_handling
 * @brief Class of an extrinsic camera-lidar calibration workspace
 *
 */
class ExtrinsicCameraCameraCalibWorkspace : public Workspace<EXTRINSIC_CAMERA_CAMERA_WS>
{
    //--- METHOD DECLARATION ---//
  public:
    ExtrinsicCameraCameraCalibWorkspace() = delete;

    ExtrinsicCameraCameraCalibWorkspace(const std::string& iWsPath,
                                       const rclcpp::Logger& iLogger) :
      Workspace<EXTRINSIC_CAMERA_CAMERA_WS>(iWsPath, iLogger)
    {
        requiredFilesForBackup_ = {CALIB_RESULTS_FILE_NAME};
        settingsTplFileName_    = "extrinsic_camera_camera_calib_ws_settings_template.ini";
    }

    /**
     * @brief Destructor
     */
    virtual ~ExtrinsicCameraCameraCalibWorkspace()
    {
    }
};

/**
 * @ingroup workspace_handling
 * @brief Class of an extrinsic lidar-vehicle calibration workspace
 *
 */
class ExtrinsicLidarVehicleCalibWorkspace : public Workspace<EXTRINSIC_LIDAR_VEHICLE_WS>
{
    //--- METHOD DECLARATION ---//
  public:
    ExtrinsicLidarVehicleCalibWorkspace() = delete;

    ExtrinsicLidarVehicleCalibWorkspace(const std::string& iWsPath,
                                        const rclcpp::Logger& iLogger) :
      Workspace<EXTRINSIC_LIDAR_VEHICLE_WS>(iWsPath, iLogger)
    {
        requiredFilesForBackup_ = {CALIB_RESULTS_FILE_NAME};
        settingsTplFileName_    = "extrinsic_lidar_vehicle_calib_ws_settings_template.ini";
    }

    /**
     * @brief Destructor
     */
    virtual ~ExtrinsicLidarVehicleCalibWorkspace()
    {
    }
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_WORKSPACE_H
