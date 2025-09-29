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

#ifndef MULTISENSORCALIBRATION_CALIBRATIONBASE_H
#define MULTISENSORCALIBRATION_CALIBRATIONBASE_H

// Std
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Qt
#include <QSettings>

// ROS
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#if __has_include(<urdf/model.hpp>)
#include <urdf/model.hpp>
#elif __has_include(<urdf/model.h>)
#include <urdf/model.h>
#else
#error "Could not find urdf model header!"
#endif

// Tiny XML
#include <tinyxml2.h>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/io/Workspace.h"
#include <multisensor_calibration/common/lib3D/core/extrinsics.hpp>
#include <multisensor_calibration_interface/srv/capture_calib_target.hpp>
#include <multisensor_calibration_interface/srv/finalize_calibration.hpp>
#include <multisensor_calibration_interface/srv/reset_calibration.hpp>

namespace fs     = std::filesystem;
namespace interf = multisensor_calibration_interface;

namespace multisensor_calibration
{

/**
 * @ingroup calibration
 * @brief Base class of all calibration nodes. This holds a common interface to all calibration
 * nodes.
 */
class CalibrationBase
{
    //--- METHOD DECLARATION ---//
  public:
    /**
     * @brief Default constructor is deleted
     */
    CalibrationBase() = delete;

    /**
     * @brief Initialization constructor
     *
     * @param[in] type Type of calibration.
     */
    CalibrationBase(ECalibrationType type);

    /**
     * @brief Destructor
     */
    virtual ~CalibrationBase();

  protected:
    /**
     * @brief Initialize and start calibration.

     * @param[in, out] ipNode Pointer to node.
     * @return True, if successful. False otherwise.
     */
    bool initializeAndStartSensorCalibration(rclcpp::Node* ipNode);

    /**
     * @brief Initialize TF listener.
     *
     * @param[in] ipNode Pointer to node.
     */
    void initializeTfListener(rclcpp::Node* ipNode);

    /**
     * @brief Finalize calibration.
     *
     * @return True, if successful. False, otherwise.
     */
    virtual bool finalizeCalibration() = 0;

    /**
     * @brief Initialize data processing, i.e. the objects that will process the sensor data and
     * detect the calibration target within the data.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    virtual bool initializeDataProcessors() = 0;

    /**
     * @brief Initialize publishers.
     *
     * @param[in, out] ipNode Pointer to node.
     * @return True, if successful. False otherwise.
     */
    virtual bool initializePublishers(rclcpp::Node* ipNode) = 0;

    /**
     * @brief Initialize services.
     *
     * @param[in, out] ipNode Pointer to node.
     * @return True, if successful. False otherwise.
     */
    virtual bool initializeServices(rclcpp::Node* ipNode);

    /**
     * @brief Initialize subscribers.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    virtual bool initializeSubscribers(rclcpp::Node* ipNode) = 0;

    /**
     * @brief Initialize workspace objects. In this class, i.e. the calibration base class, only the
     * object of the robot workspace is initialized. The initialization requires the launch
     * parameters, thus it is to be executed after the launch parameters are read.
     *
     * @return True if successful. False, otherwise (e.g. if instantiation has failed)
     */
    virtual bool initializeWorkspaceObjects();

    /**
     * @brief Check if given frame ID is in URDF model.
     *
     * @return True, if frame ID is found in model. False, otherwise.
     */
    bool isFrameIdInUrdfModel(const std::string& iFrameId) const;

    /**
     * @brief Save calibration settings to setting.ini inside calibration workspace.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    virtual bool saveCalibrationSettingsToWorkspace();

    /**
     * @brief Setup launch parameters.
     *
     * The implementation within this class hold launch parameters that are common to all
     * calibration nodes.
     *
     * @param[in] ipNode Pointer to node.
     */
    virtual void setupLaunchParameters(rclcpp::Node* ipNode) const;

    /**
     * @brief Setup dynamic parameters.
     *
     * @param[in] ipNode Pointer to node.
     */
    virtual void setupDynamicParameters(rclcpp::Node* ipNode) const = 0;

    /**
     * @brief Read launch parameters.
     *
     * The implementation within this class hold launch parameters that are common to all
     * calibration nodes, e.g. robot_ws_path, target_config_file.
     *
     * @param[in] ipNode Pointer to node.
     * @return True if successful. False, otherwise (e.g. if sanity check fails)
     */
    virtual bool readLaunchParameters(const rclcpp::Node* ipNode);

    /**
     * @brief Handle dynamic change of parameters. For example, through rqt_reconfigure.
     *
     * This loops through the list of parameters in iParameters and sets their value accordingly.
     *
     * @param[in] iParameters Parameters that have changed.
     */
    rcl_interfaces::msg::SetParametersResult handleDynamicParameterChange(
      const std::vector<rclcpp::Parameter>& iParameters);

    /**
     * @brief Virtual function to set dynamic parameter. This is called from
     * handleDynamicParameterChange for each parameter in the list that is to be changed.
     *
     * @param[in] iParameter Parameter that is to be changed.
     * @return True, if successful, i.e. if it has been changed. False, otherwise.
     */
    virtual bool setDynamicParameter(const rclcpp::Parameter& iParameter);

    /**
     * @brief Read numeric launch parameter with given name ('iParamName') and default
     * Value ('iDefaultVal').
     *
     * If parameter value is below 'iMinVal' or above 'iMaxVal', the default value is set
     *
     * @return Parameter value.
     */
    template <typename T>
    T readNumericLaunchParameter(const rclcpp::Node* ipNode,
                                 const std::string& iParamName,
                                 const T& iDefaultVal,
                                 const T& iMinVal,
                                 const T& iMaxVal) const;

    /**
     * @brief Read string launch parameter with given name ('iParamName') and default
     * Value ('iDefaultVal').
     *
     * If parameter string is empty and default value is not empty, it is assumed that the parameter
     * is not allowed to be empty. In this case the default value will be set.
     *
     * @return Parameter value.
     */
    std::string readStringLaunchParameter(const rclcpp::Node* ipNode,
                                          const std::string& iParamName,
                                          const std::string& iDefaultVal = "") const;

    /**
     * @brief Reset calibration
     */
    virtual void reset();

    /**
     * @brief Shutdown subscribers and disconnect callbacks.
     *
     * @return True, if successful. False, otherwise.
     */
    virtual bool shutdownSubscribers() = 0;

    /**
     * @brief Save calibration.
     *
     * @return True, if successful. False, otherwise.
     */
    virtual bool saveCalibration() = 0;

  private:
    /**
     * @brief Load of calibration workspace.
     *
     * @return True, if successful.
     */
    bool loadCalibrationWorkspace();

    /**
     * @brief Load of robot workspace.
     *
     * @return True, if successful.
     */
    bool loadRobotWorkspace();

    /**
     * @brief Load of robot urdf model.
     *
     * @return True, if successful.
     */
    bool loadRobotUrdfModel();

    /**
     * @brief Service call to request finalization of calibration
     *
     * @param[in] ipReq Request, with flag to finalize calibration
     * @param[out] opRes Response, empty.
     */
    bool onRequestCalibrationFinalization(
      const std::shared_ptr<interf::srv::FinalizeCalibration::Request> ipReq,
      std::shared_ptr<interf::srv::FinalizeCalibration::Response> opRes);

    /**
     * @brief Service call to request capturing of calibration target
     *
     * @param[in] ipReq Request, with flag to capture calibration target
     * @param[out] opRes Response, empty.
     */
    bool onRequestTargetCapture(
      const std::shared_ptr<interf::srv::CaptureCalibTarget::Request> ipReq,
      std::shared_ptr<interf::srv::CaptureCalibTarget::Response> opRes);

    /**
     * @brief Service call to request reset of calibration
     *
     * @param[in] iReq Request, empty
     * @param[out] oRes Response.
     */
    bool onReset(
      const std::shared_ptr<interf::srv::ResetCalibration::Request> ipReq,
      std::shared_ptr<interf::srv::ResetCalibration::Response> opRes);

    /**
     * @brief Method to read robot specific settings from settings file within robot workspace.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool readRobotSettings();

    //--- MEMBER DECLARATION ---//

  protected:
    /// Type of calibration.
    ECalibrationType type_;

    /// Flag indicating if node is initialized.
    bool isInitialized_;

    /// Logger object of node.
    rclcpp::Logger logger_;

    /// Mutex guarding the data processing.
    std::mutex dataProcessingMutex_;

    /// Callback handle to adjust parameters
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr pParameterCallbackHandle_;

    /// TF buffer needed for listener
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;

    /// Transform listener to get transform between the two sensor frames.
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    /// Pointer to service to request capturing of calibration target.
    rclcpp::Service<interf::srv::CaptureCalibTarget>::SharedPtr pCaptureSrv_;

    /// Pointer to service to request finalization of calibration.
    rclcpp::Service<interf::srv::FinalizeCalibration>::SharedPtr pFinalizeSrv_;

    /// Pointer to service to reset calibration.
    rclcpp::Service<interf::srv::ResetCalibration>::SharedPtr pResetSrv_;

    /// Absolute path to robot workspace.
    fs::path robotWsPath_;

    /// Pointer to robot workspace.
    std::shared_ptr<AbstractWorkspace> pRobotWs_;

    /// Name of robot.
    std::string robotName_;

    /// Flag to indicate if URDF model is available
    bool isUrdfModelAvailable_;

    /// absolute path to URDF model file of robot.
    fs::path urdfModelPath_;

    /// XML document of URDF model.
    tinyxml2::XMLDocument urdfModelDoc_;

    /// URDF model of robot.
    urdf::Model urdfModel_;

    /// Flag controlling whether the observations are to be saved into the calibration workspace
    /// after calibration.
    bool saveObservationsToWs_;

    /// Pointer to base calibration workspace. The instantiation of the pointer is done in the
    /// individual calibration node.
    std::shared_ptr<AbstractWorkspace> pCalibrationWs_;

    /// Path to target configuration file
    fs::path calibTargetFilePath_;

    /// Flag to capture calibration target in next sensor data package.
    bool captureCalibrationTarget_;

    /// Iteration number of the calibration routine.
    uint calibrationItrCnt_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_CALIBRATIONBASE_H