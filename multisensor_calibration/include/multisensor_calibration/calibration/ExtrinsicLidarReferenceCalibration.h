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

#ifndef MULTISENSORCALIBRATION_EXTRINSICLIDARREFERENCECALIBRATION_H
#define MULTISENSORCALIBRATION_EXTRINSICLIDARREFERENCECALIBRATION_H

// Std
#include <memory>
#include <string>

// ROS
#include <message_filters/subscriber.hpp>
#include <rclcpp/node.hpp>

// multisensor_calibration
#include "../common/common.h"
#include "../config/LidarReferenceRegistrationParameters.hpp"
#include "../config/LidarTargetDetectionParameters.hpp"
#include "../sensor_data_processing/LidarDataProcessor.h"
#include "../sensor_data_processing/ReferenceDataProcessor3d.h"
#include "Extrinsic3d3dCalibrationBase.h"

namespace multisensor_calibration
{

/**
 * @brief Node to perform extrinsic lidar-reference calibration.
 *
 * This subclasses multisensor_calibration::Extrinsic3d3dCalibrationBase.
 */
class ExtrinsicLidarReferenceCalibration
  : public Extrinsic3d3dCalibrationBase<LidarDataProcessor, ReferenceDataProcessor3d>,
    public rclcpp::Node
{

    //--- METHOD DECLARATION ---/
  public:
    /**
     * @brief Initialization constructor
     */
    ExtrinsicLidarReferenceCalibration(const std::string& nodeName,
                                       const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    /**
     * @brief Initialization constructor
     */
    ExtrinsicLidarReferenceCalibration(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Destructor
     */
    virtual ~ExtrinsicLidarReferenceCalibration();

  private:
    using CalibrationBase::handleDynamicParameterChange;

    /**
     * @brief Method to finalize calibration. This overrides the method of the parent class.
     *
     * This will calibrate the extrinsic pose based on all observations in the list and print the
     * final error and print out the result of the calibration. In this, the isolated clouds of the
     * detected calibration target are aligned using GICP and, in turn, the extrinsic 6DOF pose
     * between the sensors is calculated.
     */
    bool finalizeCalibration() override;

    /**
     * @brief Method to initialize data processing. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * This will initialize the data processors, call the initialization of the publishers within
     * the data processors and subscribe to the corresponding data topics.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool initializeDataProcessors() override;

    /**
     * @brief Method to initialize subscribers. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool initializeSubscribers(rclcpp::Node* ipNode) override;

    /**
     * @brief Method to initialize workspace objects. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * In this class, the object of the calibration
     * workspace is initialized. The initialization requires the launch parameters, thus it is to
     * be executed after the launch parameters are read.
     *
     * @return True if successful. False, otherwise (e.g. if instantiation has failed)
     */
    bool initializeWorkspaceObjects() override;

    /**
     * @brief Method to sensor data, i.e. LiDAR point clouds from the source sensor.
     *
     * This calls the processing of the data data processors, i.e. detect the calibration target
     * or possible candidates, depending on wether the command to capture the target is triggered or
     * not. When the target is detected a calibration with the last observation is performed. In
     * this, the observations might be rejected if the error of the calibration is too large.
     *
     * @param[in] ipSrcCloudMsg Pointer to point cloud message from source sensor that is to be
     * calibrated.
     */
    void onSensorDataReceived(const InputCloud_Message_T::ConstSharedPtr& ipSrcCloudMsg);

    /**
     * @brief Method to save calibration specific settings to the workspace. This overrides the
     * method of the parent class.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool saveCalibrationSettingsToWorkspace() override;

    /**
     * @brief Setup launch parameters.
     *
     * The implementation within this class hold launch parameters that are common to all
     * calibration nodes.
     *
     * @param[in] ipNode Pointer to node.
     */
    void setupLaunchParameters(rclcpp::Node* ipNode) const override;

    /**
     * @brief Setup dynamic parameters.
     *
     * @param[in] ipNode Pointer to node.
     */
    void setupDynamicParameters(rclcpp::Node* ipNode) const override;

    /**
     * @brief Method to read launch parameters. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * @param[in] iNh Object of node handle
     * @return True if successful. False, otherwise (e.g. if sanity check fails)
     */
    bool readLaunchParameters(const rclcpp::Node* ipNode) override;

    /**
     * @brief Virtual function to set dynamic parameter. This is called from
     * handleDynamicParameterChange for each parameter in the list that is to be changed.
     *
     * @param[in] iParameter Parameter that is to be changed.
     * @return True, if successful, i.e. if it has been changed. False, otherwise.
     */
    bool setDynamicParameter(const rclcpp::Parameter& iParameter) override;

    /**
     * @brief Method to reset calibration. This overrides the method of the parent class.
     * In this, the parent method is also called.
     */
    void reset() override;

    /**
     * @brief Method to shutdown subscribers and disconnect callbacks. This overrides the method of
     * the parent class. In this, the parent method is also called.
     *
     * @return True, if successful. False, otherwise.
     */
    bool shutdownSubscribers() override;

    //--- MEMBER DECLARATION ---/

  private:
    /// Object holding parameters for the registration algorithm
    LidarReferenceRegistrationParameters registrationParams_;

    /// Object holding parameters for the target detection in LiDAR data
    LidarTargetDetectionParameters lidarTargetDetectionParams_;

    /// Subscriber to point cloud from source sensor
    rclcpp::Subscription<InputCloud_Message_T>::SharedPtr pSrcCloudSubsc_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_EXTRINSICLIDARREFERENCECALIBRATION_H
