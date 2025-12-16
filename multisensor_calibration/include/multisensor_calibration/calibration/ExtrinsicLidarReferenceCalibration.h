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

#pragma once

// Std
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

    //==============================================================================
    // CONSTRUCTION / DESTRUCTION
    //==============================================================================
  public:
    ExtrinsicLidarReferenceCalibration(const std::string& nodeName,
                                       const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ExtrinsicLidarReferenceCalibration(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~ExtrinsicLidarReferenceCalibration() override;

    //==============================================================================
    // METHODS
    //==============================================================================
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

    //==============================================================================
    // METHODS: Overrides from parent
    //==============================================================================
  private:
    bool finalizeCalibration() override;

    bool initializeDataProcessors() override;

    bool initializeSubscribers(rclcpp::Node* ipNode) override;

    bool initializeWorkspaceObjects() override;

    bool saveCalibrationSettingsToWorkspace() override;

    void setupLaunchParameters(rclcpp::Node* ipNode) const override;

    void setupDynamicParameters(rclcpp::Node* ipNode) const override;

    bool readLaunchParameters(const rclcpp::Node* ipNode) override;

    bool setDynamicParameter(const rclcpp::Parameter& iParameter) override;

    void reset() override;

    bool shutdownSubscribers() override;

    //==============================================================================
    // MEMBERS
    //==============================================================================
  private:
    /// Object holding parameters for the registration algorithm
    LidarReferenceRegistrationParameters registrationParams_;

    /// Object holding parameters for the target detection in LiDAR data
    LidarTargetDetectionParameters lidarTargetDetectionParams_;

    /// Subscriber to point cloud from source sensor
    rclcpp::Subscription<InputCloud_Message_T>::SharedPtr pSrcCloudSubsc_;
};

} // namespace multisensor_calibration
