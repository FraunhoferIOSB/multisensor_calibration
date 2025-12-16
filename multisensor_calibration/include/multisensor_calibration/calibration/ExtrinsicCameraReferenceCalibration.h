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
#include <memory>
#include <string>

// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/node.hpp>
#include <tf2/LinearMath/Transform.hpp>

// PCL
#include <pcl/filters/frustum_culling.h>

// multisensor_calibration
#include "../common/common.h"
#include "../config/CameraReferenceRegistrationParameters.hpp"
#include "multisensor_calibration/calibration/Extrinsic2d3dCalibrationBase.h"
#include "multisensor_calibration/sensor_data_processing/CameraDataProcessor.h"
#include "multisensor_calibration/sensor_data_processing/ReferenceDataProcessor3d.h"
#include <multisensor_calibration_interface/srv/camera_intrinsics.hpp>

namespace multisensor_calibration
{

/**
 * @brief Node to perform extrinsic camera-reference calibration.
 *
 * This subclasses multisensor_calibration::Extrinsic2d3dCalibrationBase.
 *
 */
class ExtrinsicCameraReferenceCalibration
  : public Extrinsic2d3dCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>,
    public rclcpp::Node
{

    //==============================================================================
    // CONSTRUCTION / DESTRUCTION
    //==============================================================================
  public:
    ExtrinsicCameraReferenceCalibration(const std::string& nodeName,
                                        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ExtrinsicCameraReferenceCalibration(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~ExtrinsicCameraReferenceCalibration() override;

  private:
    //==============================================================================
    // METHODS
    //==============================================================================

    /**
     * @brief Handle service call to get camera intrinsics.
     *
     * @param[in] ipReq Request, UNUSED.
     * @param[out] opRes Response.
     */
    bool onRequestCameraIntrinsics(
      const std::shared_ptr<interf::srv::CameraIntrinsics::Request> ipReq,
      std::shared_ptr<interf::srv::CameraIntrinsics::Response> opRes);

    /**
     * @brief Method to receive sensor data, i.e. camera image from the source sensor.
     *
     * This calls the processing of the data data processors, i.e. detect the calibration target
     * or possible candidates, depending on wether the command to capture the target is triggered or
     * not. When the target is detected a calibration with the last observation is performed. In
     * this, the observations might be rejected if the error of the calibration is too large.
     *
     * @param[in] ipImgMsg Pointer to vis image message.
     */
    void onSensorDataReceived(const InputImage_Message_T::ConstSharedPtr& ipImgMsg);

    //==============================================================================
    // METHODS: Overrides from parent
    //==============================================================================
    bool finalizeCalibration() override;

    bool initializeDataProcessors() override;

    bool initializeServices(rclcpp::Node* ipNode) override;

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
    /// Object holding parameters for the camera lidar registration
    CameraReferenceRegistrationParameters registrationParams_;

    /// Service to get camera intrinsics
    rclcpp::Service<interf::srv::CameraIntrinsics>::SharedPtr pCameraIntrSrv_;

    /// Subscriber to image topic
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pImageSubsc_;
};

} // namespace multisensor_calibration
