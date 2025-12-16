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
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <rclcpp/node.hpp>

// multisensor_calibration
#include "Extrinsic3d3dCalibrationBase.h"
#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/config/LidarLidarRegistrationParameters.hpp"
#include "multisensor_calibration/config/LidarTargetDetectionParameters.hpp"
#include "multisensor_calibration/sensor_data_processing/LidarDataProcessor.h"

namespace multisensor_calibration
{

/**
 * @brief Node to perform extrinsic lidar-lidar calibration.
 *
 * This subclasses multisensor_calibration::Extrinsic3d3dCalibrationBase.
 */
class ExtrinsicLidarLidarCalibration
  : public Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>,
    public rclcpp::Node
{

    //==============================================================================
    // TYPEDEFS
    //==============================================================================
  protected:
    typedef message_filters::sync_policies::ApproximateTime<
      InputCloud_Message_T, InputCloud_Message_T>
      CloudCloudApproxSync;

    typedef message_filters::sync_policies::ExactTime<
      InputCloud_Message_T, InputCloud_Message_T>
      CloudCloudExactSync;

    //==============================================================================
    // CONSTRUCTION / DESTRUCTION
    //==============================================================================
  public:
    ExtrinsicLidarLidarCalibration(const std::string& nodeName,
                                   const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ExtrinsicLidarLidarCalibration(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~ExtrinsicLidarLidarCalibration() override;

    //==============================================================================
    // METHODS
    //==============================================================================
    /**
     * @brief Run the extrinsic calibration based on the last observation of the calibration target.
     *
     * This will remove observations without correspondence and estimate a rigid transformation
     * based on the detected marker corners.
     */
    void calibrateLastObservation();

    /**
     * @brief Method to receive synchronized sensor data, i.e. LiDAR point clouds from the source
     * and the reference sensor.
     *
     * This calls the processing of the data data processors, i.e. detect the calibration target
     * or possible candidates, depending on wether the command to capture the target is triggered or
     * not. When the target is detected a calibration with the last observation is performed. In
     * this, the observations might be rejected if the error of the calibration is too large.
     *
     * @param[in] ipSrcCloudMsg Pointer to point cloud message from source sensor that is to be
     * calibrated.
     * @param[in] ipRefCloudMsg Pointer to point cloud message from reference sensor against which
     * the source sensor is to be calibrated.
     */
    void onSensorDataReceived(
      const InputCloud_Message_T::ConstSharedPtr& ipSrcCloudMsg,
      const InputCloud_Message_T::ConstSharedPtr& ipRefCloudMsg);
    //==============================================================================
    // METHODS: Overrides from parent
    //==============================================================================
  private:
    bool finalizeCalibration() override;

    bool initializeDataProcessors() override;

    bool initializeSubscribers(rclcpp::Node* ipNode) override;

    bool initializeWorkspaceObjects() override;

    bool onRequestRemoveObservation(
      const std::shared_ptr<interf::srv::RemoveLastObservation::Request> ipReq,
      std::shared_ptr<interf::srv::RemoveLastObservation::Response> opRes) override;

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
    /// Object holding parameters for the lidar lidar registration
    LidarLidarRegistrationParameters registrationParams_;

    /// Object holding parameters for the target detection in LiDAR data
    LidarTargetDetectionParameters lidarTargetDetectionParams_;

    /// message filter for approximated message synchronization for cloud message data
    std::shared_ptr<message_filters::Synchronizer<CloudCloudApproxSync>> pCloudCloudApproxSync_;

    /// message filter for exact message synchronization for cloud message data
    std::shared_ptr<message_filters::Synchronizer<CloudCloudExactSync>> pCloudCloudExactSync_;

    /// Subscriber to point cloud from source sensor
    message_filters::Subscriber<InputCloud_Message_T> srcCloudSubsc_;

    /// Subscriber to point cloud from reference sensor
    message_filters::Subscriber<InputCloud_Message_T> refCloudSubsc_;

    /// Flag to activate the additional alignment of the ground planes.
    bool alignGroundPlanes_;

    /// ID of a frame that has an upright z-axis to find the ground planes for the alignment.
    std::string uprightFrameId_;

    /// Queue size for synchronization of image messages and point cloud
    int syncQueueSize_;

    /// Flag to activate exact time synchronization
    bool useExactSync_;
};

} // namespace multisensor_calibration
