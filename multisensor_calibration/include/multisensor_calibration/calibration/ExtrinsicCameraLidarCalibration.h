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
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <rclcpp/node.hpp>
#include <tf2/LinearMath/Transform.hpp>

// PCL
#include <pcl/filters/frustum_culling.h>

// multisensor_calibration
#include "Extrinsic2d3dCalibrationBase.h"
#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/config/CameraLidarRegistrationParameters.hpp"
#include "multisensor_calibration/config/LidarTargetDetectionParameters.hpp"
#include <multisensor_calibration_interface/srv/camera_intrinsics.hpp>

#include "multisensor_calibration/sensor_data_processing/CameraDataProcessor.h"
#include "multisensor_calibration/sensor_data_processing/LidarDataProcessor.h"

namespace multisensor_calibration
{

/**
 * @brief Node to perform extrinsic camera-lidar calibration.
 *
 * This subclasses multisensor_calibration::Extrinsic2d3dCalibrationBase.
 */
class ExtrinsicCameraLidarCalibration
  : public Extrinsic2d3dCalibrationBase<CameraDataProcessor, LidarDataProcessor>,
    public rclcpp::Node
{

    //==============================================================================
    // TYPEDEFS
    //==============================================================================
  protected:
    using Extrinsic2d3dCalibrationBase<CameraDataProcessor, LidarDataProcessor>::ImgCloudApproxSync;
    using Extrinsic2d3dCalibrationBase<CameraDataProcessor, LidarDataProcessor>::ImgCloudExactSync;

    //==============================================================================
    // CONSTRUCTION / DESTRUCTION
    //==============================================================================
  public:
    ExtrinsicCameraLidarCalibration(const std::string& nodeName,
                                    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ExtrinsicCameraLidarCalibration(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~ExtrinsicCameraLidarCalibration() override;

    //==============================================================================
    // METHODS
    //==============================================================================
  private:
    /**
     * @brief Run the extrinsic calibration based on the last observation of the calibration target.
     *
     *
     * This will remove observations without correspondence and estimate a rigid transformation
     * based on the detected marker corners using Pespective-n-Point (PnP).
     */
    void calibrateLastObservation();

    /**
     * @brief Configures frustum culling filter and applies it to pLidarDataProcessor_.
     *
     * The frustum culling filter is used to reduce the size of the point cloud in which to search
     * for the calibration target.
     */
    void configureAndApplyFrustumCulling();

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
     * @brief Method to receive synchronized sensor data, i.e. camera image and LiDAR point cloud.
     *
     * This calls the processing of the data data processors, i.e. detect the calibration target
     * or possible candidates, depending on wether the command to capture the target is triggered or
     * not. When the target is detected a calibration with the last observation is performed. In
     * this, the observations might be rejected if the error of the calibration is too large.
     *
     * @param[in] ipImgMsg Pointer to vis image message.
     * @param[in] ipCloudMsg Pointer to point cloud message.
     */
    void onSensorDataReceived(
      const InputImage_Message_T::ConstSharedPtr& ipImgMsg,
      const InputCloud_Message_T::ConstSharedPtr& ipCloudMsg);

    //==============================================================================
    // METHODS: Overrides from parent
    //==============================================================================
    bool finalizeCalibration() override;

    bool initializeDataProcessors() override;

    bool initializeServices(rclcpp::Node* ipNode) override;

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
    /// Object holding parameters for the camera lidar registration
    CameraLidarRegistrationParameters registrationParams_;

    /// Object holding parameters for the target detection in LiDAR data
    LidarTargetDetectionParameters lidarTargetDetectionParams_;

    /// Service to get camera intrinsics
    rclcpp::Service<interf::srv::CameraIntrinsics>::SharedPtr pCameraIntrSrv_;

    /// message filter for approximated message synchronization for image and cloud message data
    std::shared_ptr<message_filters::Synchronizer<ImgCloudApproxSync>> pImgCloudApproxSync_;

    /// message filter for exact message synchronization for image and cloud message data
    std::shared_ptr<message_filters::Synchronizer<ImgCloudExactSync>> pImgCloudExactSync_;

    /// Subscriber to image topic
    image_transport::SubscriberFilter imageSubsc_;

    /// Subscriber to point cloud
    message_filters::Subscriber<InputCloud_Message_T> cloudSubsc_;

    /// Queue size for synchronization of image messages and point cloud
    int syncQueueSize_;

    /// Flag to activate exact time synchronization
    bool useExactSync_;

    /// Vector of Pointers to FrustumCulling filter used for pre-processing of the lidar data.
    std::vector<pcl::FrustumCulling<InputPointType>::Ptr> pFrustumCullingFilters_;
};

} // namespace multisensor_calibration
