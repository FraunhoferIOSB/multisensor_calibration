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

#ifndef MULTISENSORCALIBRATION_EXTRINSICLIDARVEHICLECALIBRATION_H
#define MULTISENSORCALIBRATION_EXTRINSICLIDARVEHICLECALIBRATION_H

// Std
#include <memory>
#include <string>

// ROS
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2/LinearMath/Transform.hpp>

// OpenCV
#include <opencv2/core.hpp>

// multisensor_calibration
#include "../common/common.h"
#include "../config/LidarVehicleRegistrationParameters.hpp"
#include "../sensor_data_processing/LidarDataProcessor.h"
#include "Extrinsic3d3dCalibrationBase.h"
#include <multisensor_calibration_interface/srv/add_region_marker.hpp>

// #define USE_NORMAL_INFO

namespace multisensor_calibration
{

/**
 * @brief Node to perform extrinsic lidar-vehicle calibration.
 *
 * This subclasses multisensor_calibration::Extrinsic3d3dCalibrationBase.
 */
class ExtrinsicLidarVehicleCalibration
  : public Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>,
    public rclcpp::Node
{

#ifdef USE_NORMAL_INFO
    using RegionPointType = pcl::PointXYZINormal;
#else
    using RegionPointType = InputPointType;
#endif

    //--- METHOD DECLARATION ---/
  public:
    /**
     * @brief Initialization constructor
     */
    ExtrinsicLidarVehicleCalibration(const std::string& nodeName,
                                     const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    /**
     * @brief Initialization constructor
     */
    ExtrinsicLidarVehicleCalibration(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Destructor
     */
    ~ExtrinsicLidarVehicleCalibration() override = default;

  private:
    using CalibrationBase::handleDynamicParameterChange;

    /**
     * @brief Compute planar regions in cloud data based on selected region markers
     *
     * @param[in] ipCloudMsg Pointer to cloud message from which the regions cloud is to be computed.
     * @param[in] iRegionMarkers List of markers for which regions should be computed.
     * @param[out] oRegionSeedPointsPtrs List of point clouds holding the seed points for the
     * selected regions.
     * @param[out] opRegionsCloud Pointer to cloud which is to hold the regions.
     */
    void computeRegionsCloud(const InputCloud_Message_T::ConstSharedPtr& ipCloudMsg,
                             const std::vector<InputPointType>& iRegionMarkers,
                             std::vector<pcl::PointCloud<InputPointType>::Ptr>& oRegionSeedPointsPtrs,
                             pcl::PointCloud<RegionPointType>::Ptr& opRegionsCloud) const;

    /**
     * @brief Do coarse calibration based on the seed points of the regions of interest.
     *
     * This will estimate a rigid transformation between the selected seed points.
     */
    void doCoarseCalibration();

    /**
     * @brief Method to initialize subscribers. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * @return True, if all settings are valid. False, otherwise.
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
    bool initializeDataProcessors() override
    {
        return true;
    };

    /**
     * @brief Method to initialize publishers. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool initializePublishers(rclcpp::Node* ipNode) override;

    /**
     * @brief Method to initialize services. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool initializeServices(rclcpp::Node* ipNode) override;

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
     * @brief Handle point clicked in RViz
     *
     * @param[in] ipPointMsg Pointer to message holding the clicked point.
     */
    void onPointClicked(const geometry_msgs::msg::PointStamped::ConstSharedPtr& ipPointMsg);

    /**
     * @brief handle service call to request addition of region marker.
     *
     * @param[in] ipReq Request
     * @param[out] opRes Response
     */
    bool onRequestAddRegionMarker(
      const std::shared_ptr<interf::srv::AddRegionMarker::Request> ipReq,
      std::shared_ptr<interf::srv::AddRegionMarker::Response> opRes);

    /**
     * @brief Handle service call to request removing of last observation.
     *
     * @param[in] ipReq Request, with flag to capture calibration target
     * @param[out] opRes Response, empty.
     */
    bool onRequestRemoveObservation(
      const std::shared_ptr<interf::srv::RemoveLastObservation::Request> ipReq,
      std::shared_ptr<interf::srv::RemoveLastObservation::Response> opRes) override;

    /**
     * @brief Method to receive reference data from vehicle model.
     *
     * @param[in] ipRefCloudMsg Pointer to point cloud message from reference model.
     */
    void onReferenceDataReceived(
      const InputCloud_Message_T::ConstSharedPtr& ipRefCloudMsg);

    /**
     * @brief Method to receive sensor data from source LiDAR which is to be calibrated with respect
     * to the vehicle.
     *
     * @param[in] ipSrcCloudMsg Pointer to point cloud message from source sensor that is to be
     * calibrated.
     */
    void onSensorDataReceived(
      const InputCloud_Message_T::ConstSharedPtr& ipSrcCloudMsg);

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
    LidarVehicleRegistrationParameters registrationParams_;

    /// Mutex guarding the processing of sensor data.
    /// This is a reference to CalibrationBase::dataProcessingMutex_
    std::mutex& sensorDataProcessingMutex_;

    /// Mutex guarding the processing of reference data.
    std::mutex refDataProcessingMutex_;

    /// Subscriber to clicked_point of rviz
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pClickedPointSubsc_;

    /// Server to provide service to request adding of region marker
    rclcpp::Service<interf::srv::AddRegionMarker>::SharedPtr pRegionMarkerSrv_;

    /// Subscriber to point cloud topic of sensor to be calibrated
    rclcpp::Subscription<InputCloud_Message_T>::SharedPtr pSrcCloudSubsc_;

    /// Subscriber to point cloud topic of reference model
    rclcpp::Subscription<InputCloud_Message_T>::SharedPtr pRefCloudSubsc_;

    /// Publisher for regions in source cloud selected by marker and used for calibration
    rclcpp::Publisher<RoisCloud_Message_T>::SharedPtr pSrcRegionPub_;

    /// Publisher for regions in ref cloud selected by marker and used for calibration
    rclcpp::Publisher<RoisCloud_Message_T>::SharedPtr pRefRegionPub_;

    /// List of 3D points marking region correspondences in the cloud of the source lidar.
    std::vector<InputPointType> srcRegionMarkers_;

    /// List of point clouds holding the seed points for the selected regions in source cloud.
    std::vector<pcl::PointCloud<InputPointType>::Ptr> srcRegionSeedCloudPtrs_;

    /// Pointer to point cloud holding the regions of source sensor used for calibration
    pcl::PointCloud<RegionPointType>::Ptr pSrcRegionsCloud_;

    /// List of 3D points marking region correspondences in the reference data.
    std::vector<InputPointType> refRegionMarkers_;

    /// List of point clouds holding the seed points for the selected regions in reference.
    std::vector<pcl::PointCloud<InputPointType>::Ptr> refRegionSeedCloudPtrs_;

    /// Pointer to point cloud holding the regions of reference data used for calibration
    pcl::PointCloud<RegionPointType>::Ptr pRefRegionsCloud_;

    /// History of region markers added to list. This is used to remove last marker
    std::vector<std::string> regionMarkerHistory_;

    /// Pointer to transform object which will transform the reference Lidar data prior to processing.
    std::shared_ptr<tf2::Transform> pRefDataTransform_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_EXTRINSICLIDARVEHICLECALIBRATION_H
