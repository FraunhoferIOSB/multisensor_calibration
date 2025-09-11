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

#ifndef MULTISENSORCALIBRATION_DATAPROCESSOR3D_H
#define MULTISENSORCALIBRATION_DATAPROCESSOR3D_H

// ROS
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <tf2/LinearMath/Transform.h>

// PCL
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// multisensor_calibration
#include "../common/common.h"
#include "SensorDataProcessorBase.h"
#include <multisensor_calibration/common/lib3D/core/extrinsics.hpp>
#include <multisensor_calibration_interface/srv/add_marker_observations.hpp>
#include <multisensor_calibration_interface/srv/import_marker_observations.hpp>

namespace multisensor_calibration
{

/**
 * @ingroup  target_detection
 * @brief Base class for processing the 3D sensor data in the form of a 3D point cloud.
 *
 * This subclasses the base class SensorDataProcessorBase with pcl::PointCloud as template
 * specialization.
 */
class DataProcessor3d : public SensorDataProcessorBase<pcl::PointCloud<InputPointType>>
{

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Default constructor is deleted.
     *
     */
    DataProcessor3d() = delete;

    /**
     * @brief Initialization constructor, providing the node name, the sensor name as well
     * as the path to the calibration target configuration file.
     */
    DataProcessor3d(const std::string& iLoggerName,
                    const std::string& iSensorName,
                    const fs::path& iCalibTargetFilePath);

    /**
     * @brief Destructor
     */
    virtual ~DataProcessor3d();

    /**
     * @brief Method to set pointer to transform that will transform the data into a common frame.
     *
     * @param[in] pTransform Pointer to Transform object.
     */
    void setDataTransform(const std::shared_ptr<tf2::Transform>& pTransform);

    /**
     * @brief Get copy of list holding pointers to point clouds of detected calibration boards.
     */
    std::vector<pcl::PointCloud<InputPointType>::Ptr> getCalibrationTargetCloudPtrs() const;

    /**
     * @brief Get copy of pointer to last point cloud of detected calibration board.
     */
    pcl::PointCloud<InputPointType>::Ptr getLastCalibrationTargetCloud() const;

    /**
     * @brief Get copy of pointer to last input point cloud.
     */
    pcl::PointCloud<InputPointType>::Ptr getLastInputCloud() const;

    /**
     * @brief Get list of last estimated marker corners corresponding to detected marker IDs
     */
    std::vector<std::array<cv::Point3f, 4>> getLastEstimatedMarkerCorners() const;

    /**
     * @brief Get list of last estimated marker IDs.
     */
    std::vector<uint> getLastEstimatedMarkerIds() const;

    /**
     * @brief Get pointer to 3D cloud holding the estimated marker corner positions.
     * Each point holds the ID of the corresponding marker in the intensity value.
     * the corners are stored in a clockwise direction starting from top-left corner.
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr getLastMarkerCornersCloud() const;

    /**
     * @brief Get observations with unique IDs and in ascending order
     *
     * @param[out] oObservationIds Ordered set holding the IDs of 3D point observations. The
     * observations of the marker corners are associated with the marker IDs. And for each iteration
     * the ID is added to a corresponding multiple of hundred (i.e. Iteration 1 + ID 1 = 101,
     * Iteration 2 + ID 1 = 201).
     * @param[out] oCornerObservations List holding the actual observations of the marker corners
     * in the order as stored in oObservationIds. Since for each marker four corners (clockwise from
     * top-left) are observed, the size of oCornerPoints is 4x the size of oObservationIds.
     * @param[in] iIterationBegin Iteration number from which to begin the extraction.
     * @param[in] iNumIterations Number of iterations for which the observations are to be
     * extracted. If set to -1, all iterations between iTerationBegin and end will be included.
     */
    void getOrderedObservations(std::set<uint>& oObservationIds,
                                std::vector<cv::Point3f>& oCornerObservations,
                                const int& iIterationBegin = 1,
                                const int& iNumIterations  = -1) const;

    /**
     * @brief Get Pointer to cloud holding all region of interests.
     */
    pcl::PointCloud<InputPointType>::Ptr getRegionOfInterestsCloudPtr() const;

    /**
     * @brief Get sensor data from ros message.
     *
     * @param[in] ipMsg Pointer to input message.
     * @param[out] oData Sensor data.
     */
    bool getSensorDataFromMsg(const InputCloud_Message_T::ConstSharedPtr& ipMsg,
                              pcl::PointCloud<InputPointType>& oData) const;

    /**
     * @brief Method to initialize publishers
     *
     * @param[in, out] ipNode Pointer to node to which the service is to be initialized.
     * @return True, if successful. False otherwise.
     */
    virtual bool initializePublishers(rclcpp::Node* ipNode) override;

    /**
     * @brief Method to initialize services
     *
     * @param[in, out] ipNode Pointer to node to which the service is to be initialized.
     * @return True, if successful. False otherwise.
     */
    virtual bool initializeServices(rclcpp::Node* ipNode) override;

    /**
     * @overload
     * @brief Method to publish preview data, i.e. the segmented region of interest.
     *
     * @param[in] iHeader Header for the ROS message that is to be published.
     */
    void publishPreview(const std_msgs::msg::Header& iHeader) const override;

    using SensorDataProcessorBase::publishPreview;

    /**
     * @overload
     * @brief Method to publish data of successful target detection.
     *
     * @param[in] iHeader Header for the ROS message that is to be published.
     */
    void publishLastTargetDetection(const std_msgs::msg::Header& iHeader) const override;

    using SensorDataProcessorBase::publishLastTargetDetection;

    /**
     * @brief Remove observations corresponding to given calibration iteration.
     *
     * @param[in] iIterationId Iteration ID for which the observations are to be removed.
     * @return True, if successful. False, otherwise (i.e. if the specified iteration is not
     * available).
     */
    bool removeCalibIteration(const uint& iIterationId) override;

    /**
     * @brief Reset processor.
     */
    void reset() override;

    /**
     * @brief Save the observations to given output directory.
     *
     * @param[in] iOutputDir Path to output directory.
     * @return True if successful, false otherwise.
     */
    bool saveObservations(const fs::path iOutputDir) const override;

  protected:
    /**
     * @brief Service call to handle adding of reference marker positions
     *
     * @param[in] iReq Request.
     * @param[out] oRes Response.
     */
    bool onAddMarkerObservations(
      const std::shared_ptr<multisensor_calibration_interface::srv::AddMarkerObservations::Request> iReq,
      std::shared_ptr<multisensor_calibration_interface::srv::AddMarkerObservations::Response> oRes);

    /**
     * @brief Service call to handle import of reference marker positions
     *
     * @param[in] iReq Request.
     * @param[out] oRes Response.
     */
    bool onImportMarkerObservations(
      const std::shared_ptr<multisensor_calibration_interface::srv::ImportMarkerObservations::Request> iReq,
      std::shared_ptr<multisensor_calibration_interface::srv::ImportMarkerObservations::Response> oRes);

    //--- FUNCTION DECLARATION ---//

  public:
    /**
     * @brief Write observations of marker corners into file.
     *
     * @param[in] iFilePath Path to output file.
     * @param[in] iMarkerIds List of marker ids.
     * @param[in] iMarkerCorners List of marker corners. For each marker (corresponding to marker
     * ID) there are 4 corners with 3D coordinates.
     */
    static void writeMarkerObservationsToFile(
      const fs::path& iFilePath,
      const std::vector<uint>& iMarkerIds,
      const std::vector<std::array<cv::Point3f, 4>>& iMarkerCorners);

    /**
     * @brief Read observations of marker corners from file.
     *
     * @param[in] iFilePath Path to output file.
     * @param[out] oMarkerIds List of marker ids.
     * @param[out] oMarkerCorners List of marker corners. For each marker (corresponding to marker
     * ID) there are 4 corners with 3D coordinates.
     * @return True if successful, false otherwise (e.g. if file not exists).
     */
    static bool readMarkerObservationsFromFile(
      const fs::path& iFilePath,
      std::vector<uint>& oMarkerIds,
      std::vector<std::array<cv::Point3f, 4>>& oMarkerCorners);

    //--- MEMBER DECLARATION ---//

  protected:
    /// Pointer to transform object which can be used transform the Lidar data prior to processing
    /// or the detected data points in post-processing.
    std::shared_ptr<tf2::Transform> pDataTransform_;

    /// Pointer to point cloud holding the regions of interest for publishing
    pcl::PointCloud<InputPointType>::Ptr pRoisCloud_;

    /// List of pointers to input point clouds in which the target has been detected.
    std::vector<pcl::PointCloud<InputPointType>::Ptr> inputCloudPtrs_;

    /// List of pointers to point cloud of detected calibration board.
    std::vector<pcl::PointCloud<InputPointType>::Ptr> calibrationTargetCloudPtrs_;

    /// List of captured marker IDs per iteration.
    std::vector<std::vector<uint>> estimatedMarkerIds_;

    /// List of captured marker corners corresponding to detected marker IDs
    std::vector<std::vector<std::array<cv::Point3f, 4>>> estimatedMarkerCorners_;

    /// List of pointers to 3D cloud holding the estimated marker corner positions.
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> estimatedMarkerCornersCloudPtrs_;

    /// Pointer to service to add marker observations
    rclcpp::Service<multisensor_calibration_interface::srv::AddMarkerObservations>::SharedPtr
      pAddMarkerObsSrv_;

    /// Pointer to service to import marker observations
    rclcpp::Service<multisensor_calibration_interface::srv::ImportMarkerObservations>::SharedPtr
      pImportMarkerObsSrv_;

    /// Publisher for regions of interest in which a search for the calibration target is performed
    rclcpp::Publisher<RoisCloud_Message_T>::SharedPtr pRoisCloudPub_;

    /// Publisher for target point cloud
    rclcpp::Publisher<TargetPatternCloud_Message_T>::SharedPtr pTargetCloudPub_;

    /// Publisher for corners of marker positions
    rclcpp::Publisher<MarkerCornerCloud_Message_T>::SharedPtr pMarkerCornersPub_;

    /// Publisher for target board pose
    rclcpp::Publisher<TargetBoardPose_Message_T>::SharedPtr pTargetBoardPosePub_;
};

} // namespace multisensor_calibration

#endif