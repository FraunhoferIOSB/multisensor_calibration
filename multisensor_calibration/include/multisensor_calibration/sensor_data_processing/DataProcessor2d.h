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

#ifndef MULTISENSORCALIBRATION_DATAPROCESSOR2D_H
#define MULTISENSORCALIBRATION_DATAPROCESSOR2D_H

// Std
#include <array>

// ROS
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// multisensor_calibration
#include "../common/common.h"
#include "SensorDataProcessorBase.h"

namespace multisensor_calibration
{

/**
 * @ingroup  target_detection
 * @brief Base class for processing the 2D sensor data in the form of a images.
 *
 * This subclasses the base class SensorDataProcessorBase with cv::Mat as template specialization.
 */
class DataProcessor2d : public SensorDataProcessorBase<cv::Mat>
{
    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Default constructor is deleted.
     *
     */
    DataProcessor2d() = delete;

    /**
     * @brief Initialization constructor, providing the node name, the sensor name as well
     * as the path to the calibration target configuration file.
     */
    DataProcessor2d(const std::string& iLoggerName,
                    const std::string& iSensorName,
                    const fs::path& iCalibTargetFilePath);

    /**
     * @brief Destructor
     */
    virtual ~DataProcessor2d();

    /**
     * @brief Get copy of last camera image annotated with detected markers as preview.
     *
     * This will only annotate the borders of the markers, not the corners them selfs.
     */
    cv::Mat getAnnotatedCameraImagePreview() const;

    /**
     * @brief Get copy of pointer to point cloud of last detected calibration board.
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getLastCalibrationTargetCloudPtr() const;

    /**
     * @brief Get list of last detected marker corners corresponding to detected marker IDs
     * extracted with getLastCapturedMarkerIds().
     */
    std::vector<std::array<cv::Point2f, 4>> getLastCapturedMarkerCorners() const;

    /**
     * @brief Get list of last detected marker IDs.
     */
    std::vector<int> getLastCapturedMarkerIds() const;

    /**
     * @brief Get observations with unique IDs and in ascending order
     *
     * @param[out] oObservationIds Ordered set holding the IDs of 2D image observations. The
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
                                std::vector<cv::Point2f>& oCornerObservations,
                                const int& iIterationBegin = 1,
                                const int& iNumIterations  = -1) const;

    /**
     * @brief Get sensor data from ros message.
     *
     * @param[in] ipMsg Pointer to input message.
     * @param[out] oData Sensor data.
     */
    bool getSensorDataFromMsg(const InputImage_Message_T::ConstSharedPtr& ipMsg,
                              cv::Mat& oData) const;

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
     * @brief Method to publish preview data, i.e. the camera image annotated with the marker
     * detections.
     *
     * @param[in] iHeader Header for the ROS message that is to be published.
     */
    void publishPreview(const std_msgs::msg::Header& iHeader) const override;

    using SensorDataProcessorBase::publishPreview;

    /**
     * @overload
     * @brief Method to publish data of successful target detection.
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

    //--- FUNCTION DECLARATION ---//

  public:
    /**
     * @brief Write observations of marker corners into file.
     *
     * @param[in] iFilePath Path to output file.
     * @param[in] iMarkerIds List of marker ids.
     * @param[in] iMarkerCorners List of marker corners. For each marker (corresponding to marker
     * ID) there are 4 corners with 2D coordinates.
     */
    static void writeMarkerObservationsToFile(
      const fs::path& iFilePath,
      const std::vector<int>& iMarkerIds,
      const std::vector<std::array<cv::Point2f, 4>>& iMarkerCorners);

    //--- MEMBER DECLARATION ---//

  protected:
    /// Copy of last camera image annotated with detected markers used for preview.
    cv::Mat annotatedCameraImgPreview_;

    /// Camera images in which the detections are annotated.
    std::vector<cv::Mat> annotatedCameraImageDetection_;

    /// List of captured marker IDs per iteration.
    std::vector<std::vector<int>> capturedMarkerIds_;

    /// List of captured marker corners corresponding to detected marker IDs
    std::vector<std::vector<std::array<cv::Point2f, 4>>> capturedMarkerCorners_;

    /// List of pointers to point cloud of detected calibration board
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> calibrationTargetCloudPtrs_;

    /// Publisher for image annotated with marker detections
    // image_transport::Publisher annotatedImgPub_;
    rclcpp::Publisher<AnnotatedImage_Message_T>::SharedPtr pAnnotatedImgPub_;

    /// Publisher for corners of marker positions
    rclcpp::Publisher<MarkerCornersImgPnts_Message_T>::SharedPtr pMarkerCornersPub_;

    /// Publisher for target point cloud
    rclcpp::Publisher<TargetPatternCloud_Message_T>::SharedPtr pTargetCloudPub_;

    /// Publisher for target board pose
    rclcpp::Publisher<TargetBoardPose_Message_T>::SharedPtr pTargetBoardPosePub_;
};

} // namespace multisensor_calibration

#endif