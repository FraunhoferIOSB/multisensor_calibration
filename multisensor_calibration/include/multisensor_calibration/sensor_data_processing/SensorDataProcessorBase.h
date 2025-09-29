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

#ifndef MULTISENSORCALIBRATION_SENSORDATAPROCESSORBASE_H
#define MULTISENSORCALIBRATION_SENSORDATAPROCESSORBASE_H

// Std
#include <filesystem>
#include <vector>

// ROS
#include <rclcpp/logger.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>

// OpenCV
#include <opencv2/core.hpp>

// multisensor_calibration
#include "../calibration_target/CalibrationTarget.hpp"
#include "../common/common.h"
#include <multisensor_calibration/common/lib3D/core/extrinsics.hpp>

namespace fs = std::filesystem;

namespace multisensor_calibration
{

/**
 * @ingroup  target_detection
 * @brief Base class of all sensor data processor templated with the data type of the sensor data
 * which is to be processed by the class
 *
 * @tparam SensorDataT Type of data to be processed.
 */
template <class SensorDataT>
class SensorDataProcessorBase
{
    //--- ENUM DECLARATION ---//
  public:
    /**
     * @brief Enumeration holding level at which the sensor data is to be processed.
     */
    enum EProcessingLevel
    {
        PREVIEW = 0,     ///< Process sensor data only for preview purposes.
        TARGET_DETECTION ///< Do actual detection of calibration target in data processing.
    };

    /**
     * @brief Enumeration holding the different results that can be returned after data processing.
     */
    enum EProcessingResult
    {
        FAILED = -1, ///< Processing of sensor data has failed.
        SUCCESS,     ///< Processing of sensor data has succeeded. Only then, are the results that valid.
        PENDING      ///< processing is pending, i.e. waiting for more data.
    };

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Default constructor is deleted.
     *
     */
    SensorDataProcessorBase() = delete;

    /**
     * @brief Initialization constructor, providing the node name, the sensor name as well
     * as the path to the calibration target configuration file.
     */
    SensorDataProcessorBase(const std::string& iLoggerName,
                            const std::string& iSensorName,
                            const fs::path& iCalibTargetFilePath);

    /**
     * @brief Destructor
     */
    virtual ~SensorDataProcessorBase();

    /**
     * @brief Get List of all detected and estimated poses of the calibration target.
     */
    std::vector<lib3d::Extrinsics> getCalibrationTargetPoses() const;

    /**
     * @brief Get last pose of calibration target board.
     */
    lib3d::Extrinsics getLastCalibrationTargetPose() const;

    /**
     * @brief Get the number of iterations for which observations have been captured.
     */
    uint getNumCalibIterations() const;

    /**
     * @brief Get name of the sensor from which the data is processed.
     */
    std::string getSensorName() const;

    /**
     * @brief Purely virtual interface class to call the processing of the data.
     *
     * @param[in] iSensorData Sensor data.
     * @param[in] iProcLevel Level at which the data is to be processed, i.e. PREVIEW or
     * TARGET_DETECTION
     * @return Result of the processing.
     */
    virtual EProcessingResult processData(
      const SensorDataT& iSensorData,
      const EProcessingLevel& iProcLevel) = 0;

    /**
     * @brief Method to publish given extrinsic pose of the calibration target.
     *
     * @param[in] iHeader Header with which to publish the pose.
     * @param[in] iPose Pose to publish.
     * @param[in] iIsDetection Flag to indicate if pose is actual detection or just preview.
     * @param[in] ipPub Publisher.
     */
    void publishLastCalibrationTargetPose(
      const std_msgs::msg::Header& iHeader,
      const lib3d::Extrinsics& iPose,
      const bool& iIsDetection,
      const rclcpp::Publisher<TargetBoardPose_Message_T>::SharedPtr& ipPub) const;

    /**
     * @brief Method to publish preview data.
     *
     * @param[in] iHeader Header for the ROS message that is to be published.
     */
    virtual void publishPreview(const std_msgs::msg::Header& iHeader) const = 0;

    /**
     * @overload
     *
     * @param[in] iStamp Timestamp for the ROS message that is to be published.
     * @param[in] iFrameId FrameId for the ROS message that is to be published.
     */
    void publishPreview(const rclcpp::Time& iStamp, std::string& iFrameId) const;

    /**
     * @brief Method to publish data of last successful target detection.
     *
     * @param[in] iHeader Header for the ROS message that is to be published.
     */
    virtual void publishLastTargetDetection(const std_msgs::msg::Header& iHeader) const = 0;

    /**
     * @overload
     *
     * @param[in] iStamp Timestamp for the ROS message that is to be published.
     * @param[in] iFrameId FrameId for the ROS message that is to be published.
     */
    void publishLastTargetDetection(const rclcpp::Time& iStamp, std::string& iFrameId) const;

    /**
     * @brief Remove observations corresponding to given calibration iteration.
     *
     * @param[in] iIterationId Iteration ID for which the observations are to be removed.
     * @return True, if successful. False, otherwise (i.e. if the specified iteration is not
     * available).
     */
    virtual bool removeCalibIteration(const uint& iIterationId);

    /**
     * @brief Save the observations to given output directory.
     *
     * @param[in] iOutputDir Path to output directory.
     * @return True if successful, false otherwise.
     */
    virtual bool saveObservations(const fs::path iOutputDir) const;

  protected:
    /**
     * @brief Method to average the observations.
     *
     * @param[out] iCalibTargetPoses List of calibration target poses which are to be averaged.
     * @param[out] oAvgdCalibTargetPose Pose of the calibration target, averaged
     * from given poses.
     * @return True if successful, false otherwise (e.g. no observations available).
     */
    bool averageObservations(const std::vector<lib3d::Extrinsics>& iCalibTargetPoses,
                             lib3d::Extrinsics& oAvgdCalibTargetPose) const;

    /**
     * @brief Method to initialize publishers
     *
     * @param[in, out] ipNode Pointer to node to which the publishers are to be initialized.
     * @return True, if successful. False otherwise.
     */
    virtual bool initializePublishers(rclcpp::Node* ipNode) = 0;

    /**
     * @brief Method to initialize services
     *
     * @param[in, out] ioNh Pointer to node to which the services are to be initialized.
     * @return True, if successful. False otherwise.
     */
    virtual bool initializeServices(rclcpp::Node* ipNode) = 0;

    /**
     * @brief Method to reset.
     */
    virtual void reset();

    //--- MEMBER DECLARATION ---//

  protected:
    /// Logging Object
    rclcpp::Logger logger_;

    /// Flag indicating if node is initialized.
    bool isInitialized_;

    /// Name of the sensor for which the data is being processed.
    std::string sensorName_;

    /// Object of calibration target
    CalibrationTarget calibrationTarget_;

    /// Range of marker IDs deployed on calibration target
    std::pair<int, int> markerIdRange_;

    /// List of captured poses of the calibration target.
    std::vector<lib3d::Extrinsics> capturedCalibTargetPoses_;
};

} // namespace multisensor_calibration

#endif