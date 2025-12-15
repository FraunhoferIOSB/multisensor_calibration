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

#ifndef MULTISENSORCALIBRATION_EXTRINSICCALIBRATIONBASE_H
#define MULTISENSORCALIBRATION_EXTRINSICCALIBRATIONBASE_H

// Std
#include <memory>
#include <string>

// ROS
#include <tf2/LinearMath/Transform.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// multisensor_calibration
#include "CalibrationBase.h"
#include <multisensor_calibration_interface/srv/calibration_meta_data.hpp>
#include <multisensor_calibration_interface/srv/remove_last_observation.hpp>
#include <multisensor_calibration_interface/srv/sensor_extrinsics.hpp>

namespace multisensor_calibration
{

/**
 * @ingroup calibration
 * @brief Base class of all extrinsic calibration nodes.
 *
 * @tparam SrcDataProcessorT Class to process data from source sensor.
 * @tparam RefDataProcessorT Class to process data from reference sensor.
 */
template <class SrcDataProcessorT, class RefDataProcessorT>
class ExtrinsicCalibrationBase : public CalibrationBase
{
    //--- STRUCTS ---//

    /**
     * @ingroup calibration
     * @brief Struct holding extrinsic calibration data as part of the calibration result
     */
    struct ExtrinsicCalibration
    {
        /// Name of source sensor.
        std::string srcSensorName;

        /// Frame id of source sensor.
        std::string srcFrameId;

        /// Name of reference sensor.
        std::string refSensorName;

        /// Frame id of reference sensor.
        std::string refFrameId;

        /// Base frame id.
        std::string baseFrameId;

        /// Translation of extrinsic transformation.
        tf2::Vector3 XYZ;

        /// Roll, Pitch, Yaw of extrinsic transformation.
        tf2::Vector3 RPY;
    };

    /**
     * @ingroup calibration
     * @brief Calibration result for extrinsic calibrations.
     */
    struct CalibrationResult
    {
        /// Data of pairwise sensor calibration
        std::vector<ExtrinsicCalibration> calibrations;

        /// Number of target observations
        int numObservations;

        /// Error of calibration made up of a string name and the actual error value
        std::pair<std::string, double> error;

        /// Standard Deviation in XYZ and RPY of target poses when transformed between sensor
        /// frames. In this, the RPY angles are given in degrees.
        std::pair<tf2::Vector3, tf2::Vector3> target_poses_stdDev;

        /// Print out calibration results to string.
        std::string toString() const;

        /// Print URDF snippet of the calibration to string.
        std::string urdfSnippet() const;

        CalibrationResult()
        {
            calibrations.resize(1);

            target_poses_stdDev =
              std::make_pair(tf2::Vector3(NAN, NAN, NAN),
                             tf2::Vector3(NAN, NAN, NAN));
        }
    };

    //--- METHOD DECLARATION ---/
  public:
    /**
     * @brief Default constructor is deleted.
     */
    ExtrinsicCalibrationBase() = delete;

    /**
     * @brief Initialization constructor.
     *
     * @brief[in] type type of calibration.
     */
    ExtrinsicCalibrationBase(ECalibrationType type);

    /**
     * @brief Destructor
     */
    virtual ~ExtrinsicCalibrationBase();

  protected:
    using CalibrationBase::initializeAndStartSensorCalibration;

    /**
     * @brief Initialize publishers. Purely virtual.
     *
     * @param[in, out] ipNode Pointer to node.
     * @return True, if successful. False otherwise.
     */
    virtual bool initializePublishers(rclcpp::Node* ipNode) override;

    /**
     * @brief Method to initialize services. This overrides the CalibrationBase::initializeServices.
     * In this, the method from the base class is also called.
     *
     * @param[in, out] ipNode Pointer to node.
     * @return True, if successful. False otherwise.
     */
    virtual bool initializeServices(rclcpp::Node* ipNode) override;

    /**
     * @brief Handle call requesting calibration meta data.
     *
     * @param[in] ipReq Request
     * @param[out] opRes Response.
     */
    bool onRequestCalibrationMetaData(
      const std::shared_ptr<interf::srv::CalibrationMetaData::Request> ipReq,
      std::shared_ptr<interf::srv::CalibrationMetaData::Response> opRes);

    /**
     * @brief Handle call requesting removal of last observation. This is a pure virtual definition
     * and needs to be implemented in a subclass.
     *
     * @param[in] ipReq Request, with flag to capture calibration target
     * @param[out] opRes Response, empty.
     */
    virtual bool onRequestRemoveObservation(
      const std::shared_ptr<interf::srv::RemoveLastObservation::Request> ipReq,
      std::shared_ptr<interf::srv::RemoveLastObservation::Response> opRes);

    /**
     * @brief Handling call requesting sensor extrinsics.
     *
     * @param[in] ipReq Request, empty.
     * @param[out] oRes Response.
     */
    virtual bool onRequestSensorExtrinsics(
      const std::shared_ptr<interf::srv::SensorExtrinsics::Request> ipReq,
      std::shared_ptr<interf::srv::SensorExtrinsics::Response> oRes);

    /**
     * @brief Publish given sensor extrinsics as calibration result.
     *
     * @param[in] iSensorExtrinsics Sensor extrinsics to publish.
     */
    void publishCalibrationResult(const lib3d::Extrinsics& iSensorExtrinsics) const;

    /**
     * @brief Publish las sensor extrinsics in sensorExtrinsics_ as calibration result.
     */
    void publishLastCalibrationResult() const;

    /**
     * @brief Save calibration settings to setting.ini inside calibration workspace.
     *
     * This overrides CalibrationBase::saveCalibrationSettingsToWorkspace. In this, the method of
     * the parent class is also called.
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
     * @brief Read launch parameters.
     *
     * This overrides CalibrationBase::readLaunchParameters. In this, the method of
     * the parent class is also called.
     *
     * The implementation within this class hold launch parameters that are common to all
     * calibration nodes, e.g. robot_ws_path, target_config_file.
     *
     * @param[in] ipNode Pointer to node.
     * @return True if successful. False, otherwise (e.g. if sanity check fails)
     */
    bool readLaunchParameters(const rclcpp::Node* ipNode) override;

    /**
     * @brief Method to remove all observations that were captured during the given calibration
     * iteration.
     *
     * @tparam Id_T Type of IDs.
     * @tparam Obs_T Type of Observations.
     * @tparam Cloud_T Type of target clouds.
     * @tparam Pose_T Type of target board poses.
     * @param[in] iCalibrationItr Calibration iteration.
     * @param[in, out] ioIds List of IDs.
     * @param[in, out] ioObs List of Observations.
     * @param[in, out] ioClouds List of target clouds.
     * @param[in, out] ioPoses List of target board clouds.
     */
    template <typename Id_T, typename Obs_T, typename Cloud_T, typename Pose_T>
    void removeObservationsFromIteration(const uint& iCalibrationItr,
                                         std::set<Id_T>& ioIds,
                                         std::vector<Obs_T>& ioObs,
                                         std::vector<Cloud_T>& ioClouds,
                                         std::vector<Pose_T>& ioPoses) const;

    /**
     * @brief Method to remove observations from ta given source list that do not have
     * corresponding observations with the same ID in the reference list.
     *
     * @tparam Id_T Type of IDs
     * @tparam Obs_T Type of Observations
     * @param[in] iReferenceIds Reference list of IDs to check against.
     * @param[in, out] ioSrcIds List of IDs which are to be checked against the reference list.
     * @param[in, out] ioSrcObs List of Observations.
     */
    template <typename Id_T, typename Obs_T>
    void removeCornerObservationsWithoutCorrespondence(const std::set<Id_T>& iReferenceIds,
                                                       std::set<Id_T>& ioSrcIds,
                                                       std::vector<Obs_T>& ioSrcObs) const;

    /**
     * @brief Reset calibration.
     *
     * This overrides CalibrationBase::reset. In this, the method of
     * the parent class is also called.
     */
    void reset() override;

    /**
     * @brief Save calibration.
     *
     * This overrides CalibrationBase::saveCalibration. In this, the method of
     * the parent class is also called.
     */
    bool saveCalibration() override;

    /**
     * @brief Method to save the calibration into the URDF model.
     *
     * This will write out the calibration into the provided urdf model. Prior to that the old
     * model file is copied and renamed as a backup.
     */
    bool saveCalibrationToUrdfModel();

    /**
     * @brief Trigger data processors to save observations that have been used for calibration to
     * the corresponding workspace.
     *
     * @return True, if successful. False, otherwise.
     */
    bool saveObservationsToCalibrationWorkspace() const;

    /**
     * @brief Method to save the results of the calibration into file in the calibration workspace.
     *
     * This will write out a file in the calibration workspace that olds teh calibration results.
     * If the desired file already exists, the existing file is copied and renamed.
     *
     * @return True, if successful. False, otherwise.
     */
    bool saveResultsToCalibrationWorkspace() const;

    /**
     * @brief Set relative sensor extrinsics (sensorExtrinsics_) from the given frame IDs.
     *
     * This will extract a transformation from the tfListener and set the sensor extrinsics
     * accordingly.
     *
     * @param[in] iSourceFrameId Frame Id of source sensor.
     * @param[in] iReferenceFrameId Frame Id of reference sensor.
     * @return True, if successful. False, otherwise.
     */
    bool setSensorExtrinsicsFromFrameIds(const std::string& iSourceFrameId,
                                         const std::string& iReferenceFrameId);

    /**
     * @brief Compute standard deviation in detected target poses when they are transformed between
     * the sensor frames based on the estimated extrinsic calibration.
     *
     * This will transform the detected target poses from the source sensor into the frame of the
     * reference sensor and compute the standard deviation between the poses. The transformation
     * is done using the sensor extrinsic that is estimated last.
     *
     * @param[in] iSrcTargetPoses Target poses detected in source sensor.
     * @param[in] iRefTargetPoses Target poses detected in reference sensor.
     * @return The standard deviation of the target poses in XYZ (in meters) and RPY (in degrees).
     */
    std::pair<tf2::Vector3, tf2::Vector3> computeTargetPoseStdDev(
      const std::vector<lib3d::Extrinsics>& iSrcTargetPoses,
      const std::vector<lib3d::Extrinsics>& iRefTargetPoses) const;

    //--- MEMBER DECLARATION ---/

  protected:
    /// Pointert to publish calibration result.
    rclcpp::Publisher<CalibrationResult_Message_T>::SharedPtr pCalibResultPub_;

    /// Pointer to service to remove last observation
    rclcpp::Service<interf::srv::RemoveLastObservation>::SharedPtr pRemoveObsSrv_;

    /// Pointer to service to request calibration meta data
    rclcpp::Service<interf::srv::CalibrationMetaData>::SharedPtr pCalibMetaDataSrv_;

    /// Pointer to service to request sensor extrinsics
    rclcpp::Service<interf::srv::SensorExtrinsics>::SharedPtr pSensorExtrinsicsSrv_;

    /// Pointer to data processor of source sensor.
    std::shared_ptr<SrcDataProcessorT> pSrcDataProcessor_;

    /// Pointer to data processor of reference sensor.
    std::shared_ptr<RefDataProcessorT> pRefDataProcessor_;

    /// Name of the source sensor.
    std::string srcSensorName_;

    /// Topic name of the data from the source sensor
    std::string srcTopicName_;

    /// Frame id of data from the source sensor
    std::string srcFrameId_;

    /// Name of the reference sensor.
    std::string refSensorName_;

    /// Topic name of the data from the source sensor
    std::string refTopicName_;

    /// Frame id of data from the reference sensor
    std::string refFrameId_;

    /// Frame id of base frame with respect to which the source frame is to be calibrated.
    /// If omitted, the source frame will be calibrated with respect to refFrameID_.
    std::string baseFrameId_;

    /// Object holding extrinsic transformation between the source and the reference sensor.
    std::vector<lib3d::Extrinsics> sensorExtrinsics_;

    /// Object holding the result of the calibration.
    CalibrationResult calibResult_;

    /// Fag to use frame IDs in TF-Tree as initial guess. Default: false.
    bool useTfTreeAsInitialGuess_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_EXTRINSICCALIBRATIONBASE_H