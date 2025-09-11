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

#ifndef MULTISENSORCALIBRATION_EXTRINSIC2D3DCALIBRATIONBASE_H
#define MULTISENSORCALIBRATION_EXTRINSIC2D3DCALIBRATIONBASE_H

// ROS
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

// PCL
#include <pcl/point_cloud.h>

// multisensor_calibration
#include "../sensor_data_processing/CameraDataProcessor.h"
#include "ExtrinsicCalibrationBase.h"
#include <multisensor_calibration_interface/srv/camera_intrinsics.hpp>

namespace multisensor_calibration
{

/**
 * @ingroup calibration
 * @brief Base class for all extrinsic 2D-3D calibration routines.
 *
 * This subclasses multisensor_calibration::ExtrinsicCalibrationBase.
 *
 * @tparam SrcDataProcessorT Class to process data from source sensor.
 * @tparam RefDataProcessorT Class to process data from reference sensor.
 */
template <class SrcDataProcessorT, class RefDataProcessorT>
class Extrinsic2d3dCalibrationBase
  : public ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>
{

    //--- TYPEDEFS ---//
  protected:
    typedef message_filters::sync_policies::ApproximateTime<
      InputImage_Message_T, InputCloud_Message_T>
      ImgCloudApproxSync;

    typedef message_filters::sync_policies::ExactTime<
      InputImage_Message_T, InputCloud_Message_T>
      ImgCloudExactSync;

    //--- METHOD DECLARATION ---/

  public:
    /**
     * @brief Default constructor is deleted
     */
    Extrinsic2d3dCalibrationBase() = delete;

    /**
     * @brief Initialization constructor
     *
     * @param[in] type Type of calibration
     */
    Extrinsic2d3dCalibrationBase(ECalibrationType type);

    /**
     * @brief Destructor
     */
    virtual ~Extrinsic2d3dCalibrationBase();

  protected:
    void calculateAdditionalStereoCalibrations();

    /**
     * @brief Method to do PnP calibration between observation of 2d corner points from camera
     * sensor and 3d corner point from the lidar sensor.
     *
     * @param[in] iCamObsStart Begin iterator of the corner observations from the camera image.
     * @param[in] iCamObsEnd End iterator of the corner observations from the camera image.
     * @param[in] iLidarObsStart Begin iterator of the corner observations from the lidar image.
     * @param[in] iLidarObsEnd End iterator of the corner observations from the lidar image.
     * @param[in] iCameraIntrinsics Intrinsic parameters of the camera.
     * @param[in] iInlierMaxRpjError Maximum reprojection error to be accepted for inliers.
     * @param[in] iUsePoseGuess Flag to use pose guess from back of sensorExtrinsics_.
     * @param[out] oNewSensorExtrinsics Estimated sensor extrinsics.
     * @param[in] iIndices Optional list of indices within list of observation to consider for
     * calibration. Leave empty if full range of passed observations is to be used.
     * @return Pair made up of mean reprojection error and number of inliers from the RANSAC detection.
     */
    std::pair<double, int> runPnp(
      const std::vector<cv::Point2f>::const_iterator& iCamObsStart,
      const std::vector<cv::Point2f>::const_iterator& iCamObsEnd,
      const std::vector<cv::Point3f>::const_iterator& iLidarObsStart,
      const std::vector<cv::Point3f>::const_iterator& iLidarObsEnd,
      const lib3d::Intrinsics& iCameraIntrinsics,
      const float& iInlierMaxRpjError,
      const bool& iUsePoseGuess,
      lib3d::Extrinsics& oNewSensorExtrinsics,
      const std::vector<uint>& iIndices = {}) const;

    /**
     * @brief Initialize camera intrinsics from camera info topics.
     *
     * @param[in, out] iopCamProcessor Pointer to camera data processor to which the intrinsics are
     * to be set.
     * @return True, if successful. False otherwise.
     */
    bool initializeCameraIntrinsics(CameraDataProcessor* iopCamProcessor);

    /**
     * @brief Method to initialize subscribers. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool initializeSubscribers(rclcpp::Node* ipNode) override;

    /**
     * @brief Handle reception of camera info message of left camera.
     */
    void onLeftCameraInfoReceived(const sensor_msgs::msg::CameraInfo::SharedPtr pCamInfo);

    /**
     * @brief Handle reception of camera info message of right camera.
     * Only used when isStereoCamera_ == true
     */
    void onRightCameraInfoReceived(const sensor_msgs::msg::CameraInfo::SharedPtr pCamInfo);

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
     * @brief Method to read launch parameters. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * @param[in] iNh Object of node handle
     * @return True if successful. False, otherwise (e.g. if sanity check fails)
     */
    bool readLaunchParameters(const rclcpp::Node* ipNode) override;

    //--- MEMBER DECLARATION ---//

  protected:
    using ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::calibResult_;

    /// Name of the camera sensor as given in the URDF model.
    /// This is a reference to ExtrinsicCalibrationBase::srcSensorName_
    std::string& cameraSensorName_ =
      ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::srcSensorName_;

    /// Topic name of the camera images which are to be used for extrinsic calibration.
    /// This is a reference to ExtrinsicCalibrationBase::srcTopicName_
    std::string& cameraImageTopic_ =
      ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::srcTopicName_;

    /// Topic name of the camera info messages.
    /// If not explicitly set, this is constructed from #cameraImageTopic_.
    std::string cameraInfoTopic_;

    /// Frame id of image received by #imageSubsc_
    /// This is a reference to ExtrinsicCalibrationBase::srcFrameId_
    std::string& imageFrameId_ =
      ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::srcFrameId_;

    /// State of image received on the subscribed topic
    EImageState imageState_;

    /// True if camera that is calibrated is a stereo.
    bool isStereoCamera_;

    /// Name of the right camera sensor in case of a stereo camera.
    std::string rightCameraSensorName_;

    /// Topic name of the right camera info messages in case of a stereo camera.
    std::string rightCameraInfoTopic_;

    /// Frame id of the right image in case of a stereo camera.
    /// This is retrieved from the camera info message of the right camera.
    std::string rightImageFrameId_;

    /// Suffix of the right sensor name as well as the frame id for the rectified images. If the
    /// imageState_ input images is DISTORTED or UNDISTORTED, this is added to the rectified frame
    /// id. If the imageState_ is STEREO_RECTIFIED this is removed from the frame id.
    std::string rectSuffix_;

    /// Subscriber to camera info messages of left camera.
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr pLeftCamInfoSubsc_;

    /// Camera info data of left camera.
    sensor_msgs::msg::CameraInfo leftCameraInfo_;

    /// Subscriber to camera info messages of right camera. Only used when isStereoCamera_ == true
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr pRightCamInfoSubsc_;

    /// Camera info data of right camera. Only used when isStereoCamera_ == true
    sensor_msgs::msg::CameraInfo rightCameraInfo_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_EXTRINSIC2D3DCALIBRATIONBASE_H