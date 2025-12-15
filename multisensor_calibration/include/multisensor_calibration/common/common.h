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

#ifndef MULTISENSORCALIBRATION_COMMON_H
#define MULTISENSORCALIBRATION_COMMON_H

// Std
#include <string>
#include <unordered_map>

// ROS
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

// PCL
#include <pcl/point_types.h>

// multisensor_calibration
#include <multisensor_calibration_interface/msg/calibration_result.hpp>
#include <multisensor_calibration_interface/msg/stamped_float32_multi_array.hpp>
#include <multisensor_calibration_interface/msg/target_pose.hpp>

// Macro to declare unused variable
#define UNUSED_VAR(name) (void)name

// Macro to stringify variable name
#define STRINGIFY(s) STR(s)
#define STR(s) #s

#if !defined(VERSION_MAJOR)
#define VERSION_MAJOR 0
#endif

#if !defined(VERSION_MINOR)
#define VERSION_MINOR 0
#endif

#if !defined(VERSION_PATCH)
#define VERSION_PATCH 0
#endif

namespace multisensor_calibration
{

//--- TYPE DEFINITIONS ---//

/// Typedef for point type used for point clouds
typedef pcl::PointXYZI InputPointType;

//--- TOPIC NAMES ---//

// Topic name of camera image annotated with marker detections
static const std::string ANNOTATED_CAMERA_IMAGE_TOPIC_NAME = "annotated_image";

// Topic name of cloud holding marker corners. In case of the LiDAR data processor these are the 3D
// coordinates of the corners. In case of the camera data processor these are 2D image points of
// the corners.
static const std::string MARKER_CORNERS_TOPIC_NAME = "marker_corners";

// Topic name of cloud holding the selected regions of interest
static const std::string ROIS_CLOUD_TOPIC_NAME = "regions_of_interest";

// Topic name of target pattern cloud
static const std::string TARGET_PATTERN_CLOUD_TOPIC_NAME = "target_pattern";

// Topic name of target board pose
static const std::string TARGET_BOARD_POSE_TOPIC_NAME = "board_pose";

// Topic name of target board placement guidance
static const std::string PLACEMENT_GUIDANCE_TOPIC_NAME = "placement_guidance";

// Topic name of calibration result
static const std::string CALIB_RESULT_TOPIC_NAME = "calibration_result";

//--- MESSAGE DATA TYPES ---//

typedef sensor_msgs::msg::Image InputImage_Message_T;
typedef sensor_msgs::msg::PointCloud2 InputCloud_Message_T;

typedef sensor_msgs::msg::Image AnnotatedImage_Message_T;
typedef sensor_msgs::msg::PointCloud2 MarkerCornerCloud_Message_T;
typedef multisensor_calibration_interface::msg::StampedFloat32MultiArray MarkerCornersImgPnts_Message_T;
typedef multisensor_calibration_interface::msg::TargetPose TargetBoardPose_Message_T;
typedef sensor_msgs::msg::PointCloud2 RoisCloud_Message_T;
typedef sensor_msgs::msg::PointCloud2 TargetPatternCloud_Message_T;
typedef visualization_msgs::msg::Marker TargetPlacementBox_Message_T;
typedef multisensor_calibration_interface::msg::CalibrationResult CalibrationResult_Message_T;

//--- SERVICE NAMES ---//

// Service name to add marker observations
static const std::string ADD_MARKER_OBS_SRV_NAME = "add_marker_observations";

// Service name to add region marker
static const std::string ADD_REGION_MARKER_SRV_NAME = "add_region_marker";

// Service name to request camera intrinsics
static const std::string REQUEST_CAM_INTRINSICS_SRV_NAME = "request_camera_intrinsics";

// Service name to request sensor extrinsics
static const std::string REQUEST_SENSOR_EXTRINSICS_SRV_NAME = "request_sensor_extrinsics";

// Service name to capture target
static const std::string CAPTURE_TARGET_SRV_NAME = "capture_target";

// Service name to finalize calibration
static const std::string FINALIZE_CALIBRATION_SRV_NAME = "finalize_calibration";

// Service name to import marker observations
static const std::string IMPORT_MARKER_OBS_SRV_NAME = "import_marker_observations";

// Service name to remove last observation
static const std::string REMOVE_OBSERVATION_SRV_NAME = "remove_last_observation";

// Service name to request processor state
static const std::string REQUEST_STATE_SRV_NAME = "request_processor_state";

// Service name to request calibration meta data
static const std::string REQUEST_META_DATA_SRV_NAME = "request_calibration_meta_data";

// Service name to reset calibration
static const std::string RESET_SRV_NAME = "reset";

//--- NAMESPACES ---//

// Namespace of the calibrator node within the cumulative node
static const std::string CALIBRATION_SUB_NAMESPACE = "calibration";

// Namespace of the guidance node within the cumulative node
static const std::string GUIDANCE_SUB_NAMESPACE = "guidance";

// Sub namespace of the gui within the cumulative node
static const std::string GUI_SUB_NAMESPACE = "gui";

// Sub namespace of the visualization node within the cumulative node.
static const std::string VISUALIZER_SUB_NAMESPACE = "calib_visualization";

//--- STATIC VARIABLES / DEFAULT VALUES ---//

/// String of default image state.
static const std::string DEFAULT_IMG_STATE_STR = "DISTORTED";

// Name of file holding workspace/calibration settings.
static const std::string SETTINGS_FILE_NAME = "settings.ini";

// Name of root subdirectory for storing observations
static const std::string OBSERVATIONS_SUBDIR_NAME = "observations";

// Suffix of the file holding image annotated with the target detection.
static const std::string ANNOTATED_IMAGE_FILE_SUFFIX = "_annotated_image.png";

// Suffix of the file holding point cloud annotated with the target detection.
static const std::string ANNOTATED_CLOUD_FILE_SUFFIX = "_annotated_cloud.ply";

// Suffix of the file holding the marker corner observations.
static const std::string OBSERVATIONS_FILE_SUFFIX = "_marker_corner_observations.txt";

// Name of root subdirectory for workspace backups.
static const std::string WS_BACKUP_SUBDIR_NAME = "_backups";

// Name of file in which the calibration results are stored in the calibration workspace.
static const std::string CALIB_RESULTS_FILE_NAME = "calibration_results.txt";

// Name of file in which the urdf snippet is stored in the calibration workspace.
static const std::string URDF_SNIPPET_FILE_NAME = "urdf_snippet.txt";

/// Default camera sensor name.
static const std::string DEFAULT_CAMERA_SENSOR_NAME = "camera";

/// Default camera image topic to which to subscribe.
static const std::string DEFAULT_CAMERA_IMAGE_TOPIC = "/camera/image_color";

/// Default lidar sensor name.
static const std::string DEFAULT_LIDAR_SENSOR_NAME = "lidar";

/// Default lidar image topic.
static const std::string DEFAULT_LIDAR_CLOUD_TOPIC = "/lidar/cloud";

/// Default number of observations to acquire for lidar
static const int DEFAULT_NUM_LIDAR_OBSERVATIONS = 1;

/// Default size of message queue for approximated time synchronization
static const int DEFAULT_SYNC_QUEUE_SIZE = 100;

//--- ENUMS ---//

/**
 * @ingroup calibration
 * @brief Enum representing type of calibration.
 */
enum ECalibrationType
{
    EXTRINSIC_CAMERA_LIDAR_CALIBRATION = 0, ///< Extrinsic Camera-LiDAR calibration.
    EXTRINSIC_LIDAR_LIDAR_CALIBRATION,      ///< Extrinsic LiDAR-LiDAR calibration.
    EXTRINSIC_CAMERA_REFERENCE_CALIBRATION, ///< Extrinsic Camera-Reference calibration.
    EXTRINSIC_LIDAR_REFERENCE_CALIBRATION,  ///< Extrinsic LiDAR-Reference calibration.
    EXTRINSIC_LIDAR_VEHICLE_CALIBRATION,    ///< Extrinsic LiDAR-Vehicle calibration.
    STEREO_CAMERA_CALIBRATION               ///< Intrinsic and Extrinsic Camera-Camera calibration.
};

/// Map convert a value inside the ECalibrationType enum to a node name of type string.
static const std::unordered_map<ECalibrationType, std::string> CALIB_TYPE_2_NODE_NAME{
  {EXTRINSIC_CAMERA_LIDAR_CALIBRATION, "extrinsic_camera_lidar_calibration"},
  {EXTRINSIC_LIDAR_LIDAR_CALIBRATION, "extrinsic_lidar_lidar_calibration"},
  {EXTRINSIC_CAMERA_REFERENCE_CALIBRATION, "extrinsic_camera_reference_calibration"},
  {EXTRINSIC_LIDAR_REFERENCE_CALIBRATION, "extrinsic_lidar_reference_calibration"},
  {EXTRINSIC_LIDAR_VEHICLE_CALIBRATION, "extrinsic_lidar_vehicle_calibration"},
  {STEREO_CAMERA_CALIBRATION, "stereo_camera_calibration"}};

/// Map convert a value inside the ECalibrationType enum to a string.
static const std::unordered_map<ECalibrationType, std::string> CALIB_TYPE_2_STR{
  {EXTRINSIC_CAMERA_LIDAR_CALIBRATION, "Extrinsic Camera-LiDAR"},
  {EXTRINSIC_LIDAR_LIDAR_CALIBRATION, "Extrinsic LiDAR-LiDAR"},
  {EXTRINSIC_CAMERA_REFERENCE_CALIBRATION, "Extrinsic Camera-Reference"},
  {EXTRINSIC_LIDAR_REFERENCE_CALIBRATION, "Extrinsic LiDAR-Reference"},
  {EXTRINSIC_LIDAR_VEHICLE_CALIBRATION, "Extrinsic LiDAR-Vehicle"},
  {STEREO_CAMERA_CALIBRATION, "Intrinsic and Extrinsic Camera-Camera"}};

/// Map to a string to value inside the ECalibrationType enum.
static const std::unordered_map<std::string, ECalibrationType> STR_2_CALIB_TYPE{
  {"Extrinsic Camera-LiDAR", EXTRINSIC_CAMERA_LIDAR_CALIBRATION},
  {"Extrinsic LiDAR-LiDAR", EXTRINSIC_LIDAR_LIDAR_CALIBRATION},
  {"Extrinsic Camera-Reference", EXTRINSIC_CAMERA_REFERENCE_CALIBRATION},
  {"Extrinsic LiDAR-Reference", EXTRINSIC_LIDAR_REFERENCE_CALIBRATION},
  {"Extrinsic LiDAR-Vehicle", EXTRINSIC_LIDAR_VEHICLE_CALIBRATION},
  {"Intrinsic and Extrinsic Camera-Camera", STEREO_CAMERA_CALIBRATION}};

/// Enum indicating the state of the input image.
enum EImageState
{
    DISTORTED = 0,   ///< As it comes from the camera, i.e. image distortion has not yet been corrected and it is not rectified for stereo processing.
    UNDISTORTED,     ///< Image with no distortion but not rectified for stereo processing.
    STEREO_RECTIFIED ///< Image rectified for stereo processing, i.e. the epipolar lines are horizontally aligned.
};

/// Map allowing to map a value inside the EImageState enum to a string.
static const std::unordered_map<EImageState, std::string> IMG_STATE_2_STR{
  {DISTORTED, "DISTORTED"},
  {UNDISTORTED, "UNDISTORTED"},
  {STEREO_RECTIFIED, "STEREO_RECTIFIED"}};

/// Map allowing to map a string to value inside the EImageState enum.
static const std::unordered_map<std::string, EImageState> STR_2_IMG_STATE{
  {"DISTORTED", DISTORTED},
  {"UNDISTORTED", UNDISTORTED},
  {"STEREO_RECTIFIED", STEREO_RECTIFIED}};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_COMMON_H