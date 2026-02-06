# Nodes, Composable Nodes and Nodelets

## Nodes

### multisensor_calibration

Main and user-friendly entry point to start a multi-sensor calibration.
This will start the [Calibration Configurator](tutorial.md#calibration-configurator) with which any calibration can be parameterized and started.
This does not require any launch parameters and can just be started like this:
```
ros2 run multisensor_calibration multisensor_calibration
```

<hr>

### extrinsic_camera_lidar_calibration

The node for the [guided extrinsic calibration between a camera and a LiDAR sensor](tutorial.md#extrinsic-camera-lidar-calibration) with a user-friendly UI.

It comprises

- the [ExtrinsicCameraLidarCalibration(Nodelet)](#extrinsiccameralidarcalibrationnodelet) performing the actual calibration,
- the [GuidedCameraLidarTargetPlacement(Nodelet)](#guidedcameralidartargetplacementnodelet) responsible for guiding the user to place the calibration target, as well as
- an instance of 'CameraLidarCalibrationGui' as a graphical user interface.

**Launch-Parameters:**

 - ```robot_ws_path```: Path to the folder holding the robot workspace. This will NOT be created if it does not yet exist. 
    See section ['Initialize new Robot workspace'](workspaces.md#initialize-new-robot-workspace) to find out how to create a new one<br>
    *Type: String*
 - ```target_config_file```: Path to the file holding the [configuration of the calibration target](calibration_target.md).<br>
      *E.g. "$(find multisensor_calibration)/config/TargetWithCirclesAndAruco.yaml"*<br>
    *Type: String*
 - ```camera_sensor_name```: Name of the camera sensor that is to be calibrated.<br>
    *Type: String*
 - ```camera_image_topic```: Topic name of the corresponding camera images.<br>
    *Type: String*
 - ```lidar_sensor_name```: Name of the LiDAR sensor with respect to which the camera is to be
 calibrated.<br>
    *Type: String*
 - ```lidar_cloud_topic```: Topic name of the corresponding LiDAR cloud.<br>
    *Type: String*
 - ```camera_info_topic```: Name of the camera info topic. If this parameter is omitted the camera
   info topic name is constructed from the specified ```camera_image_topic```.<br>
   *Type: String*<br>
   *Default: ""*
  - ```image_state```: State of the camera images used.<br>
    *Type: String*<br>
    *Default: 'DISTORTED'*<br>
    *Possible values:*<br>
      - *'DISTORTED': As it comes from the camera, i.e. image distortion has not yet been corrected and it is not rectified for stereo processing.*<br>
      - *'UNDISTORTED': Image with no distortion but not rectified for stereo processing.*<br>
      - *'STEREO_RECTIFIED': Image rectified for stereo processing, i.e. the epipolar lines are horizontally aligned.*
 - ```is_stereo_camera```: Set to true, if camera is to be calibrated as stereo camera. If set to true, 'right_camera_sensor_name' and 'right_camera_info_topic' also need to be set.<br>
    *Type: bool*<br>
    *Default: false*
 - ```right_camera_sensor_name```: Name of the right camera sensor when the camera is to be calibrated as a stereo camera system. Required if ```is_stereo_camera == true```.<br>
    *Type: String*<br>
    *Default: ""*
 - ```right_camera_info_topic```: Topic name of the camera info corresponding to the right camera. This is needed when the camera is to be calibrated as a stereo camera system. Required if ```is_stereo_camera == true```.<br>
    *Type: String*<br>
    *Default: ""*
 - ```rect_suffix```: Suffix of the of the right sensor name as well as the frame id for the rectified images. If the 'image_state' of the input images is DISTORTED or UNDISTORTED, this is added to the rectified frame id. If the imageState_ is STEREO_RECTIFIED this is removed from the frame id.<br>
    *Type: String*<br>
    *Default: "_rect"*
 - ```base_frame_id```: If specified, the extrinsic pose will be calculated with respect to frame of the given frame ID. This does not change the frame ID of the reference sensor, i.e. the LiDAR sensor, but will perform an a posteriori transformation of the estimated extrinsic pose into the specified frame. If not specified, or left empty, the extrinsic pose will be calculated with respect to the frame of the reference sensor.<br>
    *Type: String*<br>
    *Default: ""*
 - ```use_initial_guess```: Option to use an initial guess on the extrinsic sensor pose from the TF-tree, if available.<br>
    *Type: bool*<br>
    *Default: false*
 - ```save_observations```: Option to save recorded observations that have been used for the calibration to the workspace<br>
    *type: bool*<br>
    *Default: false*
 - ```sync_queue_size```: Queue size used for the synchronization between the messages of the camera images and the LiDAR clouds<br>
    *Type: int*<br>
    *Default: 100*
 - ```use_exact_sync```: Set to true if an exact time synchronization between the camera image messages and the LiDAR cloud messages.<br>
    *Type: bool*<br>
    *Default: false*

<hr>

### extrinsic_lidar_lidar_calibration

The node for the [guided extrinsic calibration between two LiDAR sensors](tutorial.md#extrinsic-lidar-lidar-calibration) with a user-friendly UI.

It comprises

- the [ExtrinsicLidarLidarCalibration(Nodelet)](#extrinsiclidarlidarcalibrationnodelet) performing the actual calibration,
- the [GuidedLidarLidarTargetPlacement(Nodelet)](#guidedlidarlidartargetplacementnodelet) responsible for guiding the user to place the calibration target, as well as
- an instance of 'LidarLidarCalibrationGui' as a graphical user interface.

**Launch-Parameters:**

 - ```robot_ws_path```: Path to the folder holding the robot workspace. This will NOT be created if it does not yet exist. 
    See section ['Initialize new Robot workspace'](workspaces.md#initialize-new-robot-workspace) to find out how to create a new one<br>
    *Type: String*
 - ```target_config_file```: Path to the file holding the [configuration of the calibration target](calibration_target.md).<br>
      *E.g. "$(find multisensor_calibration)/config/TargetWithCirclesAndAruco.yaml"*<br>
    *Type: String*
 - ```src_lidar_sensor_name```: Name of the source lidar sensor that is to be calibrated.<br>
    *Type: String*
 - ```src_lidar_cloud_topic```: Topic name of the corresponding LiDAR cloud.<br>
    *Type: String*
 - ```ref_lidar_sensor_name```: ame of the reference LiDAR sensor with respect to which the source LiDAR sensor is to be calibrated.<br>
    *Type: String*
 - ```ref_lidar_cloud_topic```: Topic name of the corresponding LiDAR cloud.<br>
    *Type: String*
 - ```base_frame_id```: If specified, the extrinsic pose will be calculated with respect to frame of the given frame ID. This does not change the frame ID of the reference sensor, i.e. the LiDAR sensor, but will perform an a posteriori transformation of the estimated extrinsic pose into the specified frame. If not specified, or left empty, the extrinsic pose will be calculated with respect to the frame of the reference sensor.<br>
    *Type: String*<br>
    *Default: ""*
 - ```use_initial_guess```: Option to use an initial guess on the extrinsic sensor pose from the TF-tree, if available.<br>
    *Type: bool*<br>
    *Default: false*
 - ```align_ground_planes```: Set to true, to additionally align the ground planes in the sensor data. Additionally specify the upright frame ID.
    *Type: bool*
    *Default: false*
 - ```upright_frame_id```: ID of Frame which has an upwards pointing z-axis. Used to detect ground plane in sensor data.
    *Type: String*
    *Default: ""*
 - ```save_observations```: Option to save recorded observations that have been used for the calibration to the workspace<br>
    *type: bool*<br>
    *Default: false*
 - ```sync_queue_size```: Queue size used for the synchronization between the messages of the camera images and the LiDAR clouds<br>
    *Type: int*<br>
    *Default: 100*
 - ```use_exact_sync```: Set to true if an exact time synchronization between the camera image messages and the LiDAR cloud messages.<br>
    *Type: bool*<br>
    *Default: false*

<hr>

### extrinsic_camera_reference_calibration

The node for the [guided extrinsic calibration of a camera with respect to a reference](tutorial.md#extrinsic-camera-reference-calibration) with a user-friendly UI.

It comprises

- the [ExtrinsicCameraReferenceCalibration(Nodelet)](#extrinsiccamerareferencecalibrationnodelet) performing the actual calibration,
- the [GuidedCameraLidarTargetPlacement(Nodelet)](#guidedcameralidartargetplacementnodelet) responsible for guiding the user to place the calibration target, as well as
- an instance of 'CameraReferenceCalibrationGui' as a graphical user interface.

**Launch-Parameters:**

 - ```robot_ws_path```: Path to the folder holding the robot workspace. This will NOT be created if it does not yet exist. 
    See section ['Initialize new Robot workspace'](workspaces.md#initialize-new-robot-workspace) to find out how to create a new one<br>
    *Type: String*
 - ```target_config_file```: Path to the file holding the [configuration of the calibration target](calibration_target.md).<br>
      *E.g. "$(find multisensor_calibration)/config/TargetWithCirclesAndAruco.yaml"*<br>
    *Type: String*
 - ```camera_sensor_name```: Name of the camera sensor that is to be calibrated.<br>
    *Type: String*
 - ```camera_image_topic```: Topic name of the corresponding camera images.<br>
    *Type: String*
 - ```reference_name```: Name of the reference
 calibrated.<br>
    *Type: String*
 - ```reference_frame_id```: Frame ID in which the reference data is provided.<br>
    *Type: String*
 - ```camera_info_topic```: Name of the camera info topic. If this parameter is omitted the camera
   info topic name is constructed from the specified ```camera_image_topic```.<br>
   *Type: String*<br>
   *Default: ""*
  - ```image_state```: State of the camera images used.<br>
    *Type: String*<br>
    *Default: 'DISTORTED'*<br>
    *Possible values:*<br>
      - *'DISTORTED': As it comes from the camera, i.e. image distortion has not yet been corrected and it is not rectified for stereo processing.*<br>
      - *'UNDISTORTED': Image with no distortion but not rectified for stereo processing.*<br>
      - *'STEREO_RECTIFIED': Image rectified for stereo processing, i.e. the epipolar lines are horizontally aligned.*
 - ```is_stereo_camera```: Set to true, if camera is to be calibrated as stereo camera. If set to true, 'right_camera_sensor_name' and 'right_camera_info_topic' also need to be set.<br>
    *Type: bool*<br>
    *Default: false*
 - ```right_camera_sensor_name```: Name of the right camera sensor when the camera is to be calibrated as a stereo camera system. Required if ```is_stereo_camera == true```.<br>
    *Type: String*<br>
    *Default: ""*
 - ```right_camera_info_topic```: Topic name of the camera info corresponding to the right camera. This is needed when the camera is to be calibrated as a stereo camera system. Required if ```is_stereo_camera == true```.<br>
    *Type: String*<br>
    *Default: ""*
 - ```rect_suffix```: Suffix of the of the right sensor name as well as the frame id for the rectified images. If the 'image_state' of the input images is DISTORTED or UNDISTORTED, this is added to the rectified frame id. If the imageState_ is STEREO_RECTIFIED this is removed from the frame id.<br>
    *Type: String*<br>
    *Default: "_rect"*
 - ```base_frame_id```: If specified, the extrinsic pose will be calculated with respect to frame of the given frame ID. This does not change the frame ID of the reference sensor, i.e. the LiDAR sensor, but will perform an a posteriori transformation of the estimated extrinsic pose into the specified frame. If not specified, or left empty, the extrinsic pose will be calculated with respect to the frame of the reference sensor.<br>
    *Type: String*<br>
    *Default: ""*
 - ```save_observations```: Option to save recorded observations that have been used for the calibration to the workspace<br>
    *type: bool*<br>
    *Default: false*

<hr>

### extrinsic_lidar_reference_calibration

The node for the [guided extrinsic calibration of a LiDAR sensor with respect to a reference](tutorial.md#extrinsic-lidar-reference-calibration) with a user-friendly UI.

It comprises

- the [ExtrinsicLidarReferenceCalibration(Nodelet)](#extrinsic_lidar_reference_calibration) performing the actual calibration,
- the [GuidedLidarLidarTargetPlacement(Nodelet)](#guidedlidarlidartargetplacementnodelet) responsible for guiding the user to place the calibration target, as well as
- an instance of 'LidarReferenceCalibrationGui' as a graphical user interface.

**Launch-Parameters:**

 - ```robot_ws_path```: Path to the folder holding the robot workspace. This will NOT be created if it does not yet exist. 
    See section ['Initialize new Robot workspace'](workspaces.md#initialize-new-robot-workspace) to find out how to create a new one<br>
    *Type: String*
 - ```target_config_file```: Path to the file holding the [configuration of the calibration target](calibration_target.md).<br>
      *E.g. "$(find multisensor_calibration)/config/TargetWithCirclesAndAruco.yaml"*<br>
    *Type: String*
 - ```src_lidar_sensor_name```: Name of the source lidar sensor that is to be calibrated.<br>
    *Type: String*
 - ```src_lidar_cloud_topic```: Topic name of the corresponding LiDAR cloud.<br>
    *Type: String*
 - ```reference_name```: Name of the reference
 calibrated.<br>
    *Type: String*
 - ```reference_frame_id```: Frame ID in which the reference data is provided.<br>
    *Type: String*
 - ```base_frame_id```: If specified, the extrinsic pose will be calculated with respect to frame of the given frame ID. This does not change the frame ID of the reference sensor, i.e. the LiDAR sensor, but will perform an a posteriori transformation of the estimated extrinsic pose into the specified frame. If not specified, or left empty, the extrinsic pose will be calculated with respect to the frame of the reference sensor.<br>
    *Type: String*<br>
    *Default: ""*
 - ```save_observations```: Option to save recorded observations that have been used for the calibration to the workspace<br>
    *type: bool*<br>
    *Default: false*

<hr>

### publish_pointcloud

Node to load a point cloud from a specified PLY file and publish it on the given topic with the given frame_id.

*This is helpful to publish the model cloud in the context of [extrinsic LiDAR-vehicle calibration](#extrinsiclidarvehiclecalibrationnodelet).

**Launch-Parameters:**

 - ```point_cloud_file```: PLY file from which to load the point cloud.<br>
    *Type: String*<br>
    *Default: ""*
 - ```topic_name```: Topic name on which to publish the point cloud.<br>
    *Type: String*<br>
    *Default: ""*
 - ```frame_id```: Frame ID at which to publish the point cloud.<br>
    *Type: String*<br>
    *Default: ""*

<hr>

### initialize_robot_workspace

Node to [initialize a non-existing robot workspace](workspaces.md#initialize-new-robot-workspace) with given information.

**Launch-Parameters:**

- ```robot_ws_path```: Path to where the robot workspace is to be initialized.<br>
   *Type: String*<br>
   *Default: ""*
- ```robot_name```: Name of the robot to which the workspace corresponds.<br>
   *Type: String*<br>
   *Default: ""*
- ```urdf_model_path```: (Optional) Path to URDF model associated with the robot.<br>
   *Type: String*<br>
   *Default: ""*

<hr>

## Composable Nodes / Nodelets

### CameraTargetDetection / CameraDataProcessingNodelet

Nodelet to run the processing of the camera data and, in turn, the detection of the calibration target within the camera data isolated from the rest.
This is particularly helpful to develop and debug the detection of the calibration target within the camera data.

**Launch-Parameters:**

 - ```camera```: Namespace of the camera.<br>
    *Type: String*<br>
    *Default: "/camera"*
 - ```image```: Name of the image topic within the camera namespace.<br>
    *Type: String*<br>
    *Default: "image_color"*
 - ```image_state```: State of the camera images used.<br>
    *Type: String*<br>
    *Default: 'DISTORTED'*<br>
    *Possible values:*<br>
      - *'DISTORTED': As it comes from the camera, i.e. image distortion has not yet been corrected and it is not rectified for stereo processing.*<br>
      - *'UNDISTORTED': Image with no distortion but not rectified for stereo processing.*<br>
      - *'STEREO_RECTIFIED': Image rectified for stereo processing, i.e. the epipolar lines are horizontally aligned.*
 - ```target_config_file```: Path to the file holding the [configuration of the calibration target](calibration_target.md).<br>
      *E.g. "$(find multisensor_calibration)/config/TargetWithCirclesAndAruco.yaml"*
  
**Topics Published:**

 - ```~/annotated_image```: Camera image, which is annotated with the detected markers of the calibration target.
 - ```~/target_pattern```: Cloud of the calibration target detected in the camera and reprojected into a 3D cloud. 
    This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/marker_corners```: Corners of the ArUco markers on the calibration target deduced from the detected pose of the target. 
    Each point of the marker corner is enhanced with the ID of the ArUco marker inside the ```intensity``` field. 
    This is  only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/board_pose```: 6-DOF pose of the detected calibration target. 
    This is only available after the calibration target has been detected in the LiDAR cloud.
  
**Services Advertised:**

  - ```~/request_camera_intrinsics```: Service to request the loaded intrinsics of the camera sensor.
  - ```~/capture_target```: Service to trigger the capturing of the target.
  - ```~/request_processor_state```: Service to get initialization state of the processor.
 
<hr>

### LidarTargetDetection / LidarDataProcessingNodelet

Nodelet to run the processing of the LiDAR data and, in turn, the detection of the calibration target within the LiDAR point clouds isolated from the rest.
This is particularly helpful to develop and debug the detection of the calibration target within the camera data.

**Launch-Parameters:**

 - ```cloud```: Topic name of the LiDAR cloud messages in which the target is to be detected.<br>
    *Type: String*<br>
    Default: "/cloud"*
 - ```target_config_file```: Path to the file holding the [configuration of the calibration target](calibration_target.md).<br>
      *E.g. "$(find multisensor_calibration)/config/TargetWithCirclesAndAruco.yaml"*

**Topics Published:**

 - ```~/regions_of_interest```: Cloud holding separate regions of the input sensor cloud in which the calibration target is searched for. 
    These are the product of the first preprocessing of the sensor cloud to reduce the amount of data to be processed.
 - ```~/target_pattern```: Cloud holding points of the calibration target detected in the LiDAR cloud. This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/marker_corners```: Corners of the ArUco markers on the calibration target deduced from the detected pose of the target. Each point of he marker corner is enhanced with the ID of the ArUco marker inside the 'intensity' field. This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/board_pose```: 6-DOF pose of the detected calibration target. This is only available after the calibration target has been detected in the LiDAR cloud.

**Services Advertised:**

  - ```~/capture_target```: Service to trigger the capturing of the target.
  - ```~/request_processor_state```: Service to get initialization state of the processor.
  - ```~/add_marker_observations```: Service to add the coordinates of the top-left corners of the ArUco marker. This is not relevant for this nodelet.
  - ```~/import_marker_observations```: Service to import a set of marker observations. This is not relevant for this nodelet.

<hr>

### ExtrinsicCameraLidarCalibration(Nodelet)

Nodelet to perform extrinsic camera-LiDAR calibration without any ui or guidance.

**How to use as standalone:**

1. After starting up, the calibration nodelet continuously detects the ArUco marker inside the camera image and selects possible region of interests inside the LiDAR cloud. It publishes the
marker detections as well as the regions of interest in the topic '~/<camera_sensor_name\>/annotated_image' and '~/<lidar_sensor_name\>/regions_of_interest', respectively.
2. Visualize the data published in the previous step for example in rviz and make sure that the calibration target is seen both in the camera image as well as the LiDAR scan.
3. To trigger the detection of the calibration target and to capture its pose, call the provided service of the calibration nodelet: '~/capture_target'.
4. Depending on the data, the detection of the target can take up to a couple of seconds.
5. When the target has successfully been detected in both the camera image as well as the LiDAR data, an instant calibration is performed with just this target pose. A corresponding message is printed in the command line with the mean reprojection error of this calibration. If not successful a corresponding message is also printed. Typical reasons for a detection not being successful is either that the target is not detected in the LiDAR scan or that the reprojection error exceeds a certain threshold (Default: 2px). If latter is the case the threshold can be increased by calling 'rqt_dynamic_reconfigure'.
6. Since the detection of the calibration target from the camera image is very reliable, only the detection inside the LiDAR cloud is published for visual inspection, i.e. inside the topics
'~/<lidar_sensor_name\>/target_pattern' and '~/<lidar_sensor_name\>/marker_corners'.
7. If the detected pose is not satisfactory one can remove the last detection by calling the appropriate service: '~/remove_last_observation'.
8. For a good calibration 5 to 7 different poses of the calibration target should be used, ideally moving it around the three coordinate axes. For each pose, repeat steps 2-7.
9. When enough different calibration poses are collected, the calibration can be finalized by calling the service: '~/finalize_calibration'. This will do a final calibration, print out the resulting extrinsic parameters as well as the overall mean reprojection error and the confidence of the calibration. The calibration results and a urdf snippet, together with the observations (if requested) are written into the calibration workspace.
NOTE: For a good final calibration a typical mean reprojection error is between 2-4 pixels.

**Launch-Parameters:**

 - ```robot_ws_path```: Path to the folder holding the robot workspace. This will NOT be created if it does not yet exist. 
    See section ['Initialize new Robot workspace'](workspaces.md#initialize-new-robot-workspace) to find out how to create a new one<br>
    *Type: String*
 - ```target_config_file```: Path to the file holding the [configuration of the calibration target](calibration_target.md).<br>
      *E.g. "$(find multisensor_calibration)/config/TargetWithCirclesAndAruco.yaml"*<br>
    *Type: String*
 - ```camera_sensor_name```: Name of the camera sensor that is to be calibrated.<br>
    *Type: String*
 - ```camera_image_topic```: Topic name of the corresponding camera images.<br>
    *Type: String*
 - ```lidar_sensor_name```: Name of the LiDAR sensor with respect to which the camera is to be
 calibrated.<br>
    *Type: String*
 - ```lidar_cloud_topic```: Topic name of the corresponding LiDAR cloud.<br>
    *Type: String*
 - ```camera_info_topic```: Name of the camera info topic. If this parameter is omitted the camera
   info topic name is constructed from the specified ```camera_image_topic```.<br>
   *Type: String*<br>
   *Default: ""*
  - ```image_state```: State of the camera images used.<br>
    *Type: String*<br>
    *Default: 'DISTORTED'*<br>
    *Possible values:*<br>
      - *'DISTORTED': As it comes from the camera, i.e. image distortion has not yet been corrected and it is not rectified for stereo processing.*<br>
      - *'UNDISTORTED': Image with no distortion but not rectified for stereo processing.*<br>
      - *'STEREO_RECTIFIED': Image rectified for stereo processing, i.e. the epipolar lines are horizontally aligned.*
 - ```is_stereo_camera```: Set to true, if camera is to be calibrated as stereo camera. If set to true, 'right_camera_sensor_name' and 'right_camera_info_topic' also need to be set.<br>
    *Type: bool*<br>
    *Default: false*
 - ```right_camera_sensor_name```: Name of the right camera sensor when the camera is to be calibrated as a stereo camera system. Required if ```is_stereo_camera == true```.<br>
    *Type: String*<br>
    *Default: ""*
 - ```right_camera_info_topic```: Topic name of the camera info corresponding to the right camera. This is needed when the camera is to be calibrated as a stereo camera system. Required if ```is_stereo_camera == true```.<br>
    *Type: String*<br>
    *Default: ""*
 - ```rect_suffix```: Suffix of the of the right sensor name as well as the frame id for the rectified images. If the 'image_state' of the input images is DISTORTED or UNDISTORTED, this is added to the rectified frame id. If the imageState_ is STEREO_RECTIFIED this is removed from the frame id.<br>
    *Type: String*<br>
    *Default: "_rect"*
 - ```base_frame_id```: If specified, the extrinsic pose will be calculated with respect to frame of the given frame ID. This does not change the frame ID of the reference sensor, i.e. the LiDAR sensor, but will perform an a posteriori transformation of the estimated extrinsic pose into the specified frame. If not specified, or left empty, the extrinsic pose will be calculated with respect to the frame of the reference sensor.<br>
    *Type: String*<br>
    *Default: ""*
 - ```use_initial_guess```: Option to use an initial guess on the extrinsic sensor pose from the TF-tree, if available.<br>
    *Type: bool*<br>
    *Default: false*
 - ```save_observations```: Option to save recorded observations that have been used for the calibration to the workspace<br>
    *type: bool*<br>
    *Default: false*
 - ```sync_queue_size```: Queue size used for the synchronization between the messages of the camera images and the LiDAR clouds<br>
    *Type: int*<br>
    *Default: 100*
 - ```use_exact_sync```: Set to true if an exact time synchronization between the camera image messages and the LiDAR cloud messages.<br>
    *Type: bool*<br>
    *Default: false*
  
**Topics Published:**

 - ```~/<camera_sensor_name>/annotated_image```: Camera image, which is annotated with the detected markers of the calibration target.
 - ```~/<camera_sensor_name>/target_pattern```: Cloud of the calibration target detected in the camera and reprojected into a 3D cloud. This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/<camera_sensor_name>/marker_corners```: Corners of the ArUco markers on the calibration target deduced from the detected pose of the target. Each point of he marker corner is enhanced with the ID of the ArUco marker inside the 'intensity' field. This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/<camera_sensor_name>/board_pose```: 6-DOF pose of the detected calibration target. This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/<lidar_sensor_name>/regions_of_interest```: Cloud holding separate regions of the input sensor cloud in which the calibration target is searched for. These are the product of the first preprocessing of the sensor cloud to reduce the amount of data to be processed.
 - ```~/<lidar_sensor_name>/target_pattern```: Cloud holding points of the calibration target detected in the LiDAR cloud. This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/<lidar_sensor_name>/marker_corners```: Corners of the ArUco markers on the calibration target deduced from the detected pose of the target. Each point of he marker corner is enhanced with the ID of the ArUco marker inside the 'intensity' field. This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/<lidar_sensor_name>/board_pose```: 6-DOF pose of the detected calibration target. This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/calibration_result```: Calibration result published after a successful calibration.

**Services Advertised:**

  - ```~/capture_target```: Service to trigger the capturing of the target in both sensors. This will add an observation to the list, if the target detection was successful.
  - ```~/remove_last_observation```: Service to remove last observation from the top of the list. This can be executed repeatedly until the observation list is empty.
  - ```~/finalize_calibration```: Service to finalize calibration, i.e. calibrate the extrinsic pose based on all captured observations.
  - ```~/reset```: Service to reset the calibration.
  - ```~/request_calibration_meta_data```: Service to request the meta information of the calibration, e.g. sensor names, topic names, and more.
  - ```~/request_sensor_extrinsics```: Service to request the currently calculated extrinsic pose between the two sensors.
  - ```~/request_camera_intrinsics```: Service to request the loaded intrinsics of the camera sensor.
  - ```~/<lidar_sensor_name>/add_marker_observations```: Service to add the coordinates of the top-left corners of the ArUco marker to the LiDAR data as reference.
  - ```~/<lidar_sensor_name>/import_marker_observations```: Service to import a set of the top-left corners of the ArUco marker to the LiDAR data as reference. This will remove all previous observations captured for this sensor.

<hr>

### ExtrinsicCameraCameraCalibration(Nodelet)

Nodelet to perform extrinsic camera-Camera calibration without any ui or guidance.

**How to use as standalone:**

1. After starting up, the calibration nodelet continuously detects the ArUco marker inside the cameras' images. It publishes the
marker detections.
2. Visualize the data published in the previous step for example in rviz and make sure that the calibration target is seen in boths cameras.
3. To trigger the detection of the calibration target and to capture its pose, call the provided service of the calibration nodelet: '~/capture_target'.
4. Depending on the data, the detection of the target can take up to a couple of seconds.
5. When the target has successfully been detected, an instant calibration is performed with just this target pose. A corresponding message is printed in the command line with the mean reprojection error of this calibration. If not successful a corresponding message is also printed. Typical reasons for a detection not being successful is either that the target is not detected in both sensors or that the reprojection error exceeds a certain threshold (Default: 2px). If latter is the case the threshold can be increased by calling 'rqt_dynamic_reconfigure'.
6. If the detected pose is not satisfactory one can remove the last detection by calling the appropriate service: '~/remove_last_observation'.
7. For a good calibration 5 to 7 different poses of the calibration target should be used, ideally moving it around the three coordinate axes. For each pose, repeat steps 2-6.
8. When enough different calibration poses are collected, the calibration can be finalized by calling the service: '~/finalize_calibration'. This will do a final calibration, print out the resulting extrinsic parameters as well as the overall mean reprojection error and the confidence of the calibration. The calibration results and a urdf snippet, together with the observations (if requested) are written into the calibration workspace.
NOTE: For a good final calibration a typical mean reprojection error is between 2-4 pixels.

**Launch-Parameters:**

 - ```robot_ws_path```: Path to the folder holding the robot workspace. This will NOT be created if it does not yet exist. 
    See section ['Initialize new Robot workspace'](workspaces.md#initialize-new-robot-workspace) to find out how to create a new one<br>
    *Type: String*
 - ```target_config_file```: Path to the file holding the [configuration of the calibration target](calibration_target.md).<br>
      *E.g. "$(find multisensor_calibration)/config/TargetWithCirclesAndAruco.yaml"*<br>
    *Type: String*
 - ```src_camera_sensor_name```: Name of the camera sensor that is to be calibrated.<br>
    *Type: String*
 - ```src_camera_image_topic```: Topic name of the corresponding camera images.<br>
    *Type: String*
 - ```src_camera_info_topic```: Name of the camera info topic. If this parameter is omitted the camera
   info topic name is constructed from the specified ```camera_image_topic```.<br>
   *Type: String*<br>
   *Default: ""*
  - ```image_state```: State of the camera images used.<br>
    *Type: String*<br>
    *Default: 'DISTORTED'*<br>
    *Possible values:*<br>
      - *'DISTORTED': As it comes from the camera, i.e. image distortion has not yet been corrected and it is not rectified for stereo processing.*<br>
      - *'UNDISTORTED': Image with no distortion but not rectified for stereo processing.*<br>
      - *'STEREO_RECTIFIED': Image rectified for stereo processing, i.e. the epipolar lines are horizontally aligned.*
 - ```ref_camera_sensor_name```: Name of the camera sensor that is to be calibrated.<br>
    *Type: String*
 - ```ref_camera_image_topic```: Topic name of the corresponding camera images.<br>
    *Type: String*
 - ```ref_camera_info_topic```: Name of the camera info topic. If this parameter is omitted the camera
   info topic name is constructed from the specified ```camera_image_topic```.<br>
   *Type: String*<br>
   *Default: ""*
  - ```image_state```: State of the camera images used.<br>
    *Type: String*<br>
    *Default: 'DISTORTED'*<br>
    *Possible values:*<br>
      - *'DISTORTED': As it comes from the camera, i.e. image distortion has not yet been corrected and it is not rectified for stereo processing.*<br>
      - *'UNDISTORTED': Image with no distortion but not rectified for stereo processing.*<br>
      - *'STEREO_RECTIFIED': Image rectified for stereo processing, i.e. the epipolar lines are horizontally aligned.*
 - ```base_frame_id```: If specified, the extrinsic pose will be calculated with respect to frame of the given frame ID. This does not change the frame ID of the reference sensor, i.e. the LiDAR sensor, but will perform an a posteriori transformation of the estimated extrinsic pose into the specified frame. If not specified, or left empty, the extrinsic pose will be calculated with respect to the frame of the reference sensor.<br>
    *Type: String*<br>
    *Default: ""*
 - ```use_initial_guess```: Option to use an initial guess on the extrinsic sensor pose from the TF-tree, if available.<br>
    *Type: bool*<br>
    *Default: false*
 - ```save_observations```: Option to save recorded observations that have been used for the calibration to the workspace<br>
    *type: bool*<br>
    *Default: false*
 - ```sync_queue_size```: Queue size used for the synchronization between the messages of the camera images and the LiDAR clouds<br>
    *Type: int*<br>
    *Default: 100*
 - ```use_exact_sync```: Set to true if an exact time synchronization between the camera image messages and the LiDAR cloud messages.<br>
    *Type: bool*<br>
    *Default: false*
  
**Topics Published:**

 - ```~/<src_camera_sensor_name>/annotated_image```: Camera image, which is annotated with the detected markers of the calibration target.
 - ```~/<src_camera_sensor_name>/target_pattern```: Cloud of the calibration target detected in the camera and reprojected into a 3D cloud.
 - ```~/<src_camera_sensor_name>/marker_corners```: Corners of the ArUco markers on the calibration target deduced from the detected pose of the target. Each point of he marker corner is enhanced with the ID of the ArUco marker inside the 'intensity' field.
 - ```~/<src_camera_sensor_name>/board_pose```: 6-DOF pose of the detected calibration target.
 - ```~/<ref_camera_sensor_name>/regions_of_interest```: Cloud holding separate regions of the input sensor cloud in which the calibration target is searched for. These are the product of the first preprocessing of the sensor cloud to reduce the amount of data to be processed.
 - ```~/<ref_camera_sensor_name>/target_pattern```: same as above
 - ```~/<ref_camera_sensor_name>/marker_corners```: same as above
 - ```~/<ref_camera_sensor_name>/board_pose```: same as above
 - ```~/calibration_result```: Calibration result published after a successful calibration.

**Services Advertised:**

  - ```~/capture_target```: Service to trigger the capturing of the target in both sensors. This will add an observation to the list, if the target detection was successful.
  - ```~/remove_last_observation```: Service to remove last observation from the top of the list. This can be executed repeatedly until the observation list is empty.
  - ```~/finalize_calibration```: Service to finalize calibration, i.e. calibrate the extrinsic pose based on all captured observations.
  - ```~/reset```: Service to reset the calibration.
  - ```~/request_calibration_meta_data```: Service to request the meta information of the calibration, e.g. sensor names, topic names, and more.
  - ```~/request_sensor_extrinsics```: Service to request the currently calculated extrinsic pose between the two sensors.
  - ```~/request_camera_intrinsics```: Service to request the loaded intrinsics of the src camera sensor.

<hr>

### ExtrinsicLidarLidarCalibration(Nodelet)

Nodelet to perform extrinsic LiDAR-LiDAR calibration without any ui or guidance.

**How to use as standalone:**


1. After starting up, the calibration nodelet continuously selects possible region of interests inside both LiDAR clouds. It publishes the marker detections as well as the regions of interest in the topics '~/<[src|ref]_lidar_sensor_name>/regions_of_interest'.
2. Visualize the data published in the previous step for example in rviz and make sure that the calibration target is seen both in the camera image as well as the LiDAR scan.
3. To trigger the detection of the calibration target and to capture its pose, call the provided service of the calibration nodelet: '~/capture_target'.
4. Depending on the data, the detection of the target can take up to a couple of seconds.
5. When the target has successfully been detected in both LiDAR data, an instant calibration is performed with just this target pose. A corresponding message is printed in the command line with the mean reprojection error of this calibration. If not successful a corresponding message is also printed. Typical reasons for a detection not being successful is that the target is not detected in the LiDAR scan.
6. The detection inside the LiDAR cloud is published for visual inspection, i.e. inside the topics '~/<[src|ref]_lidar_sensor_name>/target_pattern' and '~/<[src|ref]_lidar_sensor_name>/marker_corners'.
7. If the detected pose is not satisfactory one can remove the last detection by calling the appropriate service: '~/remove_last_observation'.
8. For a good calibration 3 to 5 different poses of the calibration target should be used, ideally moving it around the three coordinate axes. For each pose, repeat steps 2-7.
9. When enough different calibration poses are collected, the calibration can be finalized by calling the service: '~/finalize_calibration'. This will do a final calibration, print out the resulting extrinsic parameters as well as the overall root mean squared error and the confidence of the calibration. The calibration results and a urdf snippet, together with the observations (if requested) are written into the calibration workspace.

**Launch-Parameters:**

 - ```robot_ws_path```: Path to the folder holding the robot workspace. This will NOT be created if it does not yet exist. 
    See section ['Initialize new Robot workspace'](workspaces.md#initialize-new-robot-workspace) to find out how to create a new one<br>
    *Type: String*
 - ```target_config_file```: Path to the file holding the [configuration of the calibration target](calibration_target.md).<br>
      *E.g. "$(find multisensor_calibration)/config/TargetWithCirclesAndAruco.yaml"*<br>
    *Type: String*
 - ```src_lidar_sensor_name```: Name of the source lidar sensor that is to be calibrated.<br>
    *Type: String*
 - ```src_lidar_cloud_topic```: Topic name of the corresponding LiDAR cloud.<br>
    *Type: String*
 - ```ref_lidar_sensor_name```: ame of the reference LiDAR sensor with respect to which the source LiDAR sensor is to be calibrated.<br>
    *Type: String*
 - ```ref_lidar_cloud_topic```: Topic name of the corresponding LiDAR cloud.<br>
    *Type: String*
 - ```base_frame_id```: If specified, the extrinsic pose will be calculated with respect to frame of the given frame ID. This does not change the frame ID of the reference sensor, i.e. the LiDAR sensor, but will perform an a posteriori transformation of the estimated extrinsic pose into the specified frame. If not specified, or left empty, the extrinsic pose will be calculated with respect to the frame of the reference sensor.<br>
    *Type: String*<br>
    *Default: ""*
 - ```use_initial_guess```: Option to use an initial guess on the extrinsic sensor pose from the TF-tree, if available.<br>
    *Type: bool*<br>
    *Default: false*
 - ```align_ground_planes```: Set to true, to additionally align the ground planes in the sensor data. Additionally specify the upright frame ID.
    *Type: bool*
    *Default: false*
 - ```upright_frame_id```: ID of Frame which has an upwards pointing z-axis. Used to detect ground plane in sensor data.
    *Type: String*
    *Default: ""*
 - ```save_observations```: Option to save recorded observations that have been used for the calibration to the workspace<br>
    *type: bool*<br>
    *Default: false*
 - ```sync_queue_size```: Queue size used for the synchronization between the messages of the camera images and the LiDAR clouds<br>
    *Type: int*<br>
    *Default: 100*
 - ```use_exact_sync```: Set to true if an exact time synchronization between the camera image messages and the LiDAR cloud messages.<br>
    *Type: bool*<br>
    *Default: false*
  
**Topics Published:**

 - ```~/<src_lidar_sensor_name>/regions_of_interest```: Cloud holding separate regions of the source LiDAR cloud in which the calibration target is searched for. These are the product of the first preprocessing of the sensor cloud to reduce the amount of data to be processed.
 - ```~/<src_lidar_sensor_name>/board_pose```: 6-DOF pose of the detected calibration target in the source LiDAR cloud. This is only available after the calibration target has been detected in the source LiDAR cloud.
 - ```~/<src_lidar_sensor_name>/target_pattern```: Cloud holding points of the calibration target detected in the source LiDAR cloud. This is only available after the calibration target has been detected in the source LiDAR cloud.
 - ```~/<src_lidar_sensor_name>/marker_corners```: Corners of the ArUco markers on the calibration target deduced from the detected pose of the target. Each point of the marker corner is enhanced with the ID of the ArUco marker inside the 'intensity' field. This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/<ref_lidar_sensor_name>/regions_of_interest```: Cloud holding separate regions of the input reference LiDAR cloud in which the calibration target is searched for. These are the product of the first preprocessing of the sensor cloud to reduce the amount of data to be processed.
 - ```~/<ref_lidar_sensor_name>/board_pose```: 6-DOF pose of the detected calibration target in the reference LiDAR cloud. This is only available after the calibration target has been detected in the reference LiDAR cloud.
 - ```~/<ref_lidar_sensor_name>/target_pattern```: Cloud holding points of the calibration target detected in the reference LiDAR cloud. This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/<ref_lidar_sensor_name>/marker_corners```: Corners of the ArUco markers on the calibration target deduced from the detected pose of the target. Each point of the marker corner is enhanced with the ID of the ArUco marker inside the 'intensity' field. This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/calibration_result```: Calibration result published after a successful calibration.

**Services Advertised:**

  - ```~/capture_target```: Service to trigger the capturing of the target in both sensors. This will add an observation to the list, if the target detection was successful.
  - ```~/remove_last_observation```: Service to remove last observation from the top of the list. This can be executed repeatedly until the observation list is empty.
  - ```~/finalize_calibration```: Service to finalize calibration, i.e. calibrate the extrinsic pose based on all captured observations.
  - ```~/reset```: Service to reset the calibration.
  - ```~/request_calibration_meta_data```: Service to request the meta information of the calibration, e.g. sensor names, topic names, and more.
  - ```~/request_sensor_extrinsics```: Service to request the currently calculated extrinsic pose between the two sensors.
  - ```~/<src_lidar_sensor_name>/add_marker_observations```: Service to add the coordinates of the top-left corners of the ArUco marker to the source LiDAR data.
  - ```~/<src_lidar_sensor_name>/import_marker_observations```: Service to import a set of the top-left corners of the ArUco marker to the source LiDAR data. This will remove all previous observations captured for this sensor.
  - ```~/<ref_lidar_sensor_name>/add_marker_observations```: Service to add the coordinates of the top-left corners of the ArUco marker to the reference LiDAR data.
  - ```~/<ref_lidar_sensor_name>/import_marker_observations```: Service to import a set of the top-left corners of the ArUco marker to the reference LiDAR data. This will remove all previous observations captured for this sensor.

<hr>

### ExtrinsicCameraReferenceCalibration(Nodelet)

Nodelet to perform extrinsic camera-reference calibration without any ui or guidance.

**How to use as standalone:**

1. After starting up, the calibration nodelet continuously detects the ArUco marker inside the camera image. It publishes the marker detections in the topic '~/<camera_sensor_name\>/annotated_image', respectively.
2. Visualize the data published in the previous step for example in rviz and make sure that the calibration target is seen both in the camera image as well as the LiDAR scan.
3. To trigger the detection of the calibration target and to capture its pose in the camera image, call the provided service of the calibration nodelet: '~/capture_target'.
4. Depending on the data, the detection of the target can take up to a couple of seconds.
5. When the target has successfully been detected a corresponding message is printed in the command line. If not successful a corresponding message is also printed.
6. If the detected pose is not satisfactory one can remove the last detection by calling the appropriate service: '~/remove_last_observation'.
7. Measure the top-left corner of each ArUco marker, for example, with a Total Station.
8. Pass all measured coordinates of the top-left marker corners to the calibration by calling 'ros2 service call /<node_name\>/<reference_name/>/add_marker_observations iosb_calibration_interface/srv/AddMarkerObservations "{ observation: { marker_ids: [1, 2, 3, 4], marker_top_left_point: [{x: 1.0, y: 2.0, z: 3.0}, {x: 1.0, y: 2.0, z: 3.0}, {x: 1.0, y: 2.0, z: 3.0}, {x: 1.0, y: 2.0, z: 3.0}]} }'.
9. For a good calibration 5 to 7 different poses of the calibration target should be used, ideally moving it around the three coordinate axes. For each pose, repeat steps 2-8.
10. When enough different calibration poses are collected, the calibration can be finalized by calling the service: '~/finalize_calibration'. This will do a final calibration, print out the resulting extrinsic parameters as well as the overall mean reprojection error and the confidence of the calibration. The calibration results and a urdf snippet, together with the observations (if requested) are written into the calibration workspace. For a good final calibration a typical mean reprojection error is between 2-4 pixels.

**NOTE:** Step 8 (entering of the reference coordinates) does not necessarily need to be done directly after each measuring of the coordinates. It is can also be done iteratively for all observations before Step 10. However, it is important that the measurements are added in the correct order.

**Launch-Parameters:**

 - ```robot_ws_path```: Path to the folder holding the robot workspace. This will NOT be created if it does not yet exist. 
    See section ['Initialize new Robot workspace'](workspaces.md#initialize-new-robot-workspace) to find out how to create a new one<br>
    *Type: String*
 - ```target_config_file```: Path to the file holding the [configuration of the calibration target](calibration_target.md).<br>
      *E.g. "$(find multisensor_calibration)/config/TargetWithCirclesAndAruco.yaml"*<br>
    *Type: String*
 - ```camera_sensor_name```: Name of the camera sensor that is to be calibrated.<br>
    *Type: String*
 - ```camera_image_topic```: Topic name of the corresponding camera images.<br>
    *Type: String*
 - ```reference_name```: Name of the reference
 calibrated.<br>
    *Type: String*
 - ```reference_frame_id```: Frame ID in which the reference data is provided.<br>
    *Type: String*
 - ```camera_info_topic```: Name of the camera info topic. If this parameter is omitted the camera
   info topic name is constructed from the specified ```camera_image_topic```.<br>
   *Type: String*<br>
   *Default: ""*
  - ```image_state```: State of the camera images used.<br>
    *Type: String*<br>
    *Default: 'DISTORTED'*<br>
    *Possible values:*<br>
      - *'DISTORTED': As it comes from the camera, i.e. image distortion has not yet been corrected and it is not rectified for stereo processing.*<br>
      - *'UNDISTORTED': Image with no distortion but not rectified for stereo processing.*<br>
      - *'STEREO_RECTIFIED': Image rectified for stereo processing, i.e. the epipolar lines are horizontally aligned.*
 - ```is_stereo_camera```: Set to true, if camera is to be calibrated as stereo camera. If set to true, 'right_camera_sensor_name' and 'right_camera_info_topic' also need to be set.<br>
    *Type: bool*<br>
    *Default: false*
 - ```right_camera_sensor_name```: Name of the right camera sensor when the camera is to be calibrated as a stereo camera system. Required if ```is_stereo_camera == true```.<br>
    *Type: String*<br>
    *Default: ""*
 - ```right_camera_info_topic```: Topic name of the camera info corresponding to the right camera. This is needed when the camera is to be calibrated as a stereo camera system. Required if ```is_stereo_camera == true```.<br>
    *Type: String*<br>
    *Default: ""*
 - ```rect_suffix```: Suffix of the of the right sensor name as well as the frame id for the rectified images. If the 'image_state' of the input images is DISTORTED or UNDISTORTED, this is added to the rectified frame id. If the imageState_ is STEREO_RECTIFIED this is removed from the frame id.<br>
    *Type: String*<br>
    *Default: "_rect"*
 - ```base_frame_id```: If specified, the extrinsic pose will be calculated with respect to frame of the given frame ID. This does not change the frame ID of the reference sensor, i.e. the LiDAR sensor, but will perform an a posteriori transformation of the estimated extrinsic pose into the specified frame. If not specified, or left empty, the extrinsic pose will be calculated with respect to the frame of the reference sensor.<br>
    *Type: String*<br>
    *Default: ""*
 - ```save_observations```: Option to save recorded observations that have been used for the calibration to the workspace<br>
    *type: bool*<br>
    *Default: false*
  
**Topics Published:**

 - ```~/<camera_sensor_name>/annotated_image```: Camera image, which is annotated with the detected markers of the calibration target.
 - ```~/<camera_sensor_name>/target_pattern```: Cloud of the calibration target detected in the camera and reprojected into a 3D cloud. This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/<camera_sensor_name>/marker_corners```: Corners of the ArUco markers on the calibration target deduced from the detected pose of the target. Each point of he marker corner is enhanced with the ID of the ArUco marker inside the 'intensity' field. This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/<camera_sensor_name>/board_pose```: 6-DOF pose of the detected calibration target. This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/calibration_result```: Calibration result published after a successful calibration.

**Services Advertised:**

  - ```~/capture_target```: Service to trigger the capturing of the target in both sensors. This will add an observation to the list, if the target detection was successful.
  - ```~/remove_last_observation```: Service to remove last observation from the top of the list. This can be executed repeatedly until the observation list is empty.
  - ```~/finalize_calibration```: Service to finalize calibration, i.e. calibrate the extrinsic pose based on all captured observations.
  - ```~/reset```: Service to reset the calibration.
  - ```~/request_calibration_meta_data```: Service to request the meta information of the calibration, e.g. sensor names, topic names, and more.
  - ```~/request_sensor_extrinsics```: Service to request the currently calculated extrinsic pose between the two sensors.
  - ```~/request_camera_intrinsics```: Service to request the loaded intrinsics of the camera sensor.
  - ```~/<reference_name>/add_marker_observations```: Service to add the coordinates of the top-left corners of the ArUco marker to the LiDAR data as reference.
  - ```~/<reference_name>/import_marker_observations```: Service to import a set of the top-left corners of the ArUco marker to the LiDAR data as reference. This will remove all previous observations captured for this sensor.

<hr>

### ExtrinsicLidarReferenceCalibration(Nodelet)

Nodelet to perform extrinsic lidar-reference calibration without any ui or guidance.

**How to use as standalone:**

1. After starting up, the calibration nodelet continuously selects possible region of interests inside the LiDAR cloud. It publishes the marker detections as well as the regions of interest in the topics '~/<src_lidar_sensor_name>/regions_of_interest'.
2. Visualize the data published in the previous step for example in rviz and make sure that the calibration target is seen both in the camera image as well as the LiDAR scan.
3. To trigger the detection of the calibration target and to capture its pose in the camera image, call the provided service of the calibration nodelet: '~/capture_target'.
4. Depending on the data, the detection of the target can take up to a couple of seconds.
5. When the target has successfully been detected a corresponding message is printed in the command line. If not successful a corresponding message is also printed.
6. If the detected pose is not satisfactory one can remove the last detection by calling the appropriate service: '~/remove_last_observation'.
7. Measure the top-left corner of each ArUco marker, for example, with a Total Station.
8. Pass all measured coordinates of the top-left marker corners to the calibration by calling 'ros2 service call /<node_name\>/<reference_name/>/add_marker_observations iosb_calibration_interface/srv/AddMarkerObservations "{ observation: { marker_ids: [1, 2, 3, 4], marker_top_left_point: [{x: 1.0, y: 2.0, z: 3.0}, {x: 1.0, y: 2.0, z: 3.0}, {x: 1.0, y: 2.0, z: 3.0}, {x: 1.0, y: 2.0, z: 3.0}]} }'.
9. For a good calibration 3 to 5 different poses of the calibration target should be used, ideally moving it around the three coordinate axes. For each pose, repeat steps 2-8.
10. When enough different calibration poses are collected, the calibration can be finalized by calling the service: '~/finalize_calibration'. This will do a final calibration, print out the resulting extrinsic parameters as well as the overall root mean squared error and the confidence of the calibration. The calibration results and a urdf snippet, together with the observations (if requested) are written into the calibration workspace. 

**NOTE:** Step 8 (entering of the reference coordinates) does not necessarily need to be done directly after each measuring of the coordinates. It is can also be done iteratively for all observations before Step 10. However, it is important that the measurements are added in the correct order.

**Launch-Parameters:**

 - ```robot_ws_path```: Path to the folder holding the robot workspace. This will NOT be created if it does not yet exist. 
    See section ['Initialize new Robot workspace'](workspaces.md#initialize-new-robot-workspace) to find out how to create a new one<br>
    *Type: String*
 - ```target_config_file```: Path to the file holding the [configuration of the calibration target](calibration_target.md).<br>
      *E.g. "$(find multisensor_calibration)/config/TargetWithCirclesAndAruco.yaml"*<br>
    *Type: String*
 - ```src_lidar_sensor_name```: Name of the source lidar sensor that is to be calibrated.<br>
    *Type: String*
 - ```src_lidar_cloud_topic```: Topic name of the corresponding LiDAR cloud.<br>
    *Type: String*
 - ```reference_name```: Name of the reference
 calibrated.<br>
    *Type: String*
 - ```reference_frame_id```: Frame ID in which the reference data is provided.<br>
    *Type: String*
 - ```base_frame_id```: If specified, the extrinsic pose will be calculated with respect to frame of the given frame ID. This does not change the frame ID of the reference sensor, i.e. the LiDAR sensor, but will perform an a posteriori transformation of the estimated extrinsic pose into the specified frame. If not specified, or left empty, the extrinsic pose will be calculated with respect to the frame of the reference sensor.<br>
    *Type: String*<br>
    *Default: ""*
 - ```save_observations```: Option to save recorded observations that have been used for the calibration to the workspace<br>
    *type: bool*<br>
    *Default: false*
  
**Topics Published:**

 - ```~/<src_lidar_sensor_name>/regions_of_interest```: Cloud holding separate regions of the source LiDAR cloud in which the calibration target is searched for. These are the product of the first preprocessing of the sensor cloud to reduce the amount of data to be processed.
 - ```~/<src_lidar_sensor_name>/board_pose```: 6-DOF pose of the detected calibration target in the source LiDAR cloud. This is only available after the calibration target has been detected in the source LiDAR cloud.
 - ```~/<src_lidar_sensor_name>/target_pattern```: Cloud holding points of the calibration target detected in the source LiDAR cloud. This is only available after the calibration target has been detected in the source LiDAR cloud.
 - ```~/<src_lidar_sensor_name>/marker_corners```: Corners of the ArUco markers on the calibration target deduced from the detected pose of the target. Each point of the marker corner is enhanced with the ID of the ArUco marker inside the 'intensity' field. This is only available after the calibration target has been detected in the LiDAR cloud.
 - ```~/calibration_result```: Calibration result published after a successful calibration.

**Services Advertised:**

  - ```~/capture_target```: Service to trigger the capturing of the target in both sensors. This will add an observation to the list, if the target detection was successful.
  - ```~/remove_last_observation```: Service to remove last observation from the top of the list. This can be executed repeatedly until the observation list is empty.
  - ```~/finalize_calibration```: Service to finalize calibration, i.e. calibrate the extrinsic pose based on all captured observations.
  - ```~/reset```: Service to reset the calibration.
  - ```~/request_calibration_meta_data```: Service to request the meta information of the calibration, e.g. sensor names, topic names, and more.
  - ```~/request_sensor_extrinsics```: Service to request the currently calculated extrinsic pose between the two sensors.
  - ```~/<src_lidar_sensor_name>/add_marker_observations```: Service to add the coordinates of the top-left corners of the ArUco marker to the source LiDAR data.
  - ```~/<src_lidar_sensor_name>/import_marker_observations```: Service to import a set of the top-left corners of the ArUco marker to the source LiDAR data. This will remove all previous observations captured for this sensor.
  - ```~/<reference_name>/add_marker_observations```: Service to add the coordinates of the top-left corners of the ArUco marker to the LiDAR data as reference.
  - ```~/<reference_name>/import_marker_observations```: Service to import a set of the top-left corners of the ArUco marker to the LiDAR data as reference. This will remove all previous observations captured for this sensor.

<hr>

### ExtrinsicLidarVehicleCalibration(Nodelet)

Nodelet to perform extrinsic LiDAR-LiDAR calibration without any ui.
The idea in this calibration is that for which have LiDAR sensors that also "see" part of the robots body, on can estimate the position of the LiDAR sensor by aligning the LiDAR scans to a model of the robots body.
This is done by manually selecting corresponding regions in both sensor cloud and model. To do so, use the point picking tool of rviz to pick a point in the corresponding clouds round which a planar region is to be estimated. The selected regions are then aligned using a GICP. For region selection a plane is fitted into the cloud at a selected marker point using RANSAC. Prior to GICP the clouds are filtered using Statistical Outlier Removal. The transformation guess for the GICP is computed by using a Levenberg-Marquardt algorithm to estimate a rigid transformation from the seed points around the selected region markers.
**NOTE:** This work, however, is a prototype implementation and needs further development.

**Launch-Parameters:**

 - ```robot_ws_path```: Path to the folder holding the robot workspace. This will NOT be created if it does not yet exist. 
    See section ['Initialize new Robot workspace'](workspaces.md#initialize-new-robot-workspace) to find out how to create a new one<br>
    *Type: String*
 - ```target_config_file```: Path to the file holding the [configuration of the calibration target](calibration_target.md).<br>
      *E.g. "$(find multisensor_calibration)/config/TargetWithCirclesAndAruco.yaml"*<br>
    *Type: String*
 - ```src_lidar_sensor_name```: Name of the source lidar sensor that is to be calibrated.<br>
    *Type: String*
 - ```src_lidar_cloud_topic```: Topic name of the corresponding LiDAR cloud.<br>
    *Type: String*
 - ```ref_lidar_cloud_topic```: Topic name of the reference LiDAR cloud from CAD model.<br>
    *Type: String*
 - ```base_frame_id```: If specified, the extrinsic pose will be calculated with respect to frame of the given frame ID. This does not change the frame ID of the reference sensor, i.e. the LiDAR sensor, but will perform an a posteriori transformation of the estimated extrinsic pose into the specified frame. If not specified, or left empty, the extrinsic pose will be calculated with respect to the frame of the reference sensor.<br>
    *Type: String*<br>
    *Default: ""*
 - ```use_initial_guess```: Option to use an initial guess on the extrinsic sensor pose from the TF-tree, if available.<br>
    *Type: bool*<br>
    *Default: false*
 - ```save_observations```: Option to save recorded observations that have been used for the calibration to the workspace<br>
    *type: bool*<br>
    *Default: false*

**Topics Published:**

 - ```~/<src_lidar_sensor_name>/regions_of_interest```: Cloud holding regions of the source sensor which are used to align the sensor cloud to the cloud of the vehicle model. These are computed based on the seed points clicked.
 - ```~/<ref_lidar_sensor_name>/regions_of_interest```: Cloud holding regions of the source sensor which are used to align the sensor cloud to the cloud of the vehicle model. These are computed based on the seed points clicked.
 - ```~/calibration_result```: Calibration result published after a successful calibration.

**Services Advertised:**

  - ```~/remove_last_observation```: Service to remove last observation from the top of the list. This can be executed repeatedly until the observation list is empty.
  - ```~/finalize_calibration```: Service to finalize calibration, i.e. calibrate the extrinsic pose based on all captured observations.
  - ```~/reset```: Service to reset the calibration.
  - ```~/request_calibration_meta_data```: Service to request the meta information of the calibration, e.g. sensor names, topic names, and more.
  - ```~/request_sensor_extrinsics```: Service to request the currently calculated extrinsic pose between the two sensors.
  - ```~/add_region_marker```: Service to a new seed point for a region which is later to be used to align the sensor cloud to that of the vehicle cad. Use the frame ID to distinguish between sensor cloud and vehicle cloud.

**Subscribed Topics:**

   - ```clicked_point```: Message topic provided by rviz when a point is clicked.

<hr>

### GuidedCameraLidarTargetPlacement(Nodelet)

Nodelet for guided target placement in the context of extrinsic camera-lidar calibration.

**NOTE: **The guidance feature of multisensor_calibration is currently still in development and not yet fully functional.

<hr>

### GuidedLidarLidarTargetPlacement(Nodelet)

Nodelet for guided target placement in the context of extrinsic lidar-lidar calibration.

**NOTE: **The guidance feature of multisensor_calibration is currently still in development and not yet fully functional.

<hr>

### PointCloud2Image(Nodelet)

Nodelet to fuse a point cloud and a camera image by projecting the geometric 3D information from the point cloud into the camera image.
In this, the projected points from the point cloud are colorized based on their depth, i.e. distance from the camera. The colorization is performed by applying the RAINBOW colormap from
[`cv::ColorMap`](https://docs.opencv.org/master/d3/d50/group__imgproc__colormap.html). This colorizes the range from `min_depth` to `max_depth` going from red to violet.

**Launch-Parameters:**

- ```image_state```: State of the camera images used.<br>
    *Type: String*<br>
    *Default: 'DISTORTED'*<br>
    *Possible values:*<br>
      - *'DISTORTED': As it comes from the camera, i.e. image distortion has not yet been corrected and it is not rectified for stereo processing.*<br>
      - *'UNDISTORTED': Image with no distortion but not rectified for stereo processing.*<br>
      - *'STEREO_RECTIFIED': Image rectified for stereo processing, i.e. the epipolar lines are horizontally aligned.*
- ```camera_namespace```: Optional parameter to provide a separate camera namespace from which the topic name of the camera info can be constructed.<br>
    *Type: String*<br>
    *Default: ""*
- ```min_depth```: Minimum depth for which to project points from the point cloud into the image. All points that are closer to the camera than the specified min. depth will be discarded.<br>
  *Type: float*<br>
  *Default: 0.1*
- ```max_depth```: Maximum depth for which to project points from the point cloud into the image. All points that are further away from the camera than the specified max. depth will be discarded.<br>
  *Type: float*<br>
  *Default: 20.0*
- ```sync_queue_size`: Queue size for the data synchronization using the method approximated time synchronization. Not evaluated when using exact time synchronization.<br>
  *Type: int*<br>
  *Default: 100*
- ```use_exact_sync```: Flag to activate method for exact time synchronization.<br>
  *Type: bool*
  *Default: *false*
- `temp_transform`: Temporary transform of camera with respect to LiDAR given in the form of XYZ and RPY. The values are to be given as string list separated by whitespace. If left empty or omitted the transformation will be extracted from the TF-tree by using the frame IDs of the images and the clouds.<br>
  *Type: String*
  *Default: ""*

**Subscribed Topics:**

- ```pointcloud```: Subscribes to point cloud that is to be projected into the camera image.<br>
    *Type: sensor_msgs::PointCloud2*

- ```image```: Subscribes to RGB input image from which to draw the pixel information to enhance the point cloud with color information.<br>
    *Type: sensor_msgs::Image*

**Published Topics:**

- ```~/fused_image```: Publishes the resulting RGB image in which the input camera vis image is fused with the input point cloud. The projected points from the point cloud are colorized based on their depth, i.e. distance from the camera.<br>
    *Type: sensor_msgs::Image*

<hr>

### PointCloud2PointCloudDistance(Nodelet)

Nodelet to calculate the n-to-n distance for a list of point clouds.
For each point cloud in the list, the smallest distance with respect to all other point clouds is found.
The calculated distance is normalized with respect to a given maximum distance and stored in the intensity field of the source point cloud.
This can then be used to colorize the distance in a point cloud viewer, such as rviz.

To calculate the distance between the point clouds, this nodelet uses two distance measures, which can be selected by the appropriate launch parameter:

- `point_2_point`: This calculates the euclidean distance between each point in the source point cloud and the nearest neighbor in the target point cloud (transformed into the frame of the source point cloud). 
In this, it finds the *n* nearest neighbors (ordered by their distance) and takes the distance to the first one in the list.
- `point_2_surface`: In order to calculate the distance of a point to a surface, information on the surface normal needs be available. 
Thus, in this case, the nodelet first computes the normal vectors for the transformed target point cloud. 
It then, again, finds the *n* nearest neighbors in the target point cloud to each point in the source cloud. 
The distance is then calculated by first computing the plane parameterization for each neighbor and its corresponding normal vector.
This followed by calculating the orthogonal distances of the source point w.r.t. each parameterized surface.  
The smallest of these orthogonal distances is then selected as the point-2-surface distance.

Prior to calculating the distance between two point clouds, the target point cloud is transformed into the coordinate system of the source point cloud.
In this way, since the source cloud is to be enhanced with distance information, the source cloud does not need to be transformed back prior to publishing.
The transformation is found, by first extracting the frame IDs from header of each point cloud, which can then be used to look it up in the TF tree.

**Launch-Parameters:**

- ```number_of_clouds```: Number of clouds to process. The input topics are appended with a index as suffix: `cloud_0`, `cloud_1`, ..., `cloud_N-1`.<br>
    *Type: int*<br>
    *Default: 2*
- ```processing_rate```: Rate in (Hz) at which to process the received point cloud.<br>
    *Type: int*<br>
    *Default: 1*
- ```distance_measure```: Enum: {0 = **point_2_point**, 1 = **point_2_surface**}<br>
    *Type: Enum: {0 = point_2_point, 1 = point_2_surface}*<br>
    *Default: 0*<br>
    *NOTE: The point_2_surface measure is more robust with a higher number of nearest neighbors. Especially when using point clouds from a rotating LiDAR, as a parameterization of a small number of neighbors might lead to a selection of all neighbors on one ring, making the surface estimation less accurate.*
- ```num_nearest_neighbors```: Number of nearest neighbors to consider in the target cloud for each point in the source cloud.<br>
    *Type: int*<br>
    *Default: 5*
- ```max_distance```: Maximum distance at which to truncate. This is also used for normalization when calculating the intensity.<br>
    *Type: float*<br>
    *Default: 5.0*
- ```clamp_distance_threshold```: Distance at which to clamp the source point cloud. Points exceeding the distance are removed. It is truncated to a minimum of 'max_distance'.<br>
    *Type: float*<br>
    *Default: FLT_MAX*
- `temp_transform`: *(Only available if 'number_of_clouds = 2')* Temporary transform of camera with respect to LiDAR given in the form of XYZ and RPY. The values are to be given as string list separated by whitespace. If left empty or omitted the transformation will be extracted from the TF-tree by using the frame IDs of the images and the clouds.<br>
  *Type: String*
  *Default: ""*

**Subscribed Topics:**

- ```cloud_<idx>```: Point cloud for which the distance is to be calculated.<br>
    *Type: sensor_msgs::PointCloud2*

**Published Topics:**

- ```~/cloud_<idx>_enhanced```: Source point cloud enhanced with the point-wise normalized distance (with the range of [0,1]) to the target point cloud in the intensity field.<br>
    *Type: sensor_msgs::PointCloud2*
