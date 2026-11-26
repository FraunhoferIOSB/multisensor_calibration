# Data Processing

## Calibration Target Detection in the Camera Data

The calibration target is detected in the image data by means of the ArUco markers.
In this, the [functionalities provided by OpenCV](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) are utilized to detect markers and compute the pose of the calibration target with respect to the camera.
To deduce the pose of the target from the marker detection, the algorithm requires the intrinsic parameters of the camera, which is why a good intrinsic calibration is vital.
The image points of the detected markers are stored to be later used as 2D correspondences as part of the PnP algorithm.
Furthermore, with the image positions of the detected markers and the a priori knowledge about the calibration board geometry, a 3D point cloud of the calibration target is also created. 
The detections, as well as the 6-DOF Pose of the target and the reconstructed 3D point cloud are published after the detection was successful. 
In this, the marker ID to which the detected corner belongs is stored in the intensity value of the point.

## Calibration Target Detection in the LiDAR Data

The detection and pose estimation of the calibration target within the LiDAR cloud data is realized in a number of consecutive steps.
In this numerous algorithms from the [Point Cloud Library (PCL)](https://pointclouds.org/) are used.

1. First, the input point cloud is segmented into planer regions using a region growing algorithm.
This algorithm requires the point cloud to have normal vectors which are computed prior to the region growing.

2. After that each segmented cluster whether its size and aspect ratio fits to the geometry of the calibration target.
If so, the cluster is considered as region of interest (which are also published as preview) in which a detailed detection and pose estimation of the target is performed.
In this, the algorithm fits a rotating bounding box to the cluster and checks its dimension.

3. When the actual target detection and pose estimation is to be performed, the orientation of the bounding box is first aligned to the actual orientation of the calibration target using the arrangement of the cutouts.
In this, all possible orientations are tested by counting the points in the point cloud at the assumed location of the cutouts.
The orientation for which the smallest number of points have been accumulated is taken as the orientation of the target.

4. This pose is used as an initialization to a RANSAC algorithm which tries to fit a custom sample consensus (SAC) model of the calibration target to the segmented region of interest.
In this it computes the normal vector of the board based on a random set of points from the cluster and varies position and rotation of the target pose with each RANSAC iteration.
Finally the pose with the most inliers is chosen as the winner-takes-it-all solution. 
As inliers all point that lie on the calibration target are chosen. 
With each point that falls into the circular cutout, the current estimation is penalized by reducing the number of inliers by a certain factor.

5. When RANSAC has found a pose, the coefficients are optimized by fitting a CAD model into the point cloud using GICP. 
In this only the points forming the convex hull of the cloud segmented from the input cloud are used.

6. Finally, the 3D coordinates of the marker corners are deduced based on the estimated board pose and stored as 3D correspondences.

The detections, as well as the 6-DOF Pose of the target and the segmented 3D point cloud are published after the detection was successful. 
In this, the marker ID to which the detected corner belongs is stored in the intensity value of the point.

The parameters for the algorithm outlined above are exposed as dynamic parameters and can be adjusted by calling `rqt_reconfigure` or opening the preferences from the UI (*Edit->Preferences*).
An overview of these parameters is given blow:

**Input Filter:**

- `max_range`: Maximum range at which to filter the incoming point cloud prior to any processing. Any point with a range (absolute distance from sensor) larger than max_range will be discarded. Turn to 0 to switch off.

**Normal Estimation:**

- `normal_estimation_search_method`: Select method to use for neighbor search. (0 = RadiusSearch, 1 = NearestNeighborSearch)
- `normal_estimation_search_radius`: Radius in which to search for neighbors. In case of 'RadiusSearch', this is a spatial extend. In case of 'NearestNeighborSearch', this represents the number of nearest neighbors (truncated to int).

**Region Growing:**

- `region_growing_cluster_size_min`: Minimum number of points a cluster needs to contain in order to be considered as valid inside the region growing.
- `region_growing_cluster_size_max`: Maximum number of points a cluster needs to contain in order to be considered as valid inside the region growing.
- `region_growing_number_neighbors`: Number of neighbor points to consider during region growing.
- `region_growing_angle_thresh`: Angle in degrees used as the allowable range for the normals deviation. If the deviation between points normals is less than the smoothness threshold then they are suggested to be in the same cluster.
- `region_growing_curvature_thresh`: Second criteria for the region growing.  If two points have a small normals deviation then the disparity between their curvatures is tested.

**Size Filter:**

- `size_filter_width_min_tolerance`: Tolerance (in m) of the minimum board width when filtering the clusters based on their size.
- `size_filter_width_max_tolerance`: Tolerance (in m) of the maximum board width when filtering the clusters based on their size.
- `size_filter_height_min_tolerance`: Tolerance (in m) of the minimum board height when filtering the clusters based on their size.
- `size_filter_height_max_tolerance`: Tolerance (in m) of the maximum board height when filtering the clusters based on their size.

**RANSAC:**

- `ransac_distance_thresh`: Distance threshold (in m) from model for points to count as inliers during RANSAC.
- `ransac_rotation_variance`: Maximum angle in rotation (in degrees) to be sampled when computing the new coefficients within RANSAC.
- `ransac_translation_variance`: Maximum distance in translation (in m) to be sampled when computing the new coefficients within RANSAC.
- `ransac_max_iterations`: Maximum number of iterations of the RANSAC detecting the calibration target.
- `ransac_optimize_coefficients`: Option to activate the optimization of the coefficients by means of ICP.
- `target_icp_variant`: Select ICP variant to use to optimize coefficients. (0 = ICP, 1 = PlaneICP, 2 = GICP)
- `target_icp_max_correspondence_distance`: Maximum distance for ICP to search for point correspondences. Given as ratio with respect to shorter side of calibration target.
- `target_icp_rotation_tolerance`: Rotation tolerance for convergence check. Given in degrees.
- `target_icp_translation_tolerance`: Translation tolerance for convergence check. Given in unit of the LiDAR point cloud, typically meters.

## Perspective-n-Point for 2D-3D Pose Estimation

In the extrinsic pose estimation using sets of 2D-3D correspondences (e.g. Camera-LiDAR calibration) the [Perspective-n-Point (PnP) algorithm from OpenCV](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html) is used.
The parameters to configure the PnP algorithm are exposed as dynamic parameters and can be adjusted by calling `rqt_reconfigure` or opening the preferences from the UI (*Edit->Preferences*).
An overview of these parameters is given blow:

- `limit_single_board_rpj_error`: Use max maximum reprojection error to accept during calibration of a single target pose. If false, 'board_max_rpj_error' is ignored.
- `single_board_max_rpj_error`: Limit for maximum reprojection error to accept during calibration of a single target pose. All calibrated poses, that exceed this limit are rejected.
- `single_board_min_inliers`: Threshold for minimum number of inliers to accept during calibration of a single target pose. All calibrated poses, that do not reach this threshold are rejected.
- `pnp_inlier_rpj_error_limit`: Limit for maximum reprojection error for which points are considered as RANSAC inliers during PnP.

## GICP for 3D-3D Alignment

In the alignment of 3D point clouds of two LiDAR sensors (e.g. LiDAR-LiDAR calibration) the [Generalized-ICP (GICP) of 'small_gicp'](https://github.com/koide3/small_gicp) is used.
The parameters to configure the GICP algorithm are exposed as dynamic parameters and can be adjusted by calling `rqt_reconfigure` or opening the preferences from the UI (*Edit->Preferences*).
An overview of these parameters is given blow:

- `registration_icp_variant`: Select ICP variant to use for registration. (0 = ICP, 1 = PlaneICP, 2 = GICP)
- `registration_icp_max_correspondence_distance`: Maximum distance for ICP to search for point correspondences. Given as ratio with respect to shorter side of calibration target.
- `registration_icp_rotation_tolerance`: Rotation tolerance for convergence check. Given in degrees.
- `registration_icp_translation_tolerance`: Translation tolerance for convergence check. Given in unit of the LiDAR point cloud, typically meters.

## Error Computation and Estimation of Calibration Certainty

In the calibration using the PnP algorithm to align two sensors by means of 2D-3D correspondences, the quality of the calibration is measured by the [mean reprojection error](https://docs.opencv.org/3.4/dc/dbb/tutorial_py_calibration).html).
For a good calibration a typical reprojection error lies between 2-4 Pixels.

In the calibration of two 3D sensors in which the sensor data gets aligned by means of GIPC, the quality of the calibration is measured using the [root mean squared error (RMSE)](https://en.wikipedia.org/wiki/Root_mean_square_deviation).
For a good calibration a typical RMSE lies in the range of view centimeters.

To quantify the certainty of the result, the standard deviation of the individual target poses is also calculated.
In this, for a given extrinsic calibration the poses of the detected calibration target are projected from the coordinate frame of the source sensor into the coordinate frame of the reference sensor.
The standard deviation in the translational and rotational difference should give us a good indication on the consistency of the calibration.