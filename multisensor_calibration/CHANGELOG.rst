^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package multisensor_calibration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* feat: support ROS 2 Kilted and Lyrical (Qt6, OpenCV and rclcpp API updates)
* feat: register calibration and guidance nodes as components and add headless camera-LiDAR launch
* feat: add 'latest' sync policy (no time synchronization)
* feat: reference calibration: import reference points from CSV, add/remove table rows
* feat: add ransac_max_iterations and retune LiDAR target detection defaults
* refactor: rename executable aggregate_point_cloud to accumulate_point_cloud
* fix: crashes on observation removal and when closing the point cloud distance visualizer
* fix: infinite wait for calibration metadata and services blocked during data processing
* fix: visualizers use best-effort QoS for LiDAR data and the calibration node's sync policy
* fix: RANSAC random sampling was reseeded on every call
* fix: install camera-camera calibration executable
* fix: Wayland compatibility workaround
* Contributors: Miguel Granero

2.1.0 (2026-02-23)
------------------
* feat: add camera-camera extrinsic calibration
* refactor: clean calibration inheritance structure
* Contributors: Miguel Granero

2.0.4 (2025-09-29)
------------------
* refactor: Minor style changes and deprecation removals
* Prepare for Kilted + Rolling Release
* Contributors: Miguel Granero

2.0.3 (2025-03-28)
------------------
* refactor: remove unused rviz config files
* fix: set LICENSE path to own package for bin packaging
* feat: add LICENSE file to single packages
* Contributors: Miguel Granero

2.0.2 (2025-03-28)
------------------
* feat: create small_gicp_vendor package
* fix: export library target
* fix: read version from package.xml and not from git
* Contributors: Miguel Granero

2.0.1 (2025-03-24)
------------------
* refactor: remove compilation warnings
* fix: remove deprecated pointer type
* Contributors: Miguel Granero

2.0.0 (2025-02-20)
------------------
* feat: Port of multisensor_calibration to ROS2 Jazzy.
* Contributors: Miguel Granero

1.0.0 (2025-02-18)
------------------
* Extrinsic Camera-LiDAR Calibration
* Extrinsic Camera-Reference Calibration
* Extrinsic LiDAR-LiDAR Calibration
* Extrinsic LiDAR-Reference Calibration
* Extrinsic LiDAR-Vehicle Calibration (prototype)
* Calibration Configurator
* Documentation

Contributors: Boitumelo Ruf, Miguel Granero, Christian Frese, Raphael Hagmanns, Tom Schmidt
