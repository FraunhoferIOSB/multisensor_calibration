# Installation

The Multi-Sensor Calibration Toolbox is released as an [official package for ROS 2](#official-ros-2-package), named `multisensor_calibration`.
Since ROS 1 is soon end-of-life, there will be no official release for ROS 1.

In addition to the official release, `multisensor_calibration` can also be [build from source](#build-from-source) for ROS 1 and ROS 2 as described below.

## Official ROS 2 Package

```text
sudo apt install ros-$ROS_DISTRO-multisensor-calibration
```

The source code for each ROS 2 distribution is maintained on the following branches:

| ROS 2 distribution | Branch |
| ------------------ | ------ |
| Lyrical, Kilted    | `main` |
| Jazzy              | `jazzy` |

## Build from Source

### ROS 1

<ol>
<li>
Initialize catkin workspace, if required:

```bash
mkdir -p calibration_ws/src
catkin init --workspace calibration_ws
```
</li>

<li>
Clone repository and checkout `noetic` branch:

```bash
cd calibration_ws/src
git clone https://github.com/FraunhoferIOSB/multisensor_calibration.git
cd multisensor_calibration
git checkout noetic
cd ../../
```
</li>

<li>
Initialize `rosdep` and install dependencies:

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```
</li>

<li>
(OPTIONAL) Copy custom example robot workspaces and populate resource file (see Workspace) page)

```bash
src/multisensor_calibration/scripts/populate_robot_workspaces_qrc.sh <custom_robot_ws_directory>
```
</li>

<li>
(OPTIONAL) Copy custom launch files

```bash
src/multisensor_calibration/scripts/copy_launch_files.sh <custom_launch_directory>
```
</li>

<li>
Run `catkin` to build from source:<br>
To build in 'Debug' mode add `-DCMAKE_BUILD_TYPE=Debug` to catkin command.
If 'CMAKE_BUILD_TYPE' omitted, multisensor_calibration will be build in 'Release' mode.

```bash
catkin build -j8 -DCMAKE_BUILD_TYPE=Release multisensor_calibration
```
</li>

</ol>

### ROS 2

<ol>
<li>
Create workspace folder, if required:

```bash
mkdir -p calibration_ws/src
```
</li>

<li>
Clone repository:

```bash
cd calibration_ws/src
git clone https://github.com/FraunhoferIOSB/multisensor_calibration.git
```

For ROS 2 Jazzy, check out the `jazzy` branch:

```bash
cd multisensor_calibration
git checkout jazzy
cd ../../
```
</li>

<li>
Initialize `rosdep` and install dependencies:

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```
</li>

<li>
(OPTIONAL) Copy custom example robot workspaces and populate resource file (see Workspace page)

```bash
src/multisensor_calibration/scripts/populate_robot_workspaces_qrc.sh <custom_robot_ws_directory>
```
</li>

<li>
(OPTIONAL) Copy custom launch files

```bash
src/multisensor_calibration/scripts/copy_launch_files.sh <custom_launch_directory>
```
</li>

<li>
Run 'colcon' to build from source:<br>
To build in 'Debug' mode add '--cmake-args -DCMAKE_BUILD_TYPE=Debug' to colcon command.
If 'CMAKE_BUILD_TYPE' omitted, multisensor_calibration will be build in 'Release' mode.

```bash
colcon build --symlink-install --packages-up-to multisensor_calibration
```
</li>

</ol>

### Requirements

Apart from the basic ROS requirements, `multisensor_calibration` depends on the following third party libraries.

- [**PCL**](https://pointclouds.org/)
- [**OpenCV**](https://opencv.org/)
- [**Qt**](https://www.qt.io/)
- [**small_gicp**](https://github.com/koide3/small_gicp): The package `small_gicp_vendor` wraps this library to make it available for other ROS components. This is a common practice for third-party libraries without available deb sources. It is licensed under the MIT-License.
- [**OpenMP**](https://www.openmp.org/) (optional): This is used to parallelize and speed up the processing of each point in the point cloud. If not found by CMake the processing will be done sequentially.
- [**Doxygen**](https://www.doxygen.nl/) (optional): If available, this Doxygen documentation will be build automatically.
