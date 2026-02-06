# multisensor_calibration

### Contents:

- [multisensor\_calibration](#multisensor_calibration)
    - [Contents:](#contents)
    - [Continuous Integration:](#continuous-integration)
  - [Getting Started](#getting-started)
    - [Requirements](#requirements)
    - [Build](#build)
  - [Usage](#usage)
    - [Documentation](#documentation)


------------------------

An actively maintained universal calibration toolbox for assisted, target-based multi-sensor calibration with ROS 1 and ROS 2 support. 
It provides a variety of methods and applications to calibrate complex multi-sensor systems, e.g.

- <b>Extrinsic Camera-LiDAR Calibration</b>,
- <b>Extrinsic Camera-Reference Calibration</b>,
- <b>Extrinsic LiDAR-LiDAR Calibration</b>,
- <b>Extrinsic Camera-Camera Calibration (Stereo)</b>,
- <b>Extrinsic LiDAR-Reference Calibration</b>, and
- <b>Extrinsic LiDAR-Vehicle Calibration</b> (prototype).

The software is licensed under the new [BSD 3-Clause license](license.md). If you use this project for your research, please cite:

```text
@inproceedings{
    ruf2025_multisensor_calibration,
    title={Multi-Sensor Calibration Toolbox for Large-Scale Offroad Robotics},
    author={Boitumelo Ruf and Miguel Granero and Raphael Hagmanns and Janko Petereit},
    conference={German Robotics Conference (RIG) 2025},
    year={2025},
} 
```

The `multisensor_calibration` can be installed with apt:
```bash
sudo apt install ros-$ROS_DISTRO-multisensor-calibration
```

Or manually. If you want to compile and use locally, see [Getting Started](#getting-started).

Since ROS 1 is soon end-of-life, there will be no official release for ROS 1.
However, there is a version of the source code available for ROS 1 under the branch [noetic](https://github.com/FraunhoferIOSB/multisensor_calibration/tree/noetic).


**Acknowledgement**: This software was developed as part of the projects [AKIT-PRO](https://a-kit.de) (grant no. 13N15673) and [ROBDEKON – Robotic Systems for Decontamination in Hazardous Environments](https://robdekon.de/) (grant nos. 13N14674 and 13N16538), funded by the Federal Ministry of Education and Research (BMBF) under the German Federal Government’s Research for Civil Security program.

------------------------

### Continuous Integration:

| Service    | devel   | main   |
| ---------- | ------- | ------ |
| Docs     |         | [![deploy](https://github.com/FraunhoferIOSB/multisensor_calibration/actions/workflows/docs.yml/badge.svg)](https://github.com/FraunhoferIOSB/multisensor_calibration/actions/workflows/docs.yml) |
| Build           | [![Jazzy Jalisco](https://github.com/FraunhoferIOSB/multisensor_calibration/actions/workflows/build_and_test_jazzy.yml/badge.svg?branch=devel)](https://github.com/FraunhoferIOSB/multisensor_calibration/actions/workflows/build_and_test_jazzy.yml)           | [![Jazzy Jalisco](https://github.com/FraunhoferIOSB/multisensor_calibration/actions/workflows/build_and_test_jazzy.yml/badge.svg?branch=main)](https://github.com/FraunhoferIOSB/multisensor_calibration/actions/workflows/build_and_test_jazzy.yml) |

------------------------

## Getting Started

If you have already installed the package with apt, this section can be skipped.

### Requirements

Basic requirements can be installed by calling following command from the top of the ROS workspace:

    rosdep install -y -r --from-paths src --ignore-src

Further requirements:

- [**PCL**](https://pointclouds.org/)
- [**OpenCV**](https://opencv.org/)
- [**Qt**](https://www.qt.io/)
- [**small_gicp**](https://github.com/koide3/small_gicp): TThe package `small_gicp_vendor` wraps this library to make it available for other ROS components. This is a common practice for third-party libraries without available deb sources. It is licensed under the MIT-License.
- [**OpenMP**](https://www.openmp.org/) (optional): This is used to parallelize and speed up the processing of each point in the point cloud. If not found by CMake the processing will be done sequentially.
- [**Doxygen**](https://www.doxygen.nl/) (optional): If available, this Doxygen documentation will be build automatically.

### Build

1. Clone repository:

    ```bash
    git clone https://github.com/FraunhoferIOSB/multisensor_calibration.git
    ```

2. Initialize `rosdep` and install dependencies:

    ```bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src -y --ignore-src
    ```

3. Run `colcon` to build from source:<br>
To build in 'Debug' mode add `-DCMAKE_BUILD_TYPE=Debug` to catkin command.
If 'CMAKE_BUILD_TYPE' omitted, multisensor_calibration will be build in 'Release' mode.

    ```bash
    MAKEFLAGS='-j 16' colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

## Usage

Run the following launch file from the package.

```bash
ros2 launch multisensor_calibration multi_sensor_calib_example.launch.py
```
### Documentation

See the [user documentation](https://fraunhoferiosb.github.io/multisensor_calibration/) for basic usage, tutorials and workflows.
