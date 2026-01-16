# Copyright (c) 2024 - 2025 Fraunhofer IOSB and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Fraunhofer IOSB nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


cmake_minimum_required(VERSION 3.9.0)

# get version from package.xml
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" package_xml_str)

# Extract project version.
set(VERSION_STR "0.0.0")
if(NOT package_xml_str MATCHES "<version>([0-9]+.[0-9]+.[0-9]+)</version>")
    message(FATAL_ERROR "Could not parse project version from package manifest (aborting)")
else()
    set(VERSION_STR ${CMAKE_MATCH_1})
endif()
string(REPLACE "." ";" VERSION_LIST "${VERSION_STR}")
list(GET VERSION_LIST 0 VERSION_MAJOR)
list(GET VERSION_LIST 1 VERSION_MINOR)
list(GET VERSION_LIST 2 VERSION_PATCH)

# set project name (ros convention --> lower case)
project(multisensor_calibration VERSION ${VERSION_STR})

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to Release as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE
    STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
  "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wdeprecated -fPIC)
    add_compile_options(-Wno-array-bounds)
endif()

# suppress developer warnings
set(no_dev_warnings_backup "$CACHE{CMAKE_SUPPRESS_DEVELOPER_WARNINGS}")
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS ON CACHE INTERNAL "" FORCE)

# 3rd party libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(urdf REQUIRED)
find_package(rviz_common REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(multisensor_calibration_interface REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS
    core
    calib3d
)
find_package(small_gicp_vendor REQUIRED)
find_package(small_gicp REQUIRED)


find_package(OpenMP)

find_package(QT NAMES Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui Widgets)

# setup qt
set(CMAKE_AUTOUIC ON) # The AUTOGEN_BUILD_DIR is automatically added to the target's INCLUDE_DIRECTORIES.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# set include directories
include_directories(
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

# common compile definitions
set(COMMON_COMPILE_DEFINITIONS
    -DVERSION_MAJOR=${VERSION_MAJOR}
    -DVERSION_MINOR=${VERSION_MINOR}
    -DVERSION_PATCH=${VERSION_PATCH}
    -DPCL_NO_PRECOMPILE
    -DMULTI_SENSOR_CALIBRATION_TARGET=0
    -DEXTRINSIC_CAMERA_LIDAR_CALIBRATION_TARGET=1
    -DEXTRINSIC_LIDAR_LIDAR_CALIBRATION_TARGET=2
    -DEXTRINSIC_CAMERA_REFERENCE_CALIBRATION_TARGET=3
    -DEXTRINSIC_LIDAR_REFERENCE_CALIBRATION_TARGET=4
    -DEXTRINSIC_LIDAR_VEHICLE_CALIBRATION_TARGET=5
    -DSMALL_GICP_HEAD_COMMIT_HASH="${small_gicp_VERSION}"
)

# Debug / Release compile defitions
if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    set(COMMON_COMPILE_DEFINITIONS ${COMMON_COMPILE_DEFINITIONS}
        -DDEBUG_BUILD)
else()
    set(COMMON_COMPILE_DEFINITIONS ${COMMON_COMPILE_DEFINITIONS} -DMULTI_THREADED)
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tinyxml2
  urdf
  rviz_common
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  multisensor_calibration_interface
  small_gicp
  small_gicp_vendor
)



# library source files
file(GLOB_RECURSE LIBRARY_SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/src/calibration/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/calibration_target/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/io/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/sensor_data_processing/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/visualizers/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/ui/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/guidance/*.cpp"
)

# library header files that are to be parsed by QT moc
# i.e. those containing QT macros such as Q_OBJECT
# they need to be explicitly be parsed to target, otherwise moc wont parse them
set(LIBRARY_MOC_HEADER_FILES
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/GuiBase.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/CalibrationGuiBase.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/CalibrationControlWindow.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/ImageViewDialog.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/Rviz3dViewDialog.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/CameraLidarCalibrationGui.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/LidarLidarCalibrationGui.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/CameraReferenceCalibrationGui.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/ObservationsViewDialog.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/LidarReferenceCalibrationGui.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/ExtrinsicCameraLidarConfigWidget.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/ExtrinsicCameraReferenceConfigWidget.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/ExtrinsicCameraCameraConfigWidget.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/ExtrinsicLidarLidarConfigWidget.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/ExtrinsicLidarReferenceConfigWidget.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/CalibrationConfigDialog.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/InstallWorkspaceDialog.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/MultiSensorCalibrationGui.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/multisensor_calibration/ui/AboutDialog.h"
)

# library ui files
set(GLOB_RECURSE LIBRARY_UI_FILES
    "${CMAKE_CURRENT_LIST_DIR}/src/ui/*.ui"
)

# library qrc files, also to be parsed to target in order for them to be processed
set(QRC_SRC_FILES  # With ROS1 qt5_add_resources was used instead of set
    "${CMAKE_CURRENT_LIST_DIR}/resources/settings_templates.qrc"
    "${CMAKE_CURRENT_LIST_DIR}/resources/icons.qrc"
    "${CMAKE_CURRENT_LIST_DIR}/resources/robot_workspaces.qrc"
    "${CMAKE_CURRENT_LIST_DIR}/resources/about.qrc"
)

############################################################################
## TARGETS
############################################################################

######################################
# library target
######################################
set(LIBRARY_TARGET_NAME ${PROJECT_NAME})
add_library(${LIBRARY_TARGET_NAME} SHARED
    ${LIBRARY_MOC_HEADER_FILES}
    ${LIBRARY_SOURCE_FILES}
    ${LIBRARY_UI_FILES}
    ${QRC_SRC_FILES}
)
target_link_libraries(${LIBRARY_TARGET_NAME}
    ${OpenCV_LIBRARIES}
    pcl_visualization
    small_gicp::small_gicp
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Gui
)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${LIBRARY_TARGET_NAME} OpenMP::OpenMP_CXX)
endif()
target_compile_definitions(${LIBRARY_TARGET_NAME} PRIVATE
    -DTARGET_NAME="${LIBRARY_TARGET_NAME}"
    ${COMMON_COMPILE_DEFINITIONS}
)
ament_target_dependencies(${LIBRARY_TARGET_NAME}
    ${DEPENDENCIES}
)
ament_export_targets(export_${LIBRARY_TARGET_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

rclcpp_components_register_nodes(${LIBRARY_TARGET_NAME} "multisensor_calibration::visualizers::PointCloud2PointCloudDistanceNode")
rclcpp_components_register_nodes(${LIBRARY_TARGET_NAME} "multisensor_calibration::visualizers::PointCloud2ImageNode")

######################################
# extrinsic_camera_lidar_calibration
######################################
set(CAMERA_LIDAR_CALIB_NODE_TARGET_NAME extrinsic_camera_lidar_calibration)
add_executable(${CAMERA_LIDAR_CALIB_NODE_TARGET_NAME}
    src/nodes/CalibrationNode.cpp
    ${QRC_SRC_FILES}
)
target_compile_definitions(${CAMERA_LIDAR_CALIB_NODE_TARGET_NAME} PRIVATE
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${CAMERA_LIDAR_CALIB_NODE_TARGET_NAME}"
    -DTARGET=EXTRINSIC_CAMERA_LIDAR_CALIBRATION_TARGET                     # see COMMON_COMPILE_DEFINITIONS for enum
)
target_link_libraries(${CAMERA_LIDAR_CALIB_NODE_TARGET_NAME}
    ${LIBRARY_TARGET_NAME}
    tinyxml2::tinyxml2
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Gui
)

######################################
# extrinsic_camera_reference_calibration
######################################
set(CAMERA_REFERENCE_CALIB_NODE_TARGET_NAME extrinsic_camera_reference_calibration)
add_executable(${CAMERA_REFERENCE_CALIB_NODE_TARGET_NAME}
    src/nodes/CalibrationNode.cpp
    ${QRC_SRC_FILES}
)
target_compile_definitions(${CAMERA_REFERENCE_CALIB_NODE_TARGET_NAME} PRIVATE
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${CAMERA_REFERENCE_CALIB_NODE_TARGET_NAME}"
    -DTARGET=EXTRINSIC_CAMERA_REFERENCE_CALIBRATION_TARGET                   # see COMMON_COMPILE_DEFINITIONS for enum
)
target_link_libraries(${CAMERA_REFERENCE_CALIB_NODE_TARGET_NAME}
    ${LIBRARY_TARGET_NAME}
    tinyxml2::tinyxml2
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Gui
)

######################################
# extrinsic_lidar_lidar_calibration
######################################
set(LIDAR_LIDAR_CALIB_NODE_TARGET_NAME extrinsic_lidar_lidar_calibration)
add_executable(${LIDAR_LIDAR_CALIB_NODE_TARGET_NAME}
    src/nodes/CalibrationNode.cpp
    ${QRC_SRC_FILES}
)
target_compile_definitions(${LIDAR_LIDAR_CALIB_NODE_TARGET_NAME} PRIVATE
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${LIDAR_LIDAR_CALIB_NODE_TARGET_NAME}"
    -DTARGET=EXTRINSIC_LIDAR_LIDAR_CALIBRATION_TARGET                     # see COMMON_COMPILE_DEFINITIONS for enum
)
target_link_libraries(${LIDAR_LIDAR_CALIB_NODE_TARGET_NAME}
    ${LIBRARY_TARGET_NAME}
    tinyxml2::tinyxml2
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Gui
)

######################################
# extrinsic_lidar_reference_calibration
######################################
set(LIDAR_REFERENCE_CALIB_NODE_TARGET_NAME extrinsic_lidar_reference_calibration)
add_executable(${LIDAR_REFERENCE_CALIB_NODE_TARGET_NAME}
    src/nodes/CalibrationNode.cpp
    ${QRC_SRC_FILES}
)
target_compile_definitions(${LIDAR_REFERENCE_CALIB_NODE_TARGET_NAME} PRIVATE
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${LIDAR_REFERENCE_CALIB_NODE_TARGET_NAME}"
    -DTARGET=EXTRINSIC_LIDAR_REFERENCE_CALIBRATION_TARGET                 # see COMMON_COMPILE_DEFINITIONS for enum
)
target_link_libraries(${LIDAR_REFERENCE_CALIB_NODE_TARGET_NAME}
    ${LIBRARY_TARGET_NAME}
    tinyxml2::tinyxml2
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Gui
)

######################################
# extrinsic_camera_camera_calibration
######################################
set(CAMERA_CAMERA_CALIB_NODE_TARGET_NAME extrinsic_camera_camera_calibration)
add_executable(${CAMERA_CAMERA_CALIB_NODE_TARGET_NAME}
    src/nodes/CalibrationNode.cpp
    ${QRC_SRC_FILES}
)
target_compile_definitions(${CAMERA_CAMERA_CALIB_NODE_TARGET_NAME} PRIVATE
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${CAMERA_CAMERA_CALIB_NODE_TARGET_NAME}"
    -DTARGET=STEREO_CAMERA_CALIBRATION                     # see COMMON_COMPILE_DEFINITIONS for enum
)
target_link_libraries(${CAMERA_CAMERA_CALIB_NODE_TARGET_NAME}
    ${LIBRARY_TARGET_NAME}
    tinyxml2::tinyxml2
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Gui
)

######################################
# multi_sensor_calibration
######################################
set(MULTI_SENSOR_CALIB_NODE_TARGET_NAME multi_sensor_calibration)
add_executable(${MULTI_SENSOR_CALIB_NODE_TARGET_NAME}
    src/nodes/CalibrationNode.cpp
    ${QRC_SRC_FILES}
)
target_compile_definitions(${MULTI_SENSOR_CALIB_NODE_TARGET_NAME} PRIVATE
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${MULTI_SENSOR_CALIB_NODE_TARGET_NAME}"
    -DTARGET=MULTI_SENSOR_CALIBRATION_TARGET                     # see COMMON_COMPILE_DEFINITIONS for enum
)
target_link_libraries(${MULTI_SENSOR_CALIB_NODE_TARGET_NAME}
    ${LIBRARY_TARGET_NAME}
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Gui
)

######################################
# initialize_robot_workspace
######################################
set(INIT_ROBOT_WS_NODE_TARGET_NAME initialize_robot_workspace)
add_executable(${INIT_ROBOT_WS_NODE_TARGET_NAME}
    src/nodes/InitializeRobotWsNode.cpp
    ${QRC_SRC_FILES}
)
target_link_libraries(${INIT_ROBOT_WS_NODE_TARGET_NAME}
    ${LIBRARY_TARGET_NAME}
    ${catkin_LIBRARIES}
    Qt${QT_VERSION_MAJOR}::Core
)
target_compile_definitions(${INIT_ROBOT_WS_NODE_TARGET_NAME} PRIVATE
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${INIT_ROBOT_WS_NODE_TARGET_NAME}"
)

######################################
# publish_pointcloud
######################################
set(PUB_PC_NODE_TARGET_NAME publish_point_cloud)
add_executable(${PUB_PC_NODE_TARGET_NAME} src/nodes/PublishPointCloudNode.cpp)
target_compile_definitions(${PUB_PC_NODE_TARGET_NAME} PRIVATE
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${PUB_PC_NODE_TARGET_NAME}"
)
target_link_libraries(${PUB_PC_NODE_TARGET_NAME}
    ${LIBRARY_TARGET_NAME}
)

######################################
# aggregate_point_cloud
######################################
set(AGGREGATE_PC_NODE_TARGET_NAME aggregate_point_cloud)
add_executable(${AGGREGATE_PC_NODE_TARGET_NAME} src/nodes/AggregatePointCloudNode.cpp)
target_compile_definitions(${AGGREGATE_PC_NODE_TARGET_NAME} PRIVATE
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${AGGREGATE_PC_NODE_TARGET_NAME}"                   # see COMMON_COMPILE_DEFINITIONS for enum
)
target_link_libraries(${AGGREGATE_PC_NODE_TARGET_NAME}
    ${LIBRARY_TARGET_NAME}
)

############################################################################
## INSTALLS
############################################################################

install(
  TARGETS ${LIBRARY_TARGET_NAME}
  EXPORT export_${LIBRARY_TARGET_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
    ${CAMERA_LIDAR_CALIB_NODE_TARGET_NAME}
    ${CAMERA_REFERENCE_CALIB_NODE_TARGET_NAME}
    ${LIDAR_LIDAR_CALIB_NODE_TARGET_NAME}
    ${LIDAR_REFERENCE_CALIB_NODE_TARGET_NAME}
    ${MULTI_SENSOR_CALIB_NODE_TARGET_NAME}
    ${LIDAR_VEHICLE_CALIB_NODE_TARGET_NAME}
    ${INIT_ROBOT_WS_NODE_TARGET_NAME}
    ${PUB_PC_NODE_TARGET_NAME}
    ${AGGREGATE_PC_NODE_TARGET_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

############################################################################
## TESTS
############################################################################

if(BUILD_TESTING)
    #   find_package(ament_cmake_copyright REQUIRED)
    #   ament_copyright()
    #   find_package(ament_cmake_cppcheck REQUIRED)
    #   ament_cppcheck()
    #   find_package(ament_cmake_cpplint REQUIRED)
    #   ament_cpplint(FILTERS -whitespace/newline,-readability/braces,-readability/todo)
    #   find_package(ament_cmake_flake8 REQUIRED)
    #   ament_flake8()
    #   find_package(ament_cmake_lint_cmake REQUIRED)
    #   ament_lint_cmake()
endif()

ament_package()
