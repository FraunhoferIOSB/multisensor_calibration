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

#include "../include/multisensor_calibration/ui/LidarLidarCalibrationGui.h"

// Std
#include <future>
#include <string>
#include <thread>

// Qt
#include <QCoreApplication>
#include <QLabel>
#include <QMessageBox>
#include <QObject>

// ROS
#include <tf/tf.h>

// multisensor_calibration
#include "../../include/multisensor_calibration/common/utils.hpp"
#include "../include/multisensor_calibration/common/common.h"
#include <multisensor_calibration/SensorExtrinsics.h>

namespace multisensor_calibration
{

//==================================================================================================
LidarLidarCalibrationGui::LidarLidarCalibrationGui(const std::string& iAppTitle,
                                                   const std::string& iGuiSubNamespace) :
  CalibrationGuiBase(iAppTitle, iGuiSubNamespace),
  pPlacementGuidanceDialog_(nullptr),
  pSrcLidarTargetDialog_(nullptr),
  pRefLidarTargetDialog_(nullptr)
{
}

//==================================================================================================
LidarLidarCalibrationGui::~LidarLidarCalibrationGui()
{
    //--- unload visualization nodelet if loaded
    pNodeletLoader_->unload(visualizerNodeletName_);
}

//==================================================================================================
void LidarLidarCalibrationGui::initializeGuiContents()
{
    CalibrationGuiBase::initializeGuiContents();

    //--- initialize content of placement guidance dialog
    if (pPlacementGuidanceDialog_)
    {
        pPlacementGuidanceDialog_->setFixedReferenceFrame((calibrationMetaData_.base_frame_id.empty())
                                                            ? calibrationMetaData_.ref_frame_id
                                                            : calibrationMetaData_.base_frame_id);
        pPlacementGuidanceDialog_->addAxes();
        pPlacementGuidanceDialog_->addRawSensorCloud(calibrationMetaData_.ref_topic_name);
        pPlacementGuidanceDialog_->addGuidedPlacementBox(guidanceNodeletName_ +
                                                         "/" + PLACEMENT_GUIDANCE_TOPIC_NAME);

        if (calibrationMetaData_.base_frame_id.empty() == false)
            pPlacementGuidanceDialog_->setView(Rviz3dViewDialog::TOP_DOWN);
    }

    //--- initialize content of source lidar target visualization
    if (pSrcLidarTargetDialog_)
    {
        pSrcLidarTargetDialog_->setWindowTitle(
          QString::fromStdString(calibrationMetaData_.src_sensor_name));

        pSrcLidarTargetDialog_->setFixedReferenceFrame((calibrationMetaData_.base_frame_id.empty())
                                                         ? calibrationMetaData_.src_frame_id
                                                         : calibrationMetaData_.base_frame_id);
        pSrcLidarTargetDialog_->addAxes();
        pSrcLidarTargetDialog_->addRawSensorCloud(calibrationMetaData_.src_topic_name);
        pSrcLidarTargetDialog_
          ->addRegionsOfInterestCloud(calibratorNodeletName_ +
                                      "/" + calibrationMetaData_.src_sensor_name +
                                      "/" + ROIS_CLOUD_TOPIC_NAME);
        pSrcLidarTargetDialog_
          ->addCalibTargetCloud(calibratorNodeletName_ +
                                "/" + calibrationMetaData_.src_sensor_name +
                                "/" + TARGET_PATTERN_CLOUD_TOPIC_NAME);
        pSrcLidarTargetDialog_
          ->addMarkerCornersCloud(calibratorNodeletName_ +
                                  "/" + calibrationMetaData_.src_sensor_name +
                                  "/" + MARKER_CORNERS_TOPIC_NAME);
    }

    //--- initialize content of reference lidar target visualization
    if (pRefLidarTargetDialog_)
    {
        pRefLidarTargetDialog_->setWindowTitle(
          QString::fromStdString(calibrationMetaData_.ref_sensor_name));

        pRefLidarTargetDialog_->setFixedReferenceFrame((calibrationMetaData_.base_frame_id.empty())
                                                         ? calibrationMetaData_.ref_frame_id
                                                         : calibrationMetaData_.base_frame_id);
        pRefLidarTargetDialog_->addAxes();
        pRefLidarTargetDialog_->addRawSensorCloud(calibrationMetaData_.ref_topic_name);
        pRefLidarTargetDialog_
          ->addRegionsOfInterestCloud(calibratorNodeletName_ +
                                      "/" + calibrationMetaData_.ref_sensor_name +
                                      "/" + ROIS_CLOUD_TOPIC_NAME);
        pRefLidarTargetDialog_
          ->addCalibTargetCloud(calibratorNodeletName_ +
                                "/" + calibrationMetaData_.ref_sensor_name +
                                "/" + TARGET_PATTERN_CLOUD_TOPIC_NAME);
        pRefLidarTargetDialog_
          ->addMarkerCornersCloud(calibratorNodeletName_ +
                                  "/" + calibrationMetaData_.ref_sensor_name +
                                  "/" + MARKER_CORNERS_TOPIC_NAME);
    }

    //--- hide progress dialog
    hideProgressDialog();
}
//==================================================================================================
void LidarLidarCalibrationGui::loadVisualizer()
{
    const double MAX_POINT_DISTANCE = 0.05;

    // Function to initialize visualizer nodelet and dialog
    auto initializeAndRunSensorFusion = [&]() -> bool
    {
        //--- get sensor extrinsics
        ros::ServiceClient extrinsicsClient =
          nh_.serviceClient<SensorExtrinsics>(calibratorNodeletName_ +
                                              "/" + REQUEST_SENSOR_EXTRINSICS_SRV_NAME);
        SensorExtrinsics extrinsicsSrvMsg;
        extrinsicsSrvMsg.request.extrinsic_type = SensorExtrinsics::Request::SENSOR_2_SENSOR;
        geometry_msgs::Pose& extrinsicPose      = extrinsicsSrvMsg.response.extrinsics;
        if (!extrinsicsClient.call(extrinsicsSrvMsg))
        {
            ROS_ERROR("[%s] Failed to get sensor extrinsics. "
                      "Check if calibration nodelet is initialized!",
                      guiNodeName_.c_str());

            return false;
        }

        //--- check if extrinsic pose is 0
        if (tf::Vector3(extrinsicPose.position.x,
                        extrinsicPose.position.y,
                        extrinsicPose.position.z) == tf::Vector3(0, 0, 0) &&
            tf::Quaternion(extrinsicPose.orientation.x,
                           extrinsicPose.orientation.y,
                           extrinsicPose.orientation.z,
                           extrinsicPose.orientation.w) == tf::Quaternion(0, 0, 0, 1))
        {
            ROS_ERROR("[%s] Cannot open calibration visualization. "
                      "No extrinsic sensor pose available.",
                      guiNodeName_.c_str());

            return false;
        }

        //--- construct transform from pose
        tf::Transform ref2LocalTransform;
        utils::cvtGeometryPose2TfTransform(extrinsicPose, ref2LocalTransform);
        tf::Transform local2RefTransform = ref2LocalTransform.inverse(); // invert to get absolute position and rotation
        double roll, pitch, yaw;
        local2RefTransform.getBasis().getRPY(roll, pitch, yaw);

        //--- construct temporary transform string
        std::stringstream temporaryTransform;
        temporaryTransform.imbue(std::locale("en_US.UTF-8"));
        temporaryTransform << local2RefTransform.getOrigin().x() << " "
                           << local2RefTransform.getOrigin().y() << " "
                           << local2RefTransform.getOrigin().z() << " "
                           << roll << " "
                           << pitch << " "
                           << yaw;

        //--- load nodelet
        nodelet::V_string args;
        nodelet::M_string remappings = {
          {"cloud_0", calibrationMetaData_.ref_topic_name},
          {"cloud_1", calibrationMetaData_.src_topic_name},
        };
        ros::param::set(visualizerNodeletName_ + "/number_of_clouds", 2);
        ros::param::set(visualizerNodeletName_ + "/distance_measure", 0);
        // ros::param::set(visualizerNodeletName_ + "/max_distance", MAX_POINT_DISTANCE);
        // ros::param::set(visualizerNodeletName_ + "/clamp_distance_threshold", 0.1);
        ros::param::set(visualizerNodeletName_ + "/num_nearest_neighbors", 5);
        ros::param::set(visualizerNodeletName_ + "/temp_transform", temporaryTransform.str());
        pNodeletLoader_->load(visualizerNodeletName_,
                              "multisensor_calibration/PointCloud2PointCloudDistanceNodelet",
                              remappings, args);

        return true;
    };

    //--- show progress dialog
    showProgressDialog("Initializing visualizer nodelet ...");

    //--- initialize visualizer asynchronously
    auto initFuture = std::async(initializeAndRunSensorFusion);

    //--- while future is not ready, process QEvents in order for the progress dialog not to freeze
    while (initFuture.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
        QCoreApplication::processEvents();

    if (initFuture.get() == true)
    {

        //--- load dialog
        if (pFusionDialog_ == nullptr)
        {
            pFusionDialog_ = std::make_shared<Rviz3dViewDialog>(pCalibControlWindow_.get());
            pFusionDialog_->setWindowModality(Qt::NonModal);
            pFusionDialog_->setWindowTitle("Sensor Fusion");
            pFusionDialog_->setFixedReferenceFrame((calibrationMetaData_.base_frame_id.empty())
                                                     ? calibrationMetaData_.ref_frame_id
                                                     : calibrationMetaData_.base_frame_id);
            pFusionDialog_->addAxes();
            pFusionDialog_->addRegionsOfInterestCloud(visualizerNodeletName_ + "/cloud_0_enhanced");
            pFusionDialog_->addRegionsOfInterestCloud(visualizerNodeletName_ + "/cloud_1_enhanced");

            //--- connect rejection signal, i.e. close signal of dialog, this will unload nodelet when dialog is closed
            QObject::connect(pFusionDialog_.get(), &QDialog::rejected,
                             [=]()
                             {
                                 pNodeletLoader_->unload(visualizerNodeletName_);

                                 pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled(true);
                                 pCalibControlWindow_->pbVisCalibrationPtr()->setChecked(false);
                             });
        }

        pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled(false);
        pCalibControlWindow_->pbVisCalibrationPtr()->setChecked(true);
        pFusionDialog_->show();

        QMessageBox::information(pFusionDialog_.get(), pFusionDialog_->windowTitle(),
                                 QObject::tr(
                                   "In order to visualize the calibration, the point-wise "
                                   "distance between the two point clouds is calculated and "
                                   "visualized with a rainbow colormap (red -> yellow -> green -> "
                                   "blue -> violet) in the range of [0.0, %1] m. This means, that if the calibration is good,  "
                                   "the point-wise distance in overlapping regions should be small "
                                   "and, in turn, the corresponding points should ideally be "
                                   "highlighted in red.")
                                   .arg(QString::number(MAX_POINT_DISTANCE)));
    }
    else
    {
        pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled(true);
        pCalibControlWindow_->pbVisCalibrationPtr()->setChecked(false);
    }

    hideProgressDialog();
}

//==================================================================================================
bool LidarLidarCalibrationGui::setupGuiElements()
{
    bool isSuccessful = CalibrationGuiBase::setupGuiElements();
    if (!isSuccessful)
        return false;

    pCalibControlWindow_->setWindowTitle(
      QString::fromStdString(CALIB_TYPE_2_STR.at(EXTRINSIC_LIDAR_LIDAR_CALIBRATION)) + " Calibration");

    //--- setup placement guidance dialog at the top-right corner of the display
    pPlacementGuidanceDialog_ = std::make_shared<Rviz3dViewDialog>(pCalibControlWindow_.get());
    if (!pPlacementGuidanceDialog_)
        return false;
    pPlacementGuidanceDialog_->setWindowTitle("Target Placement Guidance");
    pPlacementGuidanceDialog_->move(screenGeometry_.width() / 2, 0);
    pPlacementGuidanceDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                            (screenGeometry_.height() / 2) - titleBarHeight_ - 1);
    pCalibControlWindow_->attachPlacementGuidanceDialog(pPlacementGuidanceDialog_.get());
    pPlacementGuidanceDialog_->show();

    //--- setup source lidar target dialog at the bottom-left corner of the display
    pSrcLidarTargetDialog_ = std::make_shared<Rviz3dViewDialog>(pCalibControlWindow_.get());
    if (!pSrcLidarTargetDialog_)
        return false;
    pSrcLidarTargetDialog_->setWindowTitle("Source LiDAR Target Detections");
    pSrcLidarTargetDialog_->move(0, (screenGeometry_.height() / 2) + (2 * titleBarHeight_));
    pSrcLidarTargetDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                         (screenGeometry_.height() / 2) - titleBarHeight_ - 1);

    pCalibControlWindow_->attachSourceDialog(pSrcLidarTargetDialog_.get());
    pSrcLidarTargetDialog_->show();

    //--- setup reference lidar target dialog at the bottom-right corner of the display
    pRefLidarTargetDialog_ = std::make_shared<Rviz3dViewDialog>(pCalibControlWindow_.get());
    if (!pRefLidarTargetDialog_)
        return false;
    pRefLidarTargetDialog_->setWindowTitle("Reference LiDAR Target Detections");
    pRefLidarTargetDialog_->move(screenGeometry_.width() / 2,
                                 (screenGeometry_.height() / 2) + (2 * titleBarHeight_));
    pRefLidarTargetDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                         (screenGeometry_.height() / 2) - titleBarHeight_ - 1);
    pCalibControlWindow_->attachReferenceDialog(pRefLidarTargetDialog_.get());
    pRefLidarTargetDialog_->show();

    //--- show infinite progress dialog at screen center
    showProgressDialog("Initializing user interface ...");

    return true;
}

} // namespace multisensor_calibration
