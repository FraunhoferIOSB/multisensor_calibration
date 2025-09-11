/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/LidarReferenceCalibrationGui.h"

// Std
#include <string>

// Qt
#include <QCoreApplication>
#include <QLabel>
#include <QMessageBox>
#include <QObject>

// ROS
#include <tf2/utils.hpp>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"
#include <multisensor_calibration_interface/srv/sensor_extrinsics.hpp>

using namespace multisensor_calibration_interface::srv;
namespace multisensor_calibration
{

//==================================================================================================
LidarReferenceCalibrationGui::LidarReferenceCalibrationGui(const std::string& iAppTitle,
                                                           const std::string& iGuiSubNamespace) :
  CalibrationGuiBase(iAppTitle, iGuiSubNamespace),
  pPlacementGuidanceDialog_(nullptr),
  pSrcLidarTargetDialog_(nullptr),
  pRefObservationDialog_(nullptr)
{
}

//==================================================================================================
LidarReferenceCalibrationGui::~LidarReferenceCalibrationGui()
{
}

//==================================================================================================
void LidarReferenceCalibrationGui::initializeGuiContents()
{
    CalibrationGuiBase::initializeGuiContents();

    //--- initialize content of placement guidance dialog
    if (pPlacementGuidanceDialog_)
    {
        pPlacementGuidanceDialog_->setFixedReferenceFrame((pCalibrationMetaData_->base_frame_id.empty())
                                                            ? pCalibrationMetaData_->ref_frame_id
                                                            : pCalibrationMetaData_->base_frame_id);
        pPlacementGuidanceDialog_->addAxes();
        pPlacementGuidanceDialog_->addRawSensorCloud(pCalibrationMetaData_->src_topic_name);
        pPlacementGuidanceDialog_->addGuidedPlacementBox(guidanceNodeName_ +
                                                         "/" + PLACEMENT_GUIDANCE_TOPIC_NAME);

        if (pCalibrationMetaData_->base_frame_id.empty() == false)
            pPlacementGuidanceDialog_->setView(Rviz3dViewDialog::TOP_DOWN);
    }

    //--- initialize content of source lidar target visualization
    if (pSrcLidarTargetDialog_)
    {
        pSrcLidarTargetDialog_->setWindowTitle(
          QString::fromStdString(pCalibrationMetaData_->src_sensor_name));

        pSrcLidarTargetDialog_->setFixedReferenceFrame((pCalibrationMetaData_->base_frame_id.empty())
                                                         ? pCalibrationMetaData_->src_frame_id
                                                         : pCalibrationMetaData_->base_frame_id);
        pSrcLidarTargetDialog_->addAxes();
        pSrcLidarTargetDialog_->addRawSensorCloud(pCalibrationMetaData_->src_topic_name);
        pSrcLidarTargetDialog_
          ->addRegionsOfInterestCloud(calibratorNodeName_ +
                                      "/" + pCalibrationMetaData_->src_sensor_name +
                                      "/" + ROIS_CLOUD_TOPIC_NAME);
        pSrcLidarTargetDialog_
          ->addCalibTargetCloud(calibratorNodeName_ +
                                "/" + pCalibrationMetaData_->src_sensor_name +
                                "/" + TARGET_PATTERN_CLOUD_TOPIC_NAME);
        pSrcLidarTargetDialog_
          ->addMarkerCornersCloud(calibratorNodeName_ +
                                  "/" + pCalibrationMetaData_->src_sensor_name +
                                  "/" + MARKER_CORNERS_TOPIC_NAME);
    }

    //--- initialize content of reference lidar target visualization
    if (pRefObservationDialog_)
    {
        pRefObservationDialog_->setWindowTitle(
          QString::fromStdString(pCalibrationMetaData_->ref_sensor_name));
        pRefObservationDialog_->setSensorName(pCalibrationMetaData_->ref_sensor_name);
    }

    //--- hide progress dialog
    hideProgressDialog();
}
//==================================================================================================
void LidarReferenceCalibrationGui::loadVisualizer()
{
}

//==================================================================================================
bool LidarReferenceCalibrationGui::setupGuiElements()
{
    bool isSuccessful = CalibrationGuiBase::setupGuiElements();
    if (!isSuccessful)
        return false;

    pCalibControlWindow_->setWindowTitle(
      QString::fromStdString(CALIB_TYPE_2_STR.at(EXTRINSIC_LIDAR_REFERENCE_CALIBRATION)) + " Calibration");

    pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled(false);

    //--- setup placement guidance dialog at the top-right corner of the display
    pPlacementGuidanceDialog_ = std::make_shared<Rviz3dViewDialog>(pCalibControlWindow_.get());
    if (!pPlacementGuidanceDialog_)
        return false;
    pPlacementGuidanceDialog_->setWindowTitle("Target Placement Guidance");
    pPlacementGuidanceDialog_->move(screenGeometry_.topLeft() + QPoint(screenGeometry_.width() / 2, 0));
    pPlacementGuidanceDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                            (screenGeometry_.height() / 2) - titleBarHeight_ - 1);
    pCalibControlWindow_->attachPlacementGuidanceDialog(pPlacementGuidanceDialog_.get());
    pPlacementGuidanceDialog_->show();

    //--- setup source lidar target dialog at the bottom-left corner of the display
    pSrcLidarTargetDialog_ = std::make_shared<Rviz3dViewDialog>(pCalibControlWindow_.get());
    if (!pSrcLidarTargetDialog_)
        return false;
    pSrcLidarTargetDialog_->setWindowTitle("Source LiDAR Target Detections");
    pSrcLidarTargetDialog_->move(screenGeometry_.topLeft() + QPoint(0, (screenGeometry_.height() / 2) + (2 * titleBarHeight_)));
    pSrcLidarTargetDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                         (screenGeometry_.height() / 2) - titleBarHeight_ - 1);

    pCalibControlWindow_->attachSourceDialog(pSrcLidarTargetDialog_.get());
    pSrcLidarTargetDialog_->show();

    //--- setup reference lidar target dialog at the bottom-right corner of the display
    pRefObservationDialog_ =
      std::make_shared<ObservationsViewDialog>(this, pCalibControlWindow_.get());
    if (!pRefObservationDialog_)
        return false;
    pRefObservationDialog_->setWindowTitle("Reference");
    pRefObservationDialog_->move(screenGeometry_.topLeft() + QPoint(screenGeometry_.width() / 2,
                                                                    (screenGeometry_.height() / 2) + (2 * titleBarHeight_)));
    pRefObservationDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                         (screenGeometry_.height() / 2) - titleBarHeight_ - 1);
    pCalibControlWindow_->attachReferenceDialog(pRefObservationDialog_.get());
    pRefObservationDialog_->show();

    //--- show infinite progress dialog at screen center
    showProgressDialog("Initializing user interface ...");

    return true;
}

} // namespace multisensor_calibration
