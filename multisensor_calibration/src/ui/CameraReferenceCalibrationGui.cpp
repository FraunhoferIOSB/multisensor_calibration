/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/CameraReferenceCalibrationGui.h"

// Std
#include <string>

// ROS
#include <tf2/utils.hpp>

// Qt
#include <QCoreApplication>
#include <QMessageBox>
#include <QObject>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"
#include <multisensor_calibration_interface/srv/camera_intrinsics.hpp>
#include <multisensor_calibration_interface/srv/sensor_extrinsics.hpp>

using namespace multisensor_calibration_interface::srv;
namespace multisensor_calibration
{

//==================================================================================================
CameraReferenceCalibrationGui::CameraReferenceCalibrationGui(const std::string& iAppTitle,
                                                             const std::string& iGuiSubNamespace) :
  CalibrationGuiBase(iAppTitle, iGuiSubNamespace),
  pPlacementGuidanceDialog_(nullptr),
  pCameraTargetDialog_(nullptr),
  pRefObservationDialog_(nullptr)
{
}

//==================================================================================================
CameraReferenceCalibrationGui::~CameraReferenceCalibrationGui()
{
}

//==================================================================================================
void CameraReferenceCalibrationGui::initializeGuiContents()
{
    CalibrationGuiBase::initializeGuiContents();

    //--- initialize content of placement guidance dialog
    if (pPlacementGuidanceDialog_)
    {
        pPlacementGuidanceDialog_->subscribeToImageTopic(pNode_.get(),
                                                         guidanceNodeName_ +
                                                           "/" + PLACEMENT_GUIDANCE_TOPIC_NAME);
    }

    //--- initialize content of camera target dialog
    if (pCameraTargetDialog_)
    {
        pCameraTargetDialog_->setWindowTitle(
          QString::fromStdString(pCalibrationMetaData_->src_sensor_name));

        pCameraTargetDialog_->subscribeToImageTopic(pNode_.get(),
                                                    calibratorNodeName_ +
                                                      "/" + pCalibrationMetaData_->src_sensor_name +
                                                      "/" + ANNOTATED_CAMERA_IMAGE_TOPIC_NAME);
    }

    //--- initialize content of reference lidar target visualization
    if (pRefObservationDialog_)
    {
        pRefObservationDialog_->setWindowTitle(
          QString::fromStdString(pCalibrationMetaData_->ref_sensor_name));
        pRefObservationDialog_->setSensorName(pCalibrationMetaData_->ref_sensor_name);
        pRefObservationDialog_->initializeTfListener(pNode_.get());
    }

    //--- hide progress dialog
    hideProgressDialog();
}

//==================================================================================================
void CameraReferenceCalibrationGui::loadVisualizer()
{
}

//==================================================================================================
bool CameraReferenceCalibrationGui::setupGuiElements()
{
    bool isSuccessful = CalibrationGuiBase::setupGuiElements();
    if (!isSuccessful)
        return false;

    pCalibControlWindow_->setWindowTitle(
      QString::fromStdString(CALIB_TYPE_2_STR.at(EXTRINSIC_CAMERA_REFERENCE_CALIBRATION)) + " Calibration");

    pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled(false);

    //--- setup placement guidance dialog at the top-right corner of the display
    pPlacementGuidanceDialog_ = std::make_shared<ImageViewDialog>(pCalibControlWindow_.get());
    if (!pPlacementGuidanceDialog_)
        return false;
    pPlacementGuidanceDialog_->setWindowTitle("Target Placement Guidance");
    pPlacementGuidanceDialog_->move(screenGeometry_.topLeft() + QPoint(screenGeometry_.width() / 2, 0));
    pPlacementGuidanceDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                            (screenGeometry_.height() / 2) - titleBarHeight_ - 1);
    pCalibControlWindow_->attachPlacementGuidanceDialog(pPlacementGuidanceDialog_.get());
    pPlacementGuidanceDialog_->show();

    //--- setup camera target dialog at the bottom-left corner of the display
    pCameraTargetDialog_ = std::make_shared<ImageViewDialog>(pCalibControlWindow_.get());
    if (!pCameraTargetDialog_)
        return false;
    pCameraTargetDialog_->setWindowTitle("Camera Target Detections");
    pCameraTargetDialog_->move(screenGeometry_.topLeft() + QPoint(0, (screenGeometry_.height() / 2) + (2 * titleBarHeight_)));
    pCameraTargetDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                       (screenGeometry_.height() / 2) - titleBarHeight_ - 1);

    pCalibControlWindow_->attachSourceDialog(pCameraTargetDialog_.get());
    pCameraTargetDialog_->show();

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
