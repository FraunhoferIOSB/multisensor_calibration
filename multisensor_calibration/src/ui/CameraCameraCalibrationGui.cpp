/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/CameraCameraCalibrationGui.h"

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
#include "multisensor_calibration/common/utils.hpp"
#include <multisensor_calibration_interface/srv/camera_intrinsics.hpp>
#include <multisensor_calibration_interface/srv/sensor_extrinsics.hpp>

using namespace multisensor_calibration_interface::srv;
namespace multisensor_calibration
{

//==================================================================================================
CameraCameraCalibrationGui::CameraCameraCalibrationGui(const std::string& iAppTitle,
                                                     const std::string& iGuiSubNamespace) :
  CalibrationGuiBase(iAppTitle, iGuiSubNamespace),
  pPlacementGuidanceDialog_(nullptr),
  psrcCameraTargetDialog_(nullptr),
  prefCameraTargetDialog_(nullptr),
  pFusionDialog_(nullptr)
{
    hasCalibVisualizer_ = false;
}

//==================================================================================================
CameraCameraCalibrationGui::~CameraCameraCalibrationGui()
{
}

//==================================================================================================
void CameraCameraCalibrationGui::initializeGuiContents()
{
    CalibrationGuiBase::initializeGuiContents();

    //--- initialize content of placement guidance dialog
    if (pPlacementGuidanceDialog_)
    {
        pPlacementGuidanceDialog_->subscribeToImageTopic(pNode_.get(),
                                                         guidanceNodeName_ +
                                                           "/" + PLACEMENT_GUIDANCE_TOPIC_NAME);
    }

    //--- initialize content of src camera target dialog
    if (psrcCameraTargetDialog_)
    {
        psrcCameraTargetDialog_->setWindowTitle(
          QString::fromStdString(pCalibrationMetaData_->src_sensor_name));

        psrcCameraTargetDialog_->subscribeToImageTopic(pNode_.get(),
                                                    calibratorNodeName_ +
                                                      "/" + pCalibrationMetaData_->src_sensor_name +
                                                      "/" + ANNOTATED_CAMERA_IMAGE_TOPIC_NAME);
    }

    //--- initialize content of ref camera target visualization
    if (prefCameraTargetDialog_)
    {
        prefCameraTargetDialog_->setWindowTitle(
          QString::fromStdString(pCalibrationMetaData_->ref_sensor_name));

        prefCameraTargetDialog_->subscribeToImageTopic(pNode_.get(),
                                                    calibratorNodeName_ +
                                                      "/" + pCalibrationMetaData_->ref_sensor_name +
                                                      "/" + ANNOTATED_CAMERA_IMAGE_TOPIC_NAME);
    }

    //--- hide progress dialog
    hideProgressDialog();
}

//==================================================================================================
void CameraCameraCalibrationGui::loadVisualizer()
{
    return;
    // Function to initialize visualizer node and dialog
    auto initializeAndRunSensorFusion = [&]() -> bool
    {
        return false;
    };

    //--- show progress dialog
    showProgressDialog("Initializing visualizer node ...");

    //--- initialize visualizer asynchronously
    auto initSuccess = initializeAndRunSensorFusion();

    if (initSuccess)
    {
        //--- load dialog
        if (pFusionDialog_ == nullptr)
        {
            pFusionDialog_ = std::make_shared<ImageViewDialog>(pCalibControlWindow_.get());
            pFusionDialog_->setWindowModality(Qt::NonModal);
            pFusionDialog_->setWindowTitle("Sensor Fusion");
            pFusionDialog_->subscribeToImageTopic(pNode_.get(), "fused_image");

            //--- connect rejection signal, i.e. close signal of dialog, this will unload node when dialog is closed
            QObject::connect(pFusionDialog_.get(), &QDialog::rejected,
                             [=]()
                             {
                                 if (pVisualizerNode_)
                                 {
                                     pExecutor_->remove_node(pVisualizerNode_);
                                 }

                                 pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled(true);
                                 pCalibControlWindow_->pbVisCalibrationPtr()->setChecked(false);
                             });
        }

        pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled(false);
        pCalibControlWindow_->pbVisCalibrationPtr()->setChecked(true);
        pFusionDialog_->show();

        QMessageBox::information(pFusionDialog_.get(), pFusionDialog_->windowTitle(),
                                 QObject::tr(
                                   "@TODO"));
    }
    else
    {
        pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled(true);
        pCalibControlWindow_->pbVisCalibrationPtr()->setChecked(false);
    }

    hideProgressDialog();
}

//==================================================================================================
bool CameraCameraCalibrationGui::setupGuiElements()
{
    bool isSuccessful = CalibrationGuiBase::setupGuiElements();
    if (!isSuccessful)
        return false;

    pCalibControlWindow_->setWindowTitle(
      QString::fromStdString(CALIB_TYPE_2_STR.at(STEREO_CAMERA_CALIBRATION)) + " Calibration");

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

    //--- setup src camera target dialog at the bottom-left corner of the display
    psrcCameraTargetDialog_ = std::make_shared<ImageViewDialog>(pCalibControlWindow_.get());
    if (!psrcCameraTargetDialog_)
        return false;
    psrcCameraTargetDialog_->setWindowTitle("Src Camera Target Detections");
    psrcCameraTargetDialog_->move(screenGeometry_.topLeft() + QPoint(0, (screenGeometry_.height() / 2) + (2 * titleBarHeight_)));
    psrcCameraTargetDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                       (screenGeometry_.height() / 2) - titleBarHeight_ - 1);

    pCalibControlWindow_->attachSourceDialog(psrcCameraTargetDialog_.get());
    psrcCameraTargetDialog_->show();

    //--- setup ref camera target dialog at the bottom-right corner of the display
    prefCameraTargetDialog_ = std::make_shared<ImageViewDialog>(pCalibControlWindow_.get());
    if (!prefCameraTargetDialog_)
        return false;
    prefCameraTargetDialog_->setWindowTitle("Ref Camera Target Detections");
    prefCameraTargetDialog_->move(screenGeometry_.topLeft() + QPoint(screenGeometry_.width() / 2,
                                                                 (screenGeometry_.height() / 2) + (2 * titleBarHeight_)));
    prefCameraTargetDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                      (screenGeometry_.height() / 2) - titleBarHeight_ - 1);
    pCalibControlWindow_->attachReferenceDialog(prefCameraTargetDialog_.get());
    prefCameraTargetDialog_->show();

    //--- show infinite progress dialog at screen center
    showProgressDialog("Initializing user interface ...");

    return true;
}

} // namespace multisensor_calibration
