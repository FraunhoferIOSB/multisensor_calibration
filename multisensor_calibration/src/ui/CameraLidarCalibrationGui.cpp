/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/CameraLidarCalibrationGui.h"

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
CameraLidarCalibrationGui::CameraLidarCalibrationGui(const std::string& iAppTitle,
                                                     const std::string& iGuiSubNamespace) :
  CalibrationGuiBase(iAppTitle, iGuiSubNamespace),
  pPlacementGuidanceDialog_(nullptr),
  pCameraTargetDialog_(nullptr),
  pLidarTargetDialog_(nullptr),
  pFusionDialog_(nullptr)
{
}

//==================================================================================================
CameraLidarCalibrationGui::~CameraLidarCalibrationGui()
{
}

//==================================================================================================
void CameraLidarCalibrationGui::initializeGuiContents()
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

    //--- initialize content of lidar target visualization
    if (pLidarTargetDialog_)
    {
        pLidarTargetDialog_->setWindowTitle(
          QString::fromStdString(pCalibrationMetaData_->ref_sensor_name));

        pLidarTargetDialog_->setFixedReferenceFrame((pCalibrationMetaData_->base_frame_id.empty())
                                                      ? pCalibrationMetaData_->ref_frame_id
                                                      : pCalibrationMetaData_->base_frame_id);
        pLidarTargetDialog_->addAxes();
        pLidarTargetDialog_->addRawSensorCloud(pCalibrationMetaData_->ref_topic_name);
        pLidarTargetDialog_
          ->addRegionsOfInterestCloud(calibratorNodeName_ +
                                      "/" + pCalibrationMetaData_->ref_sensor_name +
                                      "/" + ROIS_CLOUD_TOPIC_NAME);
        pLidarTargetDialog_
          ->addCalibTargetCloud(calibratorNodeName_ +
                                "/" + pCalibrationMetaData_->ref_sensor_name +
                                "/" + TARGET_PATTERN_CLOUD_TOPIC_NAME);
        pLidarTargetDialog_
          ->addMarkerCornersCloud(calibratorNodeName_ +
                                  "/" + pCalibrationMetaData_->ref_sensor_name +
                                  "/" + MARKER_CORNERS_TOPIC_NAME);
    }

    //--- hide progress dialog
    hideProgressDialog();
}

//==================================================================================================
void CameraLidarCalibrationGui::loadVisualizer()
{
    // Function to initialize visualizer node and dialog
    auto initializeAndRunSensorFusion = [&]() -> bool
    {
        if (!pVisualizerNode_)
        {
            //--- get camera intrinsics to extract image state
            auto intrinsicsClient  = pNode_->create_client<CameraIntrinsics>(calibratorNodeName_ +
                                                                             "/" + REQUEST_CAM_INTRINSICS_SRV_NAME);
            auto intrinsicRequest  = std::make_shared<CameraIntrinsics::Request>();
            auto intrinsicResponse = intrinsicsClient->async_send_request(intrinsicRequest);
            auto retCode           = utils::doWhileWaiting(pExecutor_, intrinsicResponse, [&]()
                                                           { QCoreApplication::processEvents(); }, 100);
            if (retCode != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(pNode_->get_logger(),
                             "[%s] Failed to get camera intrinsics. "
                             "Check if calibration node is initialized!",
                             guiNodeName_.c_str());
                return false;
            }

            //--- get sensor extrinsics
            auto extrinsicsClient            = pNode_->create_client<SensorExtrinsics>(calibratorNodeName_ +
                                                                                       "/" + REQUEST_SENSOR_EXTRINSICS_SRV_NAME);
            auto extrinsicRequest            = std::make_shared<SensorExtrinsics::Request>();
            extrinsicRequest->extrinsic_type = SensorExtrinsics::Request::SENSOR_2_SENSOR;

            auto extrinsicResponse = extrinsicsClient->async_send_request(extrinsicRequest);

            if (pExecutor_->spin_until_future_complete(extrinsicResponse) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(pNode_->get_logger(),
                             "[%s] Failed to get sensor extrinsics. "
                             "Check if calibration node is initialized!",
                             guiNodeName_.c_str());
                return false;
            }
            geometry_msgs::msg::Pose& extrinsicPose = extrinsicResponse.get()->extrinsics;

            //--- check if extrinsic pose is 0
            if (tf2::Vector3(extrinsicPose.position.x,
                             extrinsicPose.position.y,
                             extrinsicPose.position.z) == tf2::Vector3(0, 0, 0) &&
                tf2::Quaternion(extrinsicPose.orientation.x,
                                extrinsicPose.orientation.y,
                                extrinsicPose.orientation.z,
                                extrinsicPose.orientation.w) == tf2::Quaternion(0, 0, 0, 1))
            {
                RCLCPP_ERROR(pNode_->get_logger(),
                             "[%s] Cannot open calibration visualization. "
                             "No extrinsic sensor pose available.",
                             guiNodeName_.c_str());

                return false;
            }

            std::vector<double> temporaryTransformCoeffs = {
              extrinsicPose.position.x, extrinsicPose.position.y,
              extrinsicPose.position.z, extrinsicPose.orientation.x,
              extrinsicPose.orientation.y, extrinsicPose.orientation.z,
              extrinsicPose.orientation.w};

            rclcpp::NodeOptions options;

            options.parameter_overrides({
              rclcpp::Parameter("image_state",
                                intrinsicResponse.get()->image_state),
              rclcpp::Parameter("min_depth", 0.5),
              rclcpp::Parameter("max_depth", 10.0),
              rclcpp::Parameter("temp_transform", temporaryTransformCoeffs),
            });
            options.use_intra_process_comms(true);
            std::vector<std::string> remapping_arguments = {
              //   "--ros-args", "--remap",
              "pointcloud:=" + pCalibrationMetaData_->ref_topic_name,
              "image:=" + pCalibrationMetaData_->src_topic_name,
              "image/camera_info:=" + utils::image2CameraInfoTopic(pCalibrationMetaData_->src_topic_name, ""),
              "calibration:=" + appTitle_ + "/" + CALIB_RESULT_TOPIC_NAME};
            options.arguments(remapping_arguments);
            pVisualizerNode_ = std::make_shared<visualizers::PointCloud2ImageNode>(options, visualizerNodeName_);
        }

        pExecutor_->add_node(pVisualizerNode_);

        return true;
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
                                   "In order to visualize the calibration, the 3D points of the "
                                   "LiDAR sensor are projected into the camera image and colorized "
                                   "according to their distance from the camera. Thus, if the "
                                   "calibration is good, then the structure in the LiDAR scan "
                                   "(which can be derived from the depth coloring) should align "
                                   "with the object in the image."));
    }
    else
    {
        pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled(true);
        pCalibControlWindow_->pbVisCalibrationPtr()->setChecked(false);
    }

    hideProgressDialog();
}

//==================================================================================================
bool CameraLidarCalibrationGui::setupGuiElements()
{
    bool isSuccessful = CalibrationGuiBase::setupGuiElements();
    if (!isSuccessful)
        return false;

    pCalibControlWindow_->setWindowTitle(
      QString::fromStdString(CALIB_TYPE_2_STR.at(EXTRINSIC_CAMERA_LIDAR_CALIBRATION)) + " Calibration");

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

    //--- setup lidar target dialog at the bottom-right corner of the display
    pLidarTargetDialog_ = std::make_shared<Rviz3dViewDialog>(pCalibControlWindow_.get());
    if (!pLidarTargetDialog_)
        return false;
    pLidarTargetDialog_->setWindowTitle("LiDAR Target Detections");
    pLidarTargetDialog_->move(screenGeometry_.topLeft() + QPoint(screenGeometry_.width() / 2,
                                                                 (screenGeometry_.height() / 2) + (2 * titleBarHeight_)));
    pLidarTargetDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                      (screenGeometry_.height() / 2) - titleBarHeight_ - 1);
    pCalibControlWindow_->attachReferenceDialog(pLidarTargetDialog_.get());
    pLidarTargetDialog_->show();

    //--- show infinite progress dialog at screen center
    showProgressDialog("Initializing user interface ...");

    return true;
}

} // namespace multisensor_calibration
