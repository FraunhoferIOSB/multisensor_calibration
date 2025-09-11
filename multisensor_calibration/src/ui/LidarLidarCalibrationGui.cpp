/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/LidarLidarCalibrationGui.h"

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
#include "multisensor_calibration/common/utils.hpp"
#include "multisensor_calibration/common/common.h"
#include <multisensor_calibration_interface/srv/sensor_extrinsics.hpp>

using namespace multisensor_calibration_interface::srv;
namespace multisensor_calibration
{

//==================================================================================================
LidarLidarCalibrationGui::LidarLidarCalibrationGui(const std::string& iAppTitle,
                                                   const std::string& iGuiSubNamespace) :
  CalibrationGuiBase(iAppTitle, iGuiSubNamespace),
  pPlacementGuidanceDialog_(nullptr),
  pSrcLidarTargetDialog_(nullptr),
  pRefLidarTargetDialog_(nullptr),
  pVisualizerNode_(nullptr)
{
}

//==================================================================================================
LidarLidarCalibrationGui::~LidarLidarCalibrationGui()
{
}

//==================================================================================================
void LidarLidarCalibrationGui::initializeGuiContents()
{
    CalibrationGuiBase::initializeGuiContents();

    //--- initialize content of placement guidance dialog
    if (pPlacementGuidanceDialog_)
    {
        pPlacementGuidanceDialog_->setFixedReferenceFrame((pCalibrationMetaData_->base_frame_id.empty())
                                                            ? pCalibrationMetaData_->ref_frame_id
                                                            : pCalibrationMetaData_->base_frame_id);
        pPlacementGuidanceDialog_->addAxes();
        pPlacementGuidanceDialog_->addRawSensorCloud(pCalibrationMetaData_->ref_topic_name);
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
    if (pRefLidarTargetDialog_)
    {
        pRefLidarTargetDialog_->setWindowTitle(
          QString::fromStdString(pCalibrationMetaData_->ref_sensor_name));

        pRefLidarTargetDialog_->setFixedReferenceFrame((pCalibrationMetaData_->base_frame_id.empty())
                                                         ? pCalibrationMetaData_->ref_frame_id
                                                         : pCalibrationMetaData_->base_frame_id);
        pRefLidarTargetDialog_->addAxes();
        pRefLidarTargetDialog_->addRawSensorCloud(pCalibrationMetaData_->ref_topic_name);
        pRefLidarTargetDialog_
          ->addRegionsOfInterestCloud(calibratorNodeName_ +
                                      "/" + pCalibrationMetaData_->ref_sensor_name +
                                      "/" + ROIS_CLOUD_TOPIC_NAME);
        pRefLidarTargetDialog_
          ->addCalibTargetCloud(calibratorNodeName_ +
                                "/" + pCalibrationMetaData_->ref_sensor_name +
                                "/" + TARGET_PATTERN_CLOUD_TOPIC_NAME);
        pRefLidarTargetDialog_
          ->addMarkerCornersCloud(calibratorNodeName_ +
                                  "/" + pCalibrationMetaData_->ref_sensor_name +
                                  "/" + MARKER_CORNERS_TOPIC_NAME);
    }

    //--- hide progress dialog
    hideProgressDialog();
}
//==================================================================================================
void LidarLidarCalibrationGui::loadVisualizer()
{
    // Function to initialize visualizer node and dialog
    auto initializeAndRunSensorFusion = [&]() -> bool
    {
        if (!pVisualizerNode_)
        {
            //--- get sensor extrinsics
            auto extrinsicsClient            = pNode_->create_client<SensorExtrinsics>(calibratorNodeName_ +
                                                                                       "/" + REQUEST_SENSOR_EXTRINSICS_SRV_NAME);
            auto extrinsicRequest            = std::make_shared<SensorExtrinsics::Request>();
            extrinsicRequest->extrinsic_type = SensorExtrinsics::Request::SENSOR_2_SENSOR;

            auto extrinsicResponse = extrinsicsClient->async_send_request(extrinsicRequest);

            auto retCode = utils::doWhileWaiting(pExecutor_, extrinsicResponse, [&]()
                                                 { QCoreApplication::processEvents(); }, 100);

            if (retCode != rclcpp::FutureReturnCode::SUCCESS)
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

            //--- coefficients of the source lidar <--> reference lidar transformation
            std::vector<double> temporaryTransformCoeffs = {
              extrinsicPose.position.x, extrinsicPose.position.y,
              extrinsicPose.position.z, extrinsicPose.orientation.x,
              extrinsicPose.orientation.y, extrinsicPose.orientation.z,
              extrinsicPose.orientation.w};

            //--- load nodel
            rclcpp::NodeOptions options;

            options.parameter_overrides({
              rclcpp::Parameter("number_of_clouds", 2),
              rclcpp::Parameter("distance_measure", 0),
              // // rclcpp::Parameter("max_distance", 2.0),
              // rclcpp::Parameter("clamp_distance_threshold", 0.1),
              rclcpp::Parameter("num_nearest_neighbors", 5),
              rclcpp::Parameter("temp_transform", temporaryTransformCoeffs),
            });
            options.use_intra_process_comms(true);
            std::vector<std::string> remapping_arguments = {
              "cloud_1:=" + pCalibrationMetaData_->ref_topic_name,
              "cloud_0:=" + pCalibrationMetaData_->src_topic_name,
              "calibration:=" + appTitle_ + "/" + CALIB_RESULT_TOPIC_NAME};

            options.arguments(remapping_arguments);
            options.use_intra_process_comms(true);
            pVisualizerNode_ = std::make_shared<multisensor_calibration::visualizers::PointCloud2PointCloudDistanceNode>(options, visualizerNodeName_);
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
            pFusionDialog_ = std::make_shared<Rviz3dViewDialog>(pCalibControlWindow_.get(), "visualizer3dViewer");
            pFusionDialog_->setWindowModality(Qt::NonModal);
            pFusionDialog_->setWindowTitle("Sensor Fusion");
            pFusionDialog_->setFixedReferenceFrame((pCalibrationMetaData_->base_frame_id.empty())
                                                     ? pCalibrationMetaData_->ref_frame_id
                                                     : pCalibrationMetaData_->base_frame_id);
            pFusionDialog_->addAxes();
            pFusionDialog_->addRegionsOfInterestCloud("cloud_0_enhanced");
            pFusionDialog_->addRegionsOfInterestCloud("cloud_1_enhanced");

            //--- connect rejection signal, i.e. close signal of dialog, this will unload node when dialog is closed
            QObject::connect(pFusionDialog_.get(), &QDialog::rejected,
                             [=]()
                             {
                                 pExecutor_->remove_node(pVisualizerNode_);

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
                                   "blue -> violet). This means, that if the calibration is good,  "
                                   "the point-wise distance in overlapping regions should be small "
                                   "and, in turn, the corresponding points should ideally be "
                                   "highlighted in red."));
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
    pRefLidarTargetDialog_ = std::make_shared<Rviz3dViewDialog>(pCalibControlWindow_.get());
    if (!pRefLidarTargetDialog_)
        return false;
    pRefLidarTargetDialog_->setWindowTitle("Reference LiDAR Target Detections");
    pRefLidarTargetDialog_->move(screenGeometry_.topLeft() + QPoint(screenGeometry_.width() / 2,
                                                                    (screenGeometry_.height() / 2) + (2 * titleBarHeight_)));
    pRefLidarTargetDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                         (screenGeometry_.height() / 2) - titleBarHeight_ - 1);
    pCalibControlWindow_->attachReferenceDialog(pRefLidarTargetDialog_.get());
    pRefLidarTargetDialog_->show();

    //--- show infinite progress dialog at screen center
    showProgressDialog("Initializing user interface ...");

    return true;
}

} // namespace multisensor_calibration
