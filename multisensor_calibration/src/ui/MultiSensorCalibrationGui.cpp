/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/MultiSensorCalibrationGui.h"

// Qt
#include <QApplication>
#include <QMessageBox>
#include <QObject>
#include <cstddef>
#include <type_traits>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/ui/CalibrationGuiBase.h"
#include "multisensor_calibration/ui/CameraLidarCalibrationGui.h"
#include "multisensor_calibration/ui/CameraReferenceCalibrationGui.h"
#include "multisensor_calibration/ui/GuiBase.h"
#include "multisensor_calibration/ui/LidarLidarCalibrationGui.h"
#include "multisensor_calibration/ui/LidarReferenceCalibrationGui.h"

#include "multisensor_calibration/calibration/ExtrinsicCameraCameraCalibration.h"
#include "multisensor_calibration/calibration/ExtrinsicCameraLidarCalibration.h"
#include "multisensor_calibration/calibration/ExtrinsicCameraReferenceCalibration.h"
#include "multisensor_calibration/calibration/ExtrinsicLidarLidarCalibration.h"
#include "multisensor_calibration/calibration/ExtrinsicLidarReferenceCalibration.h"

#include "multisensor_calibration/guidance/GuidedCameraLidarTargetPlacementNode.h"
#include "multisensor_calibration/guidance/GuidedLidarLidarTargetPlacementNode.h"

namespace multisensor_calibration
{

//==================================================================================================
MultiSensorCalibrationGui::MultiSensorCalibrationGui(const std::string& iAppTitle,
                                                     const std::string& iGuiSubNamespace) :
  GuiBase(iAppTitle, iGuiSubNamespace),
  pCalibConfigDialog_(nullptr),
  pCalibrationGui_(nullptr),
  pCalibration_(nullptr)
{
}

//==================================================================================================
MultiSensorCalibrationGui::~MultiSensorCalibrationGui()
{
    rclcpp::shutdown();
#ifdef MULTI_THREADED
    if (guidanceThread_.joinable())
        guidanceThread_.join();
    if (calibrationThread_.joinable())
        calibrationThread_.join();
#endif
}

//==================================================================================================
bool MultiSensorCalibrationGui::init(const std::shared_ptr<rclcpp::Executor>& ipExec,
                                     const rclcpp::NodeOptions& iNodeOpts)
{
    // if (!GuiBase::init(ipExec, iNodeOpts))
    //     return false;
    if (ipExec == nullptr)
        return false;
    pExecutor_ = ipExec;

    nodeOptions_ = rclcpp::NodeOptions(iNodeOpts);

    pCalibConfigDialog_ = std::make_shared<CalibrationConfigDialog>();

    connect(pCalibConfigDialog_.get(), &QDialog::accepted,
            this, &MultiSensorCalibrationGui::handleConfigDialogAccepted);
    connect(this, &GuiBase::rosLoopTerminated,
            this, &MultiSensorCalibrationGui::handleRosLoopTerminated);

    pCalibConfigDialog_->show();

    return true;
}

//==================================================================================================
void MultiSensorCalibrationGui::runExtrinsicCameraLidarCalibration()
{
    runExtrinsicCalibration<
      multisensor_calibration::ExtrinsicCameraLidarCalibration,
      multisensor_calibration::GuidedCameraLidarTargetPlacementNode,
      CameraLidarCalibrationGui>();
}

//==================================================================================================
void MultiSensorCalibrationGui::runExtrinsicCameraReferenceCalibration()
{
    runExtrinsicCalibration<
      multisensor_calibration::ExtrinsicCameraReferenceCalibration,
      multisensor_calibration::GuidedCameraLidarTargetPlacementNode,
      CameraReferenceCalibrationGui>();
}

//==================================================================================================
void MultiSensorCalibrationGui::runExtrinsicLidarLidarCalibration()
{
    runExtrinsicCalibration<
      multisensor_calibration::ExtrinsicLidarLidarCalibration,
      multisensor_calibration::GuidedLidarLidarTargetPlacementNode,
      LidarLidarCalibrationGui>();
}

//==================================================================================================
void MultiSensorCalibrationGui::runExtrinsicLidarReferenceCalibration()
{
    runExtrinsicCalibration<
      multisensor_calibration::ExtrinsicLidarReferenceCalibration,
      multisensor_calibration::GuidedLidarLidarTargetPlacementNode,
      LidarReferenceCalibrationGui>();
}

//==================================================================================================

void MultiSensorCalibrationGui::runExtrinsicCameraCameraCalibration()
{
    /* @TODO: Guidance, Gui for Camera-Camera */
    runExtrinsicCalibration<
      multisensor_calibration::ExtrinsicCameraCameraCalibration,
      MultiSensorCalibrationGui::NULLTYPE,
      MultiSensorCalibrationGui::NULLTYPE>();
}

//==================================================================================================
template <typename CalibrationType, typename GuidanceType, typename GuiType>
typename std::enable_if<
  TYPE_INHERTIS_FROM_OR_NULLTYPE(CalibrationGuiBase, GuiType) &&
    TYPE_INHERITS(CalibrationBase, CalibrationType) &&
    TYPE_INHERTIS_FROM_OR_NULLTYPE(GuidanceBase, GuidanceBase),
  void>::type
MultiSensorCalibrationGui::runExtrinsicCalibration()
{
    if (!pExecutor_)
        return;

    constexpr bool hasGuidance = TYPE_INHERITS(GuidanceBase, GuidanceType);
    constexpr bool hasGui      = TYPE_INHERITS(GuiBase, GuiType);

    std::vector<rclcpp::Parameter> parameterVector = {};

    //--- set parameters for calibration node
    for (const auto& opt : pCalibConfigDialog_->getBoolTypedCalibrationOptions())
        parameterVector.emplace_back(opt.first, opt.second);
    for (const auto& opt : pCalibConfigDialog_->getDoubleTypedCalibrationOptions())
        parameterVector.emplace_back(opt.first, opt.second);
    for (const auto& opt : pCalibConfigDialog_->getIntTypedCalibrationOptions())
        parameterVector.emplace_back(opt.first, opt.second);
    for (const auto& opt : pCalibConfigDialog_->getStringTypedCalibrationOptions())
        parameterVector.emplace_back(opt.first, opt.second);

    auto options = rclcpp::NodeOptions(nodeOptions_);
    options.parameter_overrides(parameterVector);
    options.use_intra_process_comms(true);

#ifdef MULTI_THREADED
    if constexpr (hasGuidance)
    {
        guidanceThread_ = std::thread(
          [](rclcpp::NodeOptions options, std::string appTitle)
          {
              auto guidanceNode = std::make_shared<GuidanceType>(appTitle, options);
              rclcpp::spin(guidanceNode);
          },
          options, appTitle_);
    }

    calibrationThread_ = std::thread(
      [](rclcpp::NodeOptions options, std::string appTitle)
      {
          auto calibrationNode = std::make_shared<CalibrationType>(appTitle, options);
          rclcpp::spin(calibrationNode);
      },
      options, appTitle_);

#else
    if constexpr (hasGuidance)
    {
        auto pGuidance = std::make_shared<GuidanceType>(appTitle_, options);
        pExecutor_->add_node(pGuidance);
        pGuidance_ = pGuidance;
    }
    auto pCalibration = std::make_shared<CalibrationType>(appTitle_, options);
    pExecutor_->add_node(pCalibration);
    pCalibration_ = pCalibration;
#endif

    //--- load gui
    if constexpr (hasGui)
    {
        pCalibrationGui_ = std::make_shared<GuiType>(appTitle_, multisensor_calibration::GUI_SUB_NAMESPACE);
        pCalibrationGui_->init(pExecutor_, nodeOptions_);
    }
}

//==================================================================================================
void MultiSensorCalibrationGui::handleConfigDialogAccepted()
{
    switch (pCalibConfigDialog_->selectedCalibrationType())
    {
    default:
    case EXTRINSIC_CAMERA_LIDAR_CALIBRATION:
    {
        runExtrinsicCameraLidarCalibration();
    }
    break;

    case EXTRINSIC_CAMERA_REFERENCE_CALIBRATION:
    {
        runExtrinsicCameraReferenceCalibration();
    }
    break;

    case EXTRINSIC_LIDAR_LIDAR_CALIBRATION:
    {
        runExtrinsicLidarLidarCalibration();
    }
    break;

    case EXTRINSIC_LIDAR_REFERENCE_CALIBRATION:
    {
        runExtrinsicLidarReferenceCalibration();
    }
    break;

    case STEREO_CAMERA_CALIBRATION:
    {
        runExtrinsicCameraCameraCalibration();
    }
    break;
    }
}

//==================================================================================================
void MultiSensorCalibrationGui::handleRosLoopTerminated()
{
    pCalibConfigDialog_->reject();
}

} // namespace multisensor_calibration
