/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/Rviz3dViewDialog.h"

// ROS
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/visualization_manager.hpp>

// Qt
#include <QCloseEvent>
#include <QMessageBox>

// multisensor_calibration
#include "ui_ViewDialog.h"

namespace multisensor_calibration
{

//==================================================================================================
Rviz3dViewDialog::Rviz3dViewDialog(QWidget* parent, std::string iNodeAbstractionName) :
  QDialog(parent),
  pUi_(new Ui::ViewDialog),
  isInitialized_(false),
  pRenderPanel_(nullptr),
  pVisManager_(nullptr),
  nodeAbsName_(iNodeAbstractionName),
  fixedReferenceFrame_(""),
  axisReferenceFrames_(),
  cornerCloudTopicNames_(),
  sensorCloudTopicNames_(),
  roisCloudTopicNames_(),
  distanceCloudTopicNames_(),
  targetCloudTopicNames_()
{
    pRosNodeAbs_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(nodeAbsName_);
    pUi_->setupUi(this);
}

//==================================================================================================
Rviz3dViewDialog::~Rviz3dViewDialog()
{
    delete pUi_;
}

//==================================================================================================
bool Rviz3dViewDialog::addAxes(const std::string& iReferenceFrame)
{
    //--- if visualization manager is not null, create display
    if (pVisManager_)
    {
        rviz_common::Display* pAxesDisplay_ =
          pVisManager_->createDisplay(
            "rviz_default_plugins/Axes",
            "Axes " + QString::number(axisReferenceFrames_.size()),
            true);

        if (!iReferenceFrame.empty())
            pAxesDisplay_->subProp("Reference Frame")->setValue(QString::fromStdString(iReferenceFrame));

        rviz_common::Display* pGridDisplay =
          pVisManager_->createDisplay(
            "rviz_default_plugins/Grid",
            "Grid " + QString::number(axisReferenceFrames_.size()),
            true);

        if (!iReferenceFrame.empty())
            pGridDisplay->subProp("Reference Frame")->setValue(QString::fromStdString(iReferenceFrame));
    }

    //--- if reference name not in list, add to list
    if (std::find(axisReferenceFrames_.begin(), axisReferenceFrames_.end(), iReferenceFrame) ==
        axisReferenceFrames_.end())
        axisReferenceFrames_.push_back(iReferenceFrame);

    return true;
}

//==================================================================================================
bool Rviz3dViewDialog::addGuidedPlacementBox(const std::string& iTopicName)
{
    //--- if visualization manager is not null, create display
    if (pVisManager_)
    {
        rviz_common::Display* pMarkerDisplay_ =
          pVisManager_->createDisplay(
            "rviz_default_plugins/Marker",
            "Guided Placement Box " + QString::number(placementBoxTopicNames_.size()),
            true);
        pMarkerDisplay_->subProp("Topic")->setValue(QString::fromStdString(iTopicName));
    }

    //--- if topic name not in list, add to list
    if (std::find(placementBoxTopicNames_.begin(), placementBoxTopicNames_.end(), iTopicName) ==
        placementBoxTopicNames_.end())
        placementBoxTopicNames_.push_back(iTopicName);

    return true;
}

//==================================================================================================
bool Rviz3dViewDialog::addMarkerCornersCloud(const std::string& iTopicName)
{
    //--- if visualization manager is not null, create display
    if (pVisManager_)
    {
        rviz_common::Display* pCloudDisplay_ =
          pVisManager_->createDisplay(
            "rviz_default_plugins/PointCloud2",
            "Marker Corners Cloud " + QString::number(cornerCloudTopicNames_.size()),
            true);
        pCloudDisplay_->subProp("Topic")->setValue(QString::fromStdString(iTopicName));
        pCloudDisplay_->subProp("Use Fixed Frame")->setValue("true");
        pCloudDisplay_->subProp("Color Transformer")->setValue("Intensity");
        pCloudDisplay_->subProp("Color")->setValue("252; 255; 255");
        pCloudDisplay_->subProp("Style")->setValue("Points");
        pCloudDisplay_->subProp("Size (Pixels)")->setValue(16);
    }

    //--- if topic name not in list, add to list
    if (std::find(cornerCloudTopicNames_.begin(), cornerCloudTopicNames_.end(), iTopicName) ==
        cornerCloudTopicNames_.end())
        cornerCloudTopicNames_.push_back(iTopicName);

    return true;
}

//==================================================================================================
bool Rviz3dViewDialog::addRawSensorCloud(const std::string& iTopicName)
{
    //--- if visualization manager is not null, create display
    if (pVisManager_)
    {
        rviz_common::Display* pCloudDisplay_ =
          pVisManager_->createDisplay(
            "rviz_default_plugins/PointCloud2",
            "Raw Sensor Cloud " + QString::number(sensorCloudTopicNames_.size()),
            true);
        pCloudDisplay_->subProp("Topic")->setValue(QString::fromStdString(iTopicName));
        pCloudDisplay_->subProp("Use Fixed Frame")->setValue("true");
        pCloudDisplay_->subProp("Color Transformer")->setValue("FlatColor");
        pCloudDisplay_->subProp("Color")->setValue("255; 255; 255");
        pCloudDisplay_->subProp("Style")->setValue("Points");
        pCloudDisplay_->subProp("Size (Pixels)")->setValue(1);
    }

    //--- if topic name not in list, add to list
    if (std::find(sensorCloudTopicNames_.begin(), sensorCloudTopicNames_.end(), iTopicName) ==
        sensorCloudTopicNames_.end())
        sensorCloudTopicNames_.push_back(iTopicName);

    return true;
}

//==================================================================================================
bool Rviz3dViewDialog::addRegionsOfInterestCloud(const std::string& iTopicName)
{
    //--- if visualization manager is not null, create display
    if (pVisManager_)
    {
        rviz_common::Display* pCloudDisplay_ =
          pVisManager_->createDisplay(
            "rviz_default_plugins/PointCloud2",
            "Regions-of-Interest Cloud " + QString::number(roisCloudTopicNames_.size()),
            true);
        pCloudDisplay_->subProp("Topic")->setValue(QString::fromStdString(iTopicName));
        pCloudDisplay_->subProp("Use Fixed Frame")->setValue("true");
        pCloudDisplay_->subProp("Color Transformer")->setValue("Intensity");
        pCloudDisplay_->subProp("Color")->setValue("255; 255; 255");
        pCloudDisplay_->subProp("Style")->setValue("Points");
        pCloudDisplay_->subProp("Size (Pixels)")->setValue(3);
    }

    //--- if topic name not in list, add to list
    if (std::find(roisCloudTopicNames_.begin(), roisCloudTopicNames_.end(), iTopicName) ==
        roisCloudTopicNames_.end())
        roisCloudTopicNames_.push_back(iTopicName);

    return true;
}

//==================================================================================================
bool Rviz3dViewDialog::addPointWiseDistanceCloud(const std::string& iTopicName)
{
    //--- if visualization manager is not null, create display
    if (pVisManager_)
    {
        rviz_common::Display* pCloudDisplay_ =
          pVisManager_->createDisplay(
            "rviz_default_plugins/PointCloud2",
            "Point-Wise Distance Cloud " + QString::number(distanceCloudTopicNames_.size()),
            true);
        pCloudDisplay_->subProp("Topic")->setValue(QString::fromStdString(iTopicName));
        pCloudDisplay_->subProp("Use Fixed Frame")->setValue("true");
        pCloudDisplay_->subProp("Color Transformer")->setValue("Intensity");
        pCloudDisplay_->subProp("Color")->setValue("255; 255; 255");
        pCloudDisplay_->subProp("Style")->setValue("Points");
        pCloudDisplay_->subProp("Size (Pixels)")->setValue(3);
    }

    //--- if topic name not in list, add to list
    if (std::find(distanceCloudTopicNames_.begin(), distanceCloudTopicNames_.end(), iTopicName) ==
        distanceCloudTopicNames_.end())
        distanceCloudTopicNames_.push_back(iTopicName);

    return true;
}

//==================================================================================================
bool Rviz3dViewDialog::addCalibTargetCloud(const std::string& iTopicName)
{
    //--- if visualization manager is not null, create display
    if (pVisManager_)
    {
        rviz_common::Display* pCloudDisplay_ =
          pVisManager_->createDisplay(
            "rviz_default_plugins/PointCloud2",
            "Calibration Target Cloud " + QString::number(targetCloudTopicNames_.size()),
            true);
        pCloudDisplay_->subProp("Topic")->setValue(QString::fromStdString(iTopicName));
        pCloudDisplay_->subProp("Use Fixed Frame")->setValue("true");
        pCloudDisplay_->subProp("Color Transformer")->setValue("FlatColor");
        pCloudDisplay_->subProp("Color")->setValue("252; 233; 79");
        pCloudDisplay_->subProp("Style")->setValue("Points");
        pCloudDisplay_->subProp("Size (Pixels)")->setValue(5);
    }

    //--- if topic name not in list, add to list
    if (std::find(targetCloudTopicNames_.begin(), targetCloudTopicNames_.end(), iTopicName) ==
        targetCloudTopicNames_.end())
        targetCloudTopicNames_.push_back(iTopicName);

    return true;
}

//==================================================================================================
void Rviz3dViewDialog::closeEvent(QCloseEvent* closeEvent)
{
    if (isInitialized_ && pRenderPanel_)
    {
        pUi_->verticalLayout->removeWidget(pRenderPanel_.get());

        pRenderPanel_->close();
        pRenderPanel_.reset();

        if (pVisManager_)
            pVisManager_.reset();

        isInitialized_ = false;
    }

    QDialog::closeEvent(closeEvent);
}

//==================================================================================================
bool Rviz3dViewDialog::setFixedReferenceFrame(const std::string& iFrameId)
{
    if (pVisManager_)
        pVisManager_->setFixedFrame(QString::fromStdString(iFrameId));

    fixedReferenceFrame_ = iFrameId;

    return true;
}

//==================================================================================================
bool Rviz3dViewDialog::setView(const EViews& iView)
{
    if (pVisManager_)
    {
        switch (iView)
        {
        default:
        case ORBIT:
        {
            pVisManager_->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/Orbit");
        }
        break;
        case TOP_DOWN:
            pVisManager_->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/TopDownOrtho");
            break;
        }
    }

    return true;
}

//==================================================================================================
void Rviz3dViewDialog::showEvent(QShowEvent* showEvent)
{
    if (!isInitialized_)
        initRenderPanel();

    QDialog::showEvent(showEvent);
}

//==================================================================================================
void Rviz3dViewDialog::initRenderPanel()
{
    QApplication::processEvents();
    pRenderPanel_ = std::make_shared<rviz_common::RenderPanel>(this);
    QApplication::processEvents();
    pRenderPanel_->getRenderWindow()->initialize();

    auto clock   = pRosNodeAbs_->get_raw_node()->get_clock();
    pVisManager_ = std::make_shared<rviz_common::VisualizationManager>(pRenderPanel_.get(), pRosNodeAbs_, nullptr, clock);
    pRenderPanel_->initialize(pVisManager_.get());
    QApplication::processEvents();

    pVisManager_->initialize();
    pVisManager_->startUpdate();

    auto tm = pVisManager_->getToolManager();
    tm->addTool("rviz_default_plugins/MoveCamera");

    // //--- add panel to layout
    pUi_->verticalLayout->addWidget(pRenderPanel_.get());

    //--- setup content
    if (!fixedReferenceFrame_.empty())
        setFixedReferenceFrame(fixedReferenceFrame_);
    for (std::string frameId : axisReferenceFrames_)
        addAxes(frameId);
    for (std::string topicName : cornerCloudTopicNames_)
        addMarkerCornersCloud(topicName);
    for (std::string topicName : sensorCloudTopicNames_)
        addRawSensorCloud(topicName);
    for (std::string topicName : roisCloudTopicNames_)
        addRegionsOfInterestCloud(topicName);
    for (std::string topicName : distanceCloudTopicNames_)
        addPointWiseDistanceCloud(topicName);
    for (std::string topicName : targetCloudTopicNames_)
        addCalibTargetCloud(topicName);

    isInitialized_ = true;
}

} // namespace multisensor_calibration
