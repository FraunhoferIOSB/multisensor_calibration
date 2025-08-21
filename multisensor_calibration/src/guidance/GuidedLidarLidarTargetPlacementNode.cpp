/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/guidance/GuidedLidarLidarTargetPlacementNode.h"

// Std
#define USE_MATH_DEFINES
#include <cmath>

// multisensor_calibration
#include "multisensor_calibration/common/utils.hpp"

namespace multisensor_calibration
{

using namespace utils;

//==================================================================================================
GuidedLidarLidarTargetPlacementNode::GuidedLidarLidarTargetPlacementNode(std::string appTitle, rclcpp::NodeOptions iOptions) :
  GuidanceBase(this),
  rclcpp::Node(appTitle + "_" + GUIDANCE_SUB_NAMESPACE, iOptions)
{
    resetNextTargetPose();

    appTitle_           = appTitle;
    calibratorNodeName_ = appTitle_;

    init();
}

//==================================================================================================
GuidedLidarLidarTargetPlacementNode::~GuidedLidarLidarTargetPlacementNode()
{
}

//==================================================================================================
void GuidedLidarLidarTargetPlacementNode::computeExtrinsicFovBoundingPlanes()
{
    // TODO: implement
}

//==================================================================================================
bool GuidedLidarLidarTargetPlacementNode::computeIntrinsicFovBoundingPlanes()
{
    // TODO: implement

    return true;
}

//==================================================================================================
void GuidedLidarLidarTargetPlacementNode::computeNextTargetPose()
{
    // TODO: implement
}

//==================================================================================================
bool GuidedLidarLidarTargetPlacementNode::initializePublishers()
{
    //--- advertise topic of guidance box
    guidanceBoxPub_ = this->create_publisher<TargetPlacementBox_Message_T>(std::string(this->get_name()) + "/" + PLACEMENT_GUIDANCE_TOPIC_NAME, 10);

    return true;
}

//==================================================================================================
bool GuidedLidarLidarTargetPlacementNode::initializeSubscribers()
{
    //--- call parent method
    bool isSuccessful = GuidanceBase::initializeSubscribers();
    if (!isSuccessful)
        return false;

    return true;
}

//==================================================================================================
bool GuidedLidarLidarTargetPlacementNode::initializeTimers()
{
    bool isSuccessful = GuidanceBase::initializeTimers();
    if (!isSuccessful)
        return false;

    //--- initialize trigger to call routine to load the robot workspace
    publishGuidanceBoxTimer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&GuidedLidarLidarTargetPlacementNode::publishGuidanceBox, this), nullptr, true);

    return true;
}

//==================================================================================================
void GuidedLidarLidarTargetPlacementNode::init()
{
    //--- read launch parameters
    isInitialized_ &= readLaunchParameters();

    //--- initialize publishers
    isInitialized_ &= initializePublishers();

    //--- initialize services
    isInitialized_ &= initializeServices();

    //--- initialize timers
    isInitialized_ &= initializeTimers();

    //--- start ros event loop
    if (isInitialized_)
    {
        pCalibMetaDataTimer_->reset();
    }
}

//==================================================================================================
void GuidedLidarLidarTargetPlacementNode::publishGuidanceBox() const
{
    if (!isInitialized_ || !initialPoseReceived_)
        return;

    visualization_msgs::msg::Marker markerMsg;

    //--- header
    markerMsg.header.frame_id = pCalibrationMetaData_->ref_frame_id;
    markerMsg.header.stamp    = this->get_clock()->now();

    //--- Set the namespace and id for this marker.  This serves to create a unique ID
    //--- Any marker sent with the same namespace and id will overwrite the old one
    markerMsg.ns = "target_placement_box";
    markerMsg.id = 0;

    //--- Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    markerMsg.type = visualization_msgs::msg::Marker::CUBE;

    //--- Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    markerMsg.action = visualization_msgs::msg::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    tf2::Transform transform;
    setTfTransformFromCameraExtrinsics(nextTargetPose_, transform);
    transform = transform.inverse(); // invert to get absolute pose relative to reference
    utils::cvtTfTransform2GeometryPose(transform, markerMsg.pose);

    //--- Set the scale of the marker -- 1x1x1 here means 1m on a side
    markerMsg.scale.x = (1.1 * calibrationTarget_.boardSize.width);
    markerMsg.scale.y = (1.1 * calibrationTarget_.boardSize.height);
    markerMsg.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    markerMsg.color.r = 0.0f;
    markerMsg.color.g = 0.0f;
    markerMsg.color.b = 1.0f;
    markerMsg.color.a = 0.6f;

    guidanceBoxPub_->publish(markerMsg);
}

//==================================================================================================
void GuidedLidarLidarTargetPlacementNode::resetNextTargetPose()
{
    nextTargetPose_.setTransfDirection(lib3d::Extrinsics::LOCAL_2_REF);
    nextTargetPose_.setTranslationVec(0, -3, 0);
    nextTargetPose_.setRotationMat(lib3d::Rotation::createRotationX_deg(90) *
                                   lib3d::Rotation::createRotationY_deg(30));
}

} // namespace multisensor_calibration
