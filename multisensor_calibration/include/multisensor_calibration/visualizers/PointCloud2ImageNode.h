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

#ifndef MULTISENSORCALIBRATION_POINTCLOUD2IMAGENODE_H
#define MULTISENSORCALIBRATION_POINTCLOUD2IMAGENODE_H

// Std
#include <memory>
#include <string>

// Eigen
#include <Eigen/Geometry>

// ROS
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

// PCL
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// IOSB
#include <multisensor_calibration/common/lib3D/core/camera.hpp>
#include <multisensor_calibration/common/lib3D/core/image.hpp>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"

namespace multisensor_calibration
{
namespace visualizers
{

/**
 * @ingroup visualizer
 * @brief Class implementing the node to fuse a point cloud and a camera image by projecting the
 * geometric 3D information from the point cloud into the camera image.
 */
class PointCloud2ImageNode : public rclcpp::Node
{
    //--- TYPEDEFS ---//

    typedef message_filters::sync_policies::ApproximateTime<
      InputImage_Message_T, InputCloud_Message_T>
      ImgCloudApproxSync;

    typedef message_filters::sync_policies::ExactTime<
      InputImage_Message_T, InputCloud_Message_T>
      ImgCloudExactSync;

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Default Constructor
     */
    PointCloud2ImageNode(rclcpp::NodeOptions iNodeOptions, std::string iNodeName = "pointcloud2image");

    /**
     * @brief Default Destructor
     */
    virtual ~PointCloud2ImageNode();

    /**
     * @brief Method to compute the camera pose for the pcl::FrustumCulling algorithm.
     *
     * @param[in] iCamera Camera object.
     * @param[out] oFrustumCullingPose 4x4 Matrix specifying camera pose
     */
    void computeCameraPoseForFrustumCulling(const lib3d::Camera& iCamera,
                                            Eigen::Matrix4f& oFrustumCullingPose) const;

    /**
     * @brief Perform frustum culling on point cloud given teh camera object.
     *
     * @param[in] ipCloud Pointer to input cloud
     * @param[in] iCamera Camera object
     * @param[in] iMinDepth Minumum depth
     * @param[in] iMaxDepth Maximum depth
     * @param[out] oCulledPointIndices List of point indices that are within the view frustum of the
     * camera.
     */
    void doFrustumCulling(const pcl::PointCloud<InputPointType>::Ptr& ipCloud,
                          const lib3d::Camera& iCamera,
                          const float& iMinDepth,
                          const float& iMaxDepth,
                          pcl::Indices& oCulledPointIndices) const;

    /**
     * This will use the point cloud and project it into the camera image where each projected point
     * is colorized based on its depth with respect to the camera.
     *
     * The colorization is performed by applying the RAINBOW colormap from
     * [`cv::ColorMap`](https://docs.opencv.org/master/d3/d50/group__imgproc__colormap.html).
     *
     * In order to reduce the size of the point cloud prior to projecting it into the camera image,
     * a frustum culling is performed.
     */
    void onSensorDataReceived(const InputImage_Message_T::ConstSharedPtr& ipImgMsg,
                              const InputCloud_Message_T::ConstSharedPtr& ipCloudMsg);

    /** @brief Callback for the calibration subscription. Will update the temp transofrm
     */
    void onCalibrationReceived(const CalibrationResult_Message_T::ConstSharedPtr& ipCalibMsg);

    /**
     * @overload
     * @brief The onInit method is called by node manager. It is responsible for initializing the
     * node.
     *
     * @note It is important that this method returns, otherwise the node gets stuck during the
     * initialization
     */
    void onInit();

  private:
    /**
     * @brief Method to get transform between two tf frames.
     *
     * @param[in] iTargetFrame Id of Target frame.
     * @param[in] iSourceFrame Id of Source frame.
     * @param[out] oTransform Transform from source to target frame.
     * @return True, if successful. False, otherwise.
     */
    bool getTransformBetweenTfFrames(const std::string& iTargetFrame,
                                     const std::string& iSourceFrame,
                                     geometry_msgs::msg::TransformStamped& oTransform) const;

    /**
     * @brief Method to get camera intrinsics from camera info message and store in camera_ object.
     */
    bool initializeCameraData(const std::string& iImgTopic,
                              const EImageState& iImgState,
                              const std::string& iCameraNamespace, lib3d::Camera& oCamera);

    /**
     * @brief Method to initialize subscribers of node
     *
     * @return True, if successful. False, otherwise.
     */
    bool initializePublishers();

    /**
     * @brief Method to initialize subscribers of node
     *
     * @return True, if successful. False, otherwise.
     */
    bool initializeSubscribers();

    /**
     * @brief Method to read launch parameters
     *
     * @return True if successful. False, otherwise (e.g. if sanity check fails)
     */
    bool readLaunchParameters();

    void projectCloudPointIntoDepthMap(const InputPointType& iCloudPnt,
                                       const lib3d::Camera& iCamera,
                                       lib3d::DepthMap& ioDepthMap) const;

    /**
     * @brief Method to split string by delimiter into list of floats.
     * If a substring is not a number it will not be added to the output list
     *
     * @param[in] iStr String to split.
     * @param[in] iDelimiter Delimiter at which to split.
     * @return List of floats.
     */
    std::vector<float> splitStringToFloat(const std::string& iStr, const char& iDelimiter) const;

    //--- MEMBER DECLERATION ---//

  private:
    /// name of node
    std::string nodeName_;

    /// Flag indicating initialization state of node
    bool isInitialized_;

    /// message filter for approximated message synchronization for vis or nir image and point cloud
    std::shared_ptr<message_filters::Synchronizer<ImgCloudApproxSync>> pImgCloudApproxSync_;

    /// message filter for exact message synchronization for vis or nir image and point cloud
    std::shared_ptr<message_filters::Synchronizer<ImgCloudExactSync>> pImgCloudExactSync_;

    /// subscriber to calibration result
    std::shared_ptr<rclcpp::Subscription<CalibrationResult_Message_T>> pCalibSub_;

    /// Subscriber to vis image topic
    message_filters::Subscriber<sensor_msgs::msg::Image> imageSubsc_;

    /// Subscriber to point cloud
    message_filters::Subscriber<InputCloud_Message_T> cloudSubsc_;

    /// Publisher for fused image
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pPub_;

    /// transform listener to get the transform between camera and point cloud
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    /// State of image which is received.
    EImageState imageState_;

    /// Namespace of the camera, used to compute topic of cameraInfo
    std::string cameraNamespace_;

    /// Camera object holding intrinsics and extrinsic coordinates of the camera
    lib3d::Camera camera_;

    /// Member variable holding minimum depth value to consider when fusing image and point cloud
    float minDepth_;

    /// Member variable holding maximum depth value to consider when fusing image and point cloud
    float maxDepth_;

    /// Flag indicating to use temporary transform between sensors which is passed as launch parameter
    bool useTemporaryTransform_;

    /// Temporary transform between sensors to be used if useTemporaryTransform_ is true
    tf2::Transform temporaryTransform_;

    /// Queue size for synchronization of image messages and point cloud
    int syncQueueSize_;

    /// Flag to activate exact time synchronization
    bool useExactSync_;
};
} // namespace visualizers
} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_POINTCLOUD2IMAGENODE_H
