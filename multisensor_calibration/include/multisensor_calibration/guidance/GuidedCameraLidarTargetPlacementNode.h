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

#ifndef MULTISENSORCALIBRATION_GUIDEDCAMERALIDARTARGETPLACEMENTNODELT_H
#define MULTISENSORCALIBRATION_GUIDEDCAMERALIDARTARGETPLACEMENTNODELT_H

// ROS
#include "sensor_msgs/msg/image.hpp"
#include <rclcpp/rclcpp.hpp>

// OpenCV
#include <opencv2/core.hpp>

// multisensor_calibration
#include "GuidanceBase.h"
#include "multisensor_calibration/common/common.h"
#include <multisensor_calibration/common/lib3D/core/intrinsics.hpp>
#include <multisensor_calibration_interface/srv/camera_intrinsics.hpp>
namespace multisensor_calibration
{

/**
 * @ingroup nodes
 * @ingroup guidance
 * @brief Node for guided target placement in the context of extrinsic camera-lidar calibration.
 *
 * @note The guidance feature of multisensor_calibration is currently still in development and not yet
 * functional.
 *
 */
class GuidedCameraLidarTargetPlacementNode : public GuidanceBase, public rclcpp::Node
{

    //--- METHOD DECLARATION ---//
  public:
    /**
     * @brief Default constructor.
     */
    GuidedCameraLidarTargetPlacementNode(std::string iNodeName, rclcpp::NodeOptions iOptions);

    /**
     * @brief Component constructor. The node name is taken from the node options, the
     * calibrator node name from the 'calibrator_node_name' parameter (default: node name of
     * the corresponding calibration type).
     */
    explicit GuidedCameraLidarTargetPlacementNode(const rclcpp::NodeOptions& iOptions);

    /**
     * @brief Destructor
     */
    virtual ~GuidedCameraLidarTargetPlacementNode();

    /**
     * @overload
     * @brief The onInit method is called by node manager. It is responsible for initializing the
     * node.
     *
     * @note It is important that this method returns, otherwise the node gets stuck during the
     * initialization
     */
    void init();

  private:
    typedef multisensor_calibration_interface::srv::CameraIntrinsics CameraIntrinsicsSrv;
    /**
     * @overload
     * @brief Method to compute overlapping Fov between the two sensors based on the estimated extrinsic
     * pose.
     *
     * @note This is a pure virtual method and needs to be implemented by the specific subclasses.
     */
    void computeExtrinsicFovBoundingPlanes() override;

    /**
     * @overload
     * @brief Method to compute Fov of the two sensors based on the intrinsic parameters.
     *
     * @note This is a pure virtual method and needs to be implemented by the specific subclasses.
     */
    bool computeIntrinsicFovBoundingPlanes() override;

    /**
     * @overload
     * @brief Method to compute next target pose.
     *
     * @note This is a pure virtual method and needs to be implemented by the specific subclasses.
     */
    void computeNextTargetPose() override;

    /**
     * @brief Function to draw outline parameterized by given pixels onto given image. The outline will
     * be closed, thus the last pixel will be connected to the first.
     *
     * @param[in, out] ioImage Reference of image onto which the target is to be drawn.
     * @param[in] iPixels List of pixels describing the outline which is to be drawn.
     * @param[in] iColor Color with which to draw the outline.
     * @param[in] iThickness Thickness with which to draw the outline.
     */
    void drawOutlineOntoImage(cv::Mat& ioImage, const std::vector<cv::Point2i>& iPixels,
                              const cv::Scalar& iColor, const int& iThickness) const;

    /**
     * @brief Method to draw next pose (GuidanceBase::nextTargetPose_) of calibration target
     * (GuidanceBase::calibrationTarget_) onto image.
     *
     * @param[in, out] ioImage Reference of image onto which the target is to be drawn.
     */
    void drawNextTargetPoseOntoImage(cv::Mat& ioImage);

    /**
     * @brief Method to draw text onto image.
     *
     * @param[in] iText Text to be drawn onto image.
     * @param[in, out] ioImage Reference of image onto which the target is to be drawn.
     */
    void drawTextOntoImage(const std::string& iText, cv::Mat& ioImage) const;

    /**
     * @brief Method to get camera intrinsics from the calibrator. This is connected to the camIntrinsicsTimer_.
     */
    void getCameraIntrinsics();

    /**
     * @overload
     * @brief Method to initialize publishers
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    bool initializePublishers() override;

    /**
     * @brief Method to initialize subscribers.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool initializeSubscribers() override;

    /**
     * @brief Method to initialize timers.
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    bool initializeTimers() override;

    /**
     * @brief Callback function handling image messages.
     *
     * @param[in] ipImgMsg image message.
     */
    void onImageReceived(const InputImage_Message_T::ConstSharedPtr& ipImgMsg);

    /**
     * @brief Method to project outer border frame to image.
     *
     * @param[in] iBoardSize Size of calibration board.
     * @return List of image pixels.
     */
    std::vector<cv::Point2i> projectTargetBorderFrameToImage(const cv::Size2f& iBoardSize) const;

    /**
     * @brief Method to project circular cutout to image.
     *
     * @param[in] iCoefficients Cutout coefficients.
     * @return List of image pixels.
     */
    std::vector<cv::Point2i> projectCircularCutoutToImage(const std::vector<float>& iCoefficients) const;

    /**
     * @overload
     * @brief Method to reset nextTargetPose_
     */
    void resetNextTargetPose() override;

    //--- MEMBER DECLARATION ---//

  protected:
    /// Node handle
    using GuidanceBase::pNode_;

    /// Timer object to trigger service call to get intrinsic camera data.
    rclcpp::TimerBase::SharedPtr pCamIntrinsicsTimer_;

    /// Subscriber to image topic
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pCameraImageSubsc_;

    /// Publisher for image annotated with target position
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pGuidanceImgPub_;

    /// Object of camera intrinsics;
    lib3d::Intrinsics cameraIntrinsics_;

    /// Service client for the camera intrinsics
    rclcpp::Client<CameraIntrinsicsSrv>::SharedPtr intrinsicsClient_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_GUIDEDCAMERALIDARTARGETPLACEMENTNODELT_H