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

#ifndef MULTISENSORCALIBRATION_UI_RVIZ3DVIEWDIALOG_H
#define MULTISENSORCALIBRATION_UI_RVIZ3DVIEWDIALOG_H

// std
#include <memory>
#include <vector>

// Qt
#include <QDialog>
#include <QWidget>

// Rviz
#include "rviz_common/visualization_manager.hpp"
#include <rviz_common/display.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_frame.hpp>

namespace multisensor_calibration
{

namespace Ui
{
class ViewDialog;
}

/**
 * @ingroup ui
 * @brief Dialog class to show 3D data (e.g. point clouds) inside an RViz dialog.

 * @see https://docs.ros.org/en/hydro/api/librviz_tutorial/html/
 * @see https://github.com/ros-visualization/visualization_tutorials/blob/hydro-devel/librviz_tutorial/src/myviz.cpp
 */
class Rviz3dViewDialog : public QDialog
{
    Q_OBJECT

    //--- ENUM DECLERATION ---//

  public:
    /**
     * @brief Enumeration holding possible views in 3D view.
     *
     */
    enum EViews
    {
        ORBIT = 0,
        TOP_DOWN
    };

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Constructor
     *
     * @param parent
     */
    Rviz3dViewDialog(QWidget* parent = nullptr, std::string iNodeAbstractionName = "rviz3dViewNodeAbs");

    /**
     * @brief Destructor
     *
     */
    ~Rviz3dViewDialog();

    /**
     * @brief Add coordinate axes frame for given reference frame.
     *
     * @param[in] iReferenceFrame Name of the reference frame. If left empty, it will be placed
     * at the center of the current view.
     * @return True, if successful. False, otherwise.
     */
    bool addAxes(const std::string& iReferenceFrame = "");

    /**
     * @brief Add box object indicating where to place the calibration target. The position of the
     * box within the 3D view is always updated via the given topic.
     *
     * @param[in] iTopicName Topic name on which to receive position updates.
     * @return True, if successful. False, otherwise.
     */
    bool addGuidedPlacementBox(const std::string& iTopicName);

    /**
     * @brief Add cloud holding the marker corners to the 3D view. The marker corners will be
     * published via the given topic.
     *
     * @param[in] iTopicName Topic name on which the marker corners will be published.
     * @return True, if successful. False, otherwise.
     */
    bool addMarkerCornersCloud(const std::string& iTopicName);

    /**
     * @brief Add raw point cloud from a LiDAR sensor to the 3D view.
     *
     * @param[in] iTopicName Topic name on which the point cloud will be published.
     * @return True, if successful. False, otherwise.
     */
    bool addRawSensorCloud(const std::string& iTopicName);

    /**
     * @brief Add cloud holding the region of interest, i.e. the cluster candidates, to the 3D view.
     *
     * @param[in] iTopicName Topic name on which the cloud will be published.
     * @return True, if successful. False, otherwise.
     */
    bool addRegionsOfInterestCloud(const std::string& iTopicName);

    /**
     * @brief Add cloud representing the point-wise distance to the 3D view.
     *
     * @param[in] iTopicName Topic name on which the cloud will be published.
     * @return True, if successful. False, otherwise.
     */
    bool addPointWiseDistanceCloud(const std::string& iTopicName);

    /**
     * @brief Add cloud holding the detected calibration target to the 3D view.
     *
     * @param[in] iTopicName Topic name on which the target cloud will be published.
     * @return True, if successful. False, otherwise.
     */
    bool addCalibTargetCloud(const std::string& iTopicName);

    /** @overload
     */
    void closeEvent(QCloseEvent* closeEvent) override;

    /**
     * @brief Set fixed reference frame with given frame id.
     *
     * @param[in] iFrameId Frame ID of the reference frame.
     * @return True, if successful. False, otherwise.
     */
    bool setFixedReferenceFrame(const std::string& iFrameId);

    /**
     * @brief Set view of the 3D display
     *
     * @return True, if successful. False, otherwise.
     */
    bool setView(const EViews& iView);

    /** @overload
     */
    void showEvent(QShowEvent* showEvent) override;

  private:
    /**
     * @brief Initialize panel on which the 3D view is rendered.
     *
     */
    void initRenderPanel();

    //--- MEMBER DECLARATION ---//

  private:
    /// Pointer to UI
    Ui::ViewDialog* pUi_;

    /// Flag indicating if initialization was successful.
    bool isInitialized_;

    /// Pointer to render panel in which the view is to be rendered.
    std::shared_ptr<rviz_common::RenderPanel> pRenderPanel_;

    /// Pointer to visualization manager and frame.
    std::shared_ptr<rviz_common::VisualizationManager> pVisManager_;

    /// Pointer to RosNodeAbstraction, needed in ROS 2
    std::string nodeAbsName_;
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> pRosNodeAbs_;

    /// Pointer to a WindowManagerInterface, needed in ROS 2
    rviz_common::WindowManagerInterface* pWindowManager_;

    /// Frame ID of the fixed reference frame.
    std::string fixedReferenceFrame_;

    /// List of frame IDs for which axes are to be displayed.
    std::vector<std::string> axisReferenceFrames_;

    /// List of topic names on which the pose of the placement box is ar published.
    std::vector<std::string> placementBoxTopicNames_;

    /// List of topic names on which the clouds holding the marker corners are published.
    std::vector<std::string> cornerCloudTopicNames_;

    /// List of topic names on which the raw sensor clouds are published.
    std::vector<std::string> sensorCloudTopicNames_;

    /// List of topic names on which the clouds holding regions of interest, i.e. the cluster
    /// candidates are published.
    std::vector<std::string> roisCloudTopicNames_;

    /// List of topic names on which the clouds holding point-wise distance are published.
    std::vector<std::string> distanceCloudTopicNames_;

    /// List of topic names on which the clouds holding the detected calibration targets are
    /// published.
    std::vector<std::string> targetCloudTopicNames_;
};

} // namespace multisensor_calibration
#endif // MULTISENSORCALIBRATION_UI_RVIZ3DVIEWDIALOG_H
