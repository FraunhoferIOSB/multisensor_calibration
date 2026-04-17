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

#ifndef MULTISENSORCALIBRATION_UI_IMAGEVIEWDIALOG_H
#define MULTISENSORCALIBRATION_UI_IMAGEVIEWDIALOG_H

// std
#include <atomic>
#include <memory>

// Qt
#include <QDialog>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QResizeEvent>
#include <QSize>

// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/rclcpp.hpp>

// multisensor_calibration
#include "../common/common.h"

namespace multisensor_calibration
{

namespace Ui
{
class ViewDialog;
}

/**
 * @ingroup ui
 * @brief Dialog class to show image data.
 *
 */
class ImageViewDialog : public QDialog
{
    Q_OBJECT

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Constructor
     */
    ImageViewDialog(QWidget* parent = nullptr);

    /**
     * @brief Destructor
     *
     */
    ~ImageViewDialog();

    /**
     * @brief Callback function to receive and display ROS image messages.
     *
     * @param[in] ipImgMsg Pointer to vis image message.
     */
    void imageMessageCallback(const InputImage_Message_T::ConstSharedPtr& ipImgMsg);

    /**
     * @brief Assign the image view to subscribe to a given image topic available on the given
     * node.
     *
     * @param[in] ipNode Pointer to node
     * @param[in] iTopicName Topic on which to subscribe.
     */
    void subscribeToImageTopic(rclcpp::Node* ipNode, const std::string& iTopicName);

  protected:
    void resizeEvent(QResizeEvent* event) override;

    //--- MEMBER DECLARATION ---//

  private:
    /// Pointer to UI
    Ui::ViewDialog* ui;

    /// Pointer to graphics view in which the image is displayed.
    std::shared_ptr<QGraphicsView> pImageGraphicsView_;

    /// Pointer to the graphics scene.
    std::shared_ptr<QGraphicsScene> pImageGraphicsScene_;

    /// Pointer to the PixMap item which holds the image to diplay.
    QGraphicsPixmapItem* pPixmapItem_;

    /// Subscriber to the image topic.
    image_transport::Subscriber imageSubsc_;

    /// Guards against queuing multiple pending frame updates on the GUI thread.
    std::atomic<bool> pendingFrameUpdate_{false};

    /// Last displayed image size; used to avoid redundant fitInView() calls.
    QSize lastImageSize_;
};

} // namespace multisensor_calibration
#endif // MULTISENSORCALIBRATION_UI_IMAGEVIEWDIALOG_H
