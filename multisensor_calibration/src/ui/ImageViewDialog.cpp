/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/ImageViewDialog.h"

// Qt
#include <QCloseEvent>
#include <QImage>
#include <QMessageBox>

// ROS
#include <cv_bridge/cv_bridge.hpp>

// multisensor_calibration
#include "ui_ViewDialog.h"
#include <multisensor_calibration/common/lib3D/core/visualization.hpp>

namespace multisensor_calibration
{

//==================================================================================================
ImageViewDialog::ImageViewDialog(QWidget* parent) :
  QDialog(parent),
  ui(new Ui::ViewDialog),
  pImageGraphicsView_(new QGraphicsView),
  pImageGraphicsScene_(new QGraphicsScene),
  pPixmapItem_(nullptr)
{
    ui->setupUi(this);
    ui->verticalLayout->addWidget(pImageGraphicsView_.get());

    pImageGraphicsScene_->setBackgroundBrush(Qt::black);
    pImageGraphicsView_->setScene(pImageGraphicsScene_.get());
}

//==================================================================================================
ImageViewDialog::~ImageViewDialog()
{
    delete ui;
    delete pPixmapItem_;
}

//==================================================================================================
void ImageViewDialog::imageMessageCallback(const InputImage_Message_T::ConstSharedPtr& ipImgMsg)
{
    //--- get image from message
    cv_bridge::CvImageConstPtr pCvBridgeImg;
    try
    {
        pCvBridgeImg = cv_bridge::toCvShare(ipImgMsg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("multisensor_calibration::ImageViewDialog"),
                     "Exception while trying to convert image message."
                     "\n\t> Possible cause: Subscription to raw (non-debayered) image stream."
                     "\n\t> cv_bridge::Exception: %s",
                     e.what());
        return;
    }

    // Object of OpenCV camera image
    cv::Mat cvImage;

    //--- convert to bgr or simply copy to member variable
    if (pCvBridgeImg->encoding == "mono8")
        cv::cvtColor(pCvBridgeImg->image, cvImage, CV_GRAY2RGB);
    else if (pCvBridgeImg->encoding == "bgr8")
        cv::cvtColor(pCvBridgeImg->image, cvImage, CV_BGR2RGB);
    else
        pCvBridgeImg->image.copyTo(cvImage);

    //--- create qimage with shared memory
    if (!pPixmapItem_)
        pPixmapItem_ = pImageGraphicsScene_->addPixmap(
          QPixmap::fromImage(lib3d::cvMat2QImage_shared(cvImage)));
    else
        pPixmapItem_->setPixmap(QPixmap::fromImage(lib3d::cvMat2QImage_shared(cvImage)));
    pImageGraphicsView_->fitInView(pPixmapItem_, Qt::KeepAspectRatio);
}

//==================================================================================================
void ImageViewDialog::subscribeToImageTopic(rclcpp::Node* ipNode,
                                            const std::string& iTopicName)
{
    image_transport::ImageTransport imgTransp(ipNode->shared_from_this());
    imageSubsc_ =
      imgTransp.subscribe(iTopicName, 1,
                          std::bind(&ImageViewDialog::imageMessageCallback, this,
                                    std::placeholders::_1));
}

} // namespace multisensor_calibration
