/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/GuiBase.h"

// Qt
#include <QCoreApplication>

namespace multisensor_calibration
{

//==================================================================================================
GuiBase::GuiBase() :
  QObject(nullptr)
{
}

//==================================================================================================
GuiBase::GuiBase(const std::string& iAppTitle,
                 const std::string& iGuiSubNamespace) :
  QObject(nullptr),
  appTitle_(iAppTitle),
  guiNodeName_(iAppTitle + "_" + iGuiSubNamespace),
  isInitialized_(true),
  pNode_(nullptr),
  pExecutor_(nullptr)
{
    //--- initialize spin timer
    spinTimer_.setInterval(100);
    spinTimer_.setSingleShot(false);
    connect(&spinTimer_, &QTimer::timeout, this, &GuiBase::spinOnce);
}

//==================================================================================================
GuiBase::~GuiBase()
{
}

//==================================================================================================
std::string GuiBase::getGuiNodeName() const
{
    return guiNodeName_;
}

//==================================================================================================
rclcpp::Node* GuiBase::nodePtr() const
{
    return pNode_.get();
}

//==================================================================================================
rclcpp::Node::SharedPtr GuiBase::nodeSharedPtr() const
{
    return pNode_;
}

//==================================================================================================
rclcpp::Executor::SharedPtr GuiBase::executor() const
{
    return pExecutor_;
}

//==================================================================================================
bool GuiBase::init(const std::shared_ptr<rclcpp::Executor>& ipExec,
                   const rclcpp::NodeOptions& iNodeOpts)
{
    if (ipExec == nullptr)
        return false;

    //--- copy executor
    pExecutor_ = ipExec;

    //--- initialize ros node
    pNode_ = std::make_shared<rclcpp::Node>(guiNodeName_, iNodeOpts);
    if (pNode_ == nullptr)
        return false;

    pExecutor_->add_node(pNode_);
    spinTimer_.start();

    return true;
}

//==================================================================================================
void GuiBase::spinOnce()
{
    if (!pExecutor_->is_spinning())
        pExecutor_->spin_some(std::chrono::milliseconds(60));

    if (!rclcpp::ok())
    {
        emit rosLoopTerminated();
        QCoreApplication::instance()->quit();
    }
}

} // namespace multisensor_calibration