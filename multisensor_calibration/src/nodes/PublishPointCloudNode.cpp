/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

// Std
#include <filesystem>

// ROS
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#if !defined(TARGET_NAME)
#define TARGET_NAME ""
#endif

using namespace pcl::io;

const std::string DEFAULT_TOPIC_NAME = "cloud";
const std::string DEFAULT_FRAME_ID   = "map";

/**
 * @brief Entry point for the 'point_cloud_publisher' node.
 *
 * This will load a point cloud from a specified PLY file and publish it on the
 * given topic with the given frame_id.
 */
class PointCloudPublisher : public rclcpp::Node
{
  public:
    PointCloudPublisher() :
      Node(TARGET_NAME)
    {
        //--- Setup launch parameters
        declare_parameter<std::string>("point_cloud_file", "");
        declare_parameter<std::string>("topic_name", DEFAULT_TOPIC_NAME);
        declare_parameter<std::string>("frame_id", DEFAULT_FRAME_ID);

        //--- Get the launch parameters
        std::string pointcloudFileStr = get_parameter("point_cloud_file").as_string();
        std::string topicName         = get_parameter("topic_name").as_string();
        std::string frameId           = get_parameter("frame_id").as_string();

        //--- sanity check for parameters
        std::filesystem::path pointcloudFilePath(pointcloudFileStr);
        if (pointcloudFilePath.is_relative())
        {
            pointcloudFilePath = std::filesystem::current_path();
            pointcloudFilePath /= pointcloudFileStr;
        }

        if (pointcloudFilePath.extension() != ".ply")
        {
            RCLCPP_ERROR(this->get_logger(), "'point_cloud_file' does not end with '.ply'.");
            isInitialized_ = false;
            return;
        }
        if (!std::filesystem::exists(pointcloudFilePath))
        {
            RCLCPP_ERROR(this->get_logger(), "'point_cloud_file' does not exits. File path: %s",
                         pointcloudFilePath.c_str());
            isInitialized_ = false;
            return;
        }
        if (topicName.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "'topic_name' is empty. Setting to default: %s",
                         DEFAULT_TOPIC_NAME.c_str());
            isInitialized_ = false;
            return;
        }
        if (frameId.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "'frame_id' is empty. Setting to default: %s",
                         DEFAULT_FRAME_ID.c_str());
            isInitialized_ = false;
            return;
        }

        //--- Publisher for the Point Cloud
        pPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topicName, 10);

        //--- Load the pointcloud from the .ply file
        pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud =
          std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        loadPLYFile<pcl::PointXYZ>(pointcloudFilePath, *pCloud);

        pcl::toROSMsg(*pCloud, cloudMsg_);
        cloudMsg_.header.frame_id = frameId;

        //-- create timer with publish rate of 5Hz
        using namespace std::chrono_literals;
        pTimer_ = this->create_wall_timer(200ms,
                                          std::bind(&PointCloudPublisher::timer_callback, this));

        isInitialized_ = true;
    }

    bool isInitialized() const
    {
        return isInitialized_;
    }

  private:
    void timer_callback()
    {
        cloudMsg_.header.stamp = this->get_clock()->now();
        pPublisher_->publish(cloudMsg_);
    }

    bool isInitialized_;

    sensor_msgs::msg::PointCloud2 cloudMsg_;

    rclcpp::TimerBase::SharedPtr pTimer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pPublisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudPublisher>();
    if (!node->isInitialized())
        return 1;

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}