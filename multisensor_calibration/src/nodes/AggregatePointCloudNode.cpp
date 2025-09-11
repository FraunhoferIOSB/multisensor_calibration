/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

// ROS
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#if !defined(TARGET_NAME)
#define TARGET_NAME ""
#endif

using namespace pcl::io;

class AggregatePointCloud : public rclcpp::Node
{
  public:
    AggregatePointCloud() :
      Node(TARGET_NAME)
    {
        declare_parameter<std::string>("cloud_topic_name", "");
        declare_parameter<int>("num_aggregation", 10);

        topicName_      = get_parameter("cloud_topic_name").as_string();
        numAggregation_ = get_parameter("num_aggregation").as_int();

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          topicName_, 1, std::bind(&AggregatePointCloud::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          topicName_ + "_aggregated", 1);
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        if (aggregationItr_ == 0)
        {
            if (aggregatedCloud_)
            {
                prevAggregatedCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::copyPointCloud(*aggregatedCloud_, *prevAggregatedCloud_);
            }
            aggregatedCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr msgCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud_x(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud_xy(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *msgCloud);

        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(msgCloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.5, 6.);
        pass.filter(*filteredCloud_x);
        pass.setInputCloud(filteredCloud_x);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-5.5, 5.5);
        pass.filter(*filteredCloud_xy);
        pass.setInputCloud(filteredCloud_xy);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-2., 4);
        pass.filter(*filteredCloud);

        (*aggregatedCloud_) += *filteredCloud;

        aggregationItr_++;

        if (prevAggregatedCloud_)
        {
            sensor_msgs::msg::PointCloud2 cloudMsg;
            pcl::toROSMsg(*prevAggregatedCloud_, cloudMsg);
            cloudMsg.header = msg->header;
            publisher_->publish(cloudMsg);
        }

        if (aggregationItr_ == numAggregation_)
        {
            aggregationItr_ = 0;
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr aggregatedCloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr prevAggregatedCloud_;

    std::string topicName_;
    int numAggregation_;
    int aggregationItr_ = 0;
};

int main(int argc, char** argv)
{
    //---Initialize ROS node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AggregatePointCloud>());
    rclcpp::shutdown();
    return 0;
}