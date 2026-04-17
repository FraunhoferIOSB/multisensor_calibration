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
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @brief Standalone node that accumulates N point cloud messages and publishes
 *        the concatenated result.
 *
 * Parameters:
 *   input_topic   (string) : Topic to subscribe to. Output is published on
 *                            <input_topic>_accumulated.
 *   num_messages  (int)    : Number of messages to accumulate before publishing.
 *                            Default: 10.
 *
 * The sensor is assumed to be stationary during accumulation (no motion
 * compensation). The published cloud uses the timestamp of the last received
 * message.
 */

// ROS
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#if !defined(TARGET_NAME)
#define TARGET_NAME "accumulate_point_cloud"
#endif

class AccumulatePointCloud : public rclcpp::Node
{
public:
    AccumulatePointCloud() :
      Node(TARGET_NAME)
    {
        declare_parameter<std::string>("input_topic", "");
        declare_parameter<int>("num_messages", 10);

        inputTopic_  = get_parameter("input_topic").as_string();
        numMessages_ = get_parameter("num_messages").as_int();

        if (inputTopic_.empty())
        {
            RCLCPP_ERROR(get_logger(),
                         "Parameter 'input_topic' must be set. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        if (numMessages_ < 1)
        {
            RCLCPP_WARN(get_logger(),
                        "num_messages must be >= 1, clamping to 1.");
            numMessages_ = 1;
        }

        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          inputTopic_,
          rclcpp::SensorDataQoS(),
          std::bind(&AccumulatePointCloud::onCloud, this, std::placeholders::_1));

        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          inputTopic_ + "_accumulated",
          rclcpp::QoS(10));

        RCLCPP_INFO(get_logger(),
                    "Accumulating %d messages from '%s', publishing on '%s'.",
                    numMessages_,
                    inputTopic_.c_str(),
                    (inputTopic_ + "_accumulated").c_str());
    }

private:
    void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI> incoming;
        pcl::fromROSMsg(*msg, incoming);
        accumulated_ += incoming;
        lastHeader_   = msg->header;
        ++count_;

        if (count_ >= numMessages_)
        {
            sensor_msgs::msg::PointCloud2 outMsg;
            pcl::toROSMsg(accumulated_, outMsg);
            outMsg.header = lastHeader_;
            publisher_->publish(outMsg);

            RCLCPP_DEBUG(get_logger(),
                         "Published accumulated cloud (%zu points from %d messages).",
                         accumulated_.size(), numMessages_);

            accumulated_.clear();
            count_ = 0;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    publisher_;

    std::string                      inputTopic_;
    int                              numMessages_{10};
    int                              count_{0};
    pcl::PointCloud<pcl::PointXYZI>  accumulated_;
    std_msgs::msg::Header            lastHeader_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AccumulatePointCloud>());
    rclcpp::shutdown();
    return 0;
}
