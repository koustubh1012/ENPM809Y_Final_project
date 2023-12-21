#pragma once
#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"

using namespace std::chrono_literals;

namespace group22_final{
    class BatteryTfBroadcaster : public rclcpp::Node
    {
        public:
        BatteryTfBroadcaster(std::string node_name) : Node(node_name)
        {
            // parameter to decide whether to execute the broadcaster or not
            RCLCPP_INFO(this->get_logger(), "Battery Broadcaster Started");
            camera_1_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera1/image",rclcpp::SensorDataQoS(),
            std::bind(&BatteryTfBroadcaster::camera_1_sub_cb , this , std::placeholders::_1));

            camera_2_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera2/image",rclcpp::SensorDataQoS(),
            std::bind(&BatteryTfBroadcaster::camera_2_sub_cb , this , std::placeholders::_1));

            camera_3_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera3/image",rclcpp::SensorDataQoS(),
            std::bind(&BatteryTfBroadcaster::camera_3_sub_cb , this , std::placeholders::_1));

            camera_4_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera4/image",rclcpp::SensorDataQoS(),
            std::bind(&BatteryTfBroadcaster::camera_4_sub_cb , this , std::placeholders::_1));

            camera_5_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera5/image",rclcpp::SensorDataQoS(),
            std::bind(&BatteryTfBroadcaster::camera_5_sub_cb , this , std::placeholders::_1));

            tf_broadcaster_battery_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        }

        private:
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera_1_subscription_;
        void camera_1_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg);
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera_2_subscription_;
        void camera_2_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg);
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera_3_subscription_;
        void camera_3_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg);
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera_4_subscription_;
        void camera_4_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg);
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera_5_subscription_;
        void camera_5_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg);

        // /*!< Broadcaster object */
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_battery_;
        // rclcpp::TimerBase::SharedPtr tf_timer_battery_;

    };
}