/**
 * @file battery_tf_broadcaster.h
 * @author FNU Koustubh
 * @brief 
 * @version 0.1
 * @date 2023-12-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"

/**
 * @brief  This class is used to broadcast the transform of the battery from the logical camera to the world frame
 * @param  camera_1_subscription_ : Subscription to the logical camera 1    
 * @param  camera_2_subscription_ : Subscription to the logical camera 2
 * @param  camera_3_subscription_ : Subscription to the logical camera 3
 * @param  camera_4_subscription_ : Subscription to the logical camera 4
 * @param  camera_5_subscription_ : Subscription to the logical camera 5
 * 
 * 
 */
using namespace std::chrono_literals;

namespace group22_final{
    class BatteryTfBroadcaster : public rclcpp::Node
    {
        public:
        BatteryTfBroadcaster(std::string node_name) : Node(node_name)
        {
            RCLCPP_INFO(this->get_logger(), "Battery Broadcaster Started");

            // create subscription for camera1
            camera_1_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera1/image",rclcpp::SensorDataQoS(),
            std::bind(&BatteryTfBroadcaster::camera_1_sub_cb , this , std::placeholders::_1));

            // create subscription for camera2
            camera_2_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera2/image",rclcpp::SensorDataQoS(),
            std::bind(&BatteryTfBroadcaster::camera_2_sub_cb , this , std::placeholders::_1));

            // create subscription for camera3
            camera_3_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera3/image",rclcpp::SensorDataQoS(),
            std::bind(&BatteryTfBroadcaster::camera_3_sub_cb , this , std::placeholders::_1));

            // create subscription for camera4
            camera_4_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera4/image",rclcpp::SensorDataQoS(),
            std::bind(&BatteryTfBroadcaster::camera_4_sub_cb , this , std::placeholders::_1));

            // create subscription for camera5
            camera_5_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera5/image",rclcpp::SensorDataQoS(),
            std::bind(&BatteryTfBroadcaster::camera_5_sub_cb , this , std::placeholders::_1));
/**
 * @brief  TF Broadcaster for the battery
 * @param  tf_broadcaster_battery_ : Broadcaster object
 * 
 * 
 */
            tf_broadcaster_battery_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        }
/**
 * @brief   Callback function for the logical camera 1
 * @param   msg : Message from the logical camera 1
 * @param   tf_broadcaster_battery_ : Broadcaster object
 * @param  transformStamped : Transform object
 * @param  transformStamped.header.frame_id : Frame id of the battery
 * @param  transformStamped.child_frame_id : Frame id of the map
 * @param  camera_1_subscription_ : Subscription to the logical camera 1
 * @param  camera_1_sub_cb : Callback function for the logical camera 1
 * @param camera_2_subscription_ : Subscription to the logical camera 2
 * @param  camera_2_sub_cb : Callback function for the logical camera 2
 * @param camera_3_subscription_ : Subscription to the logical camera 3
 * @param  camera_3_sub_cb : Callback function for the logical camera 3
 * @param camera_4_subscription_ : Subscription to the logical camera 4
 * @param  camera_4_sub_cb : Callback function for the logical camera 4
 * @param camera_5_subscription_ : Subscription to the logical camera 5
 * @param  camera_5_sub_cb : Callback function for the logical camera 5
 *  
 */

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

    };// class BatteryTfBroadcaster
} // namespace group22_final