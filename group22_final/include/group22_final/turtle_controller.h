#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"

using namespace std::chrono_literals;

namespace group22_final{
    class TurtleBot3Controller: public rclcpp::Node{
        public:TurtleBot3Controller(std::string node_name):
        Node(node_name){
            this->declare_parameter("aruco_0.wp1.type", "battery");
            this->declare_parameter("aruco_0.wp1.color", "green");
            this->declare_parameter("aruco_0.wp2.type", "battery");
            this->declare_parameter("aruco_0.wp2.color", "red");
            this->declare_parameter("aruco_0.wp3.type", "battery");
            this->declare_parameter("aruco_0.wp3.color", "orange");
            this->declare_parameter("aruco_0.wp4.type", "battery");
            this->declare_parameter("aruco_0.wp4.color", "purple");
            this->declare_parameter("aruco_0.wp5.type", "battery");
            this->declare_parameter("aruco_0.wp5.color", "blue");
            this->declare_parameter("aruco_1.wp1.type", "battery");
            this->declare_parameter("aruco_1.wp1.color", "blue");
            this->declare_parameter("aruco_1.wp2.type", "battery");
            this->declare_parameter("aruco_1.wp2.color", "green");
            this->declare_parameter("aruco_1.wp3.type", "battery");
            this->declare_parameter("aruco_1.wp3.color", "orange");
            this->declare_parameter("aruco_1.wp4.type", "battery");
            this->declare_parameter("aruco_1.wp4.color", "red");
            this->declare_parameter("aruco_1.wp5.type", "battery");
            this->declare_parameter("aruco_1.wp5.color", "purple");

            aruco_zero_.wp1.type = this->get_parameter("aruco_0.wp1.type").as_string();
            aruco_zero_.wp1.color = this->get_parameter("aruco_0.wp1.color").as_string();
            aruco_zero_.wp2.type = this->get_parameter("aruco_0.wp2.type").as_string();
            aruco_zero_.wp2.color = this->get_parameter("aruco_0.wp2.color").as_string();
            aruco_zero_.wp3.type = this->get_parameter("aruco_0.wp3.type").as_string();
            aruco_zero_.wp3.color = this->get_parameter("aruco_0.wp3.color").as_string();
            aruco_zero_.wp4.type = this->get_parameter("aruco_0.wp4.type").as_string();
            aruco_zero_.wp4.color = this->get_parameter("aruco_0.wp4.color").as_string();
            aruco_zero_.wp5.type = this->get_parameter("aruco_0.wp5.type").as_string();
            aruco_zero_.wp5.color = this->get_parameter("aruco_0.wp5.color").as_string();

            aruco_one_.wp1.type = this->get_parameter("aruco_1.wp1.type").as_string();
            aruco_one_.wp1.color = this->get_parameter("aruco_1.wp1.color").as_string();
            aruco_one_.wp2.type = this->get_parameter("aruco_1.wp2.type").as_string();
            aruco_one_.wp2.color = this->get_parameter("aruco_1.wp2.color").as_string();
            aruco_one_.wp3.type = this->get_parameter("aruco_1.wp3.type").as_string();
            aruco_one_.wp3.color = this->get_parameter("aruco_1.wp3.color").as_string();
            aruco_one_.wp4.type = this->get_parameter("aruco_1.wp4.type").as_string();
            aruco_one_.wp4.color = this->get_parameter("aruco_1.wp4.color").as_string();
            aruco_one_.wp5.type = this->get_parameter("aruco_1.wp5.type").as_string();
            aruco_one_.wp5.color = this->get_parameter("aruco_1.wp5.color").as_string();

            RCLCPP_INFO_STREAM_ONCE(this->get_logger(),"type: " << aruco_one_.wp1.type);

            marker_sub_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers",rclcpp::SensorDataQoS(),
            std::bind(&TurtleBot3Controller::marker_cb , this , std::placeholders::_1));

            aruco_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            aruco_transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*aruco_tf_buffer_);
            aruco_listen_timer_ = this->create_wall_timer(50ms, std::bind(&TurtleBot3Controller::aruco_listen_timer_cb_, this));

        }
        private:
            rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr marker_sub_;
            void marker_cb(ros2_aruco_interfaces::msg::ArucoMarkers msg);
            struct wp{
                std::string type;
                std::string color;
            };
            struct marker{
                struct wp wp1;
                struct wp wp2;
                struct wp wp3;
                struct wp wp4;
                struct wp wp5;
            };
            struct marker aruco_zero_;
            struct marker aruco_one_;
            std::vector<std::string> waypoints;


            std::unique_ptr<tf2_ros::Buffer> aruco_tf_buffer_;
            /*!< Transform listener object */
            std::shared_ptr<tf2_ros::TransformListener> aruco_transform_listener_{nullptr};
            /*!< Wall timer object */
            rclcpp::TimerBase::SharedPtr aruco_listen_timer_;
            void aruco_listen_transform(const std::string &source_frame, const std::string &target_frame);
            void aruco_listen_timer_cb_();
    };
}