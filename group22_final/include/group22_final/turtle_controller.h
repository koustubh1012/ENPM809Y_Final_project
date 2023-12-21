#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace std::chrono_literals;

namespace group22_final{
    class TurtleBot3Controller: public rclcpp::Node{
        public:
        using NavigateToWaypoints = nav2_msgs::action::FollowWaypoints;
        using GoalHandleWaypoints = rclcpp_action::ClientGoalHandle<NavigateToWaypoints>;
        TurtleBot3Controller(std::string node_name):
        Node(node_name){
            // declare the paramerters for aruco markers and set default values
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

            // set the parameters from the passed arguments or yaml file
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

            // create subscription for reading aruco markers
            marker_sub_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers",rclcpp::SensorDataQoS(),
            std::bind(&TurtleBot3Controller::marker_cb , this , std::placeholders::_1));

            // sleep for 10 seconds
            RCLCPP_INFO_STREAM(this->get_logger(), "Simulation will start initialising in 10 sec");
            std::this_thread::sleep_for(std::chrono::seconds(10));

            battery_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            battery_transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*battery_tf_buffer_);

            client_ = rclcpp_action::create_client<NavigateToWaypoints>(this, "follow_waypoints");
            // initialize the publisher
            initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>( "initialpose", 10);
            // set the initial pose for navigation
            set_initial_pose();
            RCLCPP_INFO_STREAM(this->get_logger(), "Initialising... sleeping for 10 seconds");
            std::this_thread::sleep_for(std::chrono::seconds(10));
            RCLCPP_INFO_STREAM(this->get_logger(), "Initialised");
        }
        private:
            // create subscription for aruco markers
            rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr marker_sub_;
            // marker callback function
            void marker_cb(ros2_aruco_interfaces::msg::ArucoMarkers msg);
            // structure for defining waypoint
            struct wp{
                std::string type;
                std::string color;
            };
            // structure for defining each marker
            struct marker{
                struct wp wp1;
                struct wp wp2;
                struct wp wp3;
                struct wp wp4;
                struct wp wp5;
            };

            struct marker aruco_zero_;                  // create structure to store parameters from type zero marker
            struct marker aruco_one_;                   // create structure to store parameters from type one markers
            std::vector<std::string> waypoints;         // vector to store the waypoints from aruco markers
            std::vector<std::string> bat_colors_;       // vector to store battery colors detected from camera
            std::vector<float> goal_x;                  // vector to store battery x coordinates from camera
            std::vector<float> goal_y;                  // vector to store battery y coordinates from camera


            std::unique_ptr<tf2_ros::Buffer> battery_tf_buffer_;
            /*!< Transform listener object */
            std::shared_ptr<tf2_ros::TransformListener> battery_transform_listener_{nullptr};

            // function to listen to tranform between map and battery frame
            std::pair<float, float> battery_listen_transform(const std::string &source_frame, const std::string &target_frame);
            //function to call the listen tranform function
            void battery_listen_cb_();

            rclcpp_action::Client<NavigateToWaypoints>::SharedPtr client_;
            rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

            void goal_response_callback(std::shared_future<GoalHandleWaypoints::SharedPtr> future);
            void feedback_callback(GoalHandleWaypoints::SharedPtr, const std::shared_ptr<const NavigateToWaypoints::Feedback> feedback);
            void result_callback(const GoalHandleWaypoints::WrappedResult& result);
            
            // function to send the goal poses to the robot
            void send_goal();                          
            // function to set intial pose of the tobot 
            void set_initial_pose();                   
    };
}