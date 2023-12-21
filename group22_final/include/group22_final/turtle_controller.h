/**
 * @file turtle_controller.h
 * @author FNU Koustubh
 * @brief 
 * @version 0.1
 * @date 2023-12-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */
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

/**
 * @brief   This class is used to control the turtlebot3
 * @param   marker_sub_ : Subscription to the aruco markers
 * @param   battery_tf_buffer_ : Buffer to store the transform between map and battery frame
 * @param   battery_transform_listener_ : Listener object to listen to the transform between map and battery frame
 * @param   client_ : Client object to send the goal poses to the robot
 * @param   initial_pose_pub_ : Publisher object to set the initial pose of the robot
 * @param   aruco_zero_ : Structure to store the parameters from type zero marker
 * @param   aruco_one_ : Structure to store the parameters from type one markers
 * @param   waypoints : Vector to store the waypoints from aruco markers
 * @param   bat_colors_ : Vector to store battery colors detected from camera
 * @param   goal_x : Vector to store battery x coordinates from camera
 * @param   goal_y : Vector to store battery y coordinates from camera
 * @param   battery_tf_buffer_ : Buffer to store the transform between map and battery frame
 * @param   battery_transform_listener_ : Listener object to listen to the transform between map and battery frame
 * @details This class is used to control the turtlebot3. It subscribes to the aruco markers and stores the waypoints from the markers. It also subscribes to the camera to detect the battery and send the goal poses to the robot. It also sets the initial pose of the robot.
 * 
 *
 * 
 */

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
/**
 * @brief   TF Listener for the battery
 * @param   battery_tf_buffer_ : Buffer to store the transform between map and battery frame
 * @param   battery_transform_listener_ : Listener object to listen to the transform between map and battery frame
 * @param   initail_pose_pub_ : Publisher object to set the initial pose of the robot
 * 
 * 
 */
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

/**
 * @brief   Callback function for the aruco markers
 * @param   msg : Message from the aruco markers
 * @param   waypoints : Vector to store the waypoints from aruco markers
 * @param   bat_colors_ : Vector to store battery colors detected from camera
 * @param  goal_x : Vector to store battery x coordinates from camera
 * @param  goal_y : Vector to store battery y coordinates from camera
 * @param  struct marker wp1 : Structure to store the parameters from type zero marker
 * @param  struct marker wp2 : Structure to store the parameters from type zero marker
 * @param  struct marker wp3 : Structure to store the parameters from type zero marker
 * @param  struct marker wp4 : Structure to store the parameters from type zero marker
 * @param  struct marker wp5 : Structure to store the parameters from type zero marker
 * 
 * 
 */
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

/**
 * @brief   Callback function for the logical camera 
 * @param   msg : Message from the logical camera 
 * @param   tf_broadcaster_battery_ : Broadcaster object
 * @param  battery_tf_buffer_ : Buffer to store the transform between map and battery frame
 * 
 */
            std::unique_ptr<tf2_ros::Buffer> battery_tf_buffer_;
            /*!< Transform listener object */
            std::shared_ptr<tf2_ros::TransformListener> battery_transform_listener_{nullptr};

            // function to listen to tranform between map and battery frame
            std::pair<float, float> battery_listen_transform(const std::string &source_frame, const std::string &target_frame);
            //function to call the listen tranform function
            void battery_listen_cb_();
/**
 * @brief  Callback function for the logical camera 
 * @param goal_response_callback
 * @param feedback_callback
 * @param result_callback
 * @param client_ : Client object to send the goal poses to the robot
 * @param initial_pose_pub_ : Publisher object to set the initial pose of the robot
 * 
 * 
 * 
 */
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