#include "turtle_controller.h"
#include <rclcpp/rclcpp.hpp>

void group22_final::TurtleBot3Controller::marker_cb(ros2_aruco_interfaces::msg::ArucoMarkers msg){
    // RCLCPP_INFO_STREAM_ONCE(this->get_logger(),"Marker: " << msg.marker_ids[0]);
    if(msg.marker_ids[0] == 0){
        waypoints.push_back(aruco_zero_.wp1.color);
        waypoints.push_back(aruco_zero_.wp2.color);
        waypoints.push_back(aruco_zero_.wp3.color);
        waypoints.push_back(aruco_zero_.wp4.color);
        waypoints.push_back(aruco_zero_.wp5.color);
    }
    else if(msg.marker_ids[0] == 1){
        waypoints.push_back(aruco_one_.wp1.color);
        waypoints.push_back(aruco_one_.wp2.color);
        waypoints.push_back(aruco_one_.wp3.color);
        waypoints.push_back(aruco_one_.wp4.color);
        waypoints.push_back(aruco_one_.wp5.color);
    }
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Waypoint 1: " << waypoints[0]);
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Waypoint 2: " << waypoints[1]);
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Waypoint 3: " << waypoints[2]);
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Waypoint 4: " << waypoints[3]);
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Waypoint 5: " << waypoints[4]);
}


// function to listen to aruco tranform
std::pair<float, float> group22_final::TurtleBot3Controller::aruco_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;
    try
    {
        t_stamped = aruco_tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 10ms);
    }
    catch (const tf2::TransformException &ex)
    {
        // RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        // return;
    }
    pose_out.position.x = t_stamped.transform.translation.x;
    pose_out.position.y = t_stamped.transform.translation.y;
    pose_out.orientation = t_stamped.transform.rotation;
    // RCLCPP_INFO_STREAM(this->get_logger(), "X: " << pose_out.position.x);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Y: " << pose_out.position.y);
    return std::make_pair(pose_out.position.x, pose_out.position.y);
}


// timer callback function for aruco marker detection
void group22_final::TurtleBot3Controller::aruco_listen_timer_cb_()
{
    auto temp1 = aruco_listen_transform("world", "battery_1_frame");
    blue.x = temp1.first;
    blue.y = temp1.second;
    RCLCPP_INFO_STREAM(this->get_logger(), "Blue X and Y: " << blue.x <<" "<<blue.y);
    auto temp2 = aruco_listen_transform("world", "battery_2_frame");
    orange.x = temp2.first;
    orange.y = temp2.second;
    RCLCPP_INFO_STREAM(this->get_logger(), "Orange X and Y: " << orange.x <<" "<<orange.y);
    auto temp3 = aruco_listen_transform("world", "battery_3_frame");
    purple.x = temp3.first;
    purple.y = temp3.second;
    RCLCPP_INFO_STREAM(this->get_logger(), "Purple X and Y: " << purple.x <<" "<<purple.y);
    auto temp4 = aruco_listen_transform("world", "battery_4_frame");
    green.x = temp4.first;
    green.y = temp4.second;
    RCLCPP_INFO_STREAM(this->get_logger(), "Green X and Y: " << green.x <<" "<<green.y);
    auto temp5 = aruco_listen_transform("world", "battery_5_frame");
    red.x = temp5.first;
    red.y = temp5.second;
    RCLCPP_INFO_STREAM(this->get_logger(), "Red X and Y: " << red.x <<" "<<red.y);
}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<group22_final::TurtleBot3Controller>("turtle_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}