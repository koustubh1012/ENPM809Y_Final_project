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


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<group22_final::TurtleBot3Controller>("turtle_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}