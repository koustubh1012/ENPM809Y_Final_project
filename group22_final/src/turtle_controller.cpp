#include "turtle_controller.h"
#include <rclcpp/rclcpp.hpp>

void group22_final::TurtleBot3Controller::marker_cb(ros2_aruco_interfaces::msg::ArucoMarkers msg){
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(),"Marker: " << msg.marker_ids[0]);
}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<group22_final::TurtleBot3Controller>("turtle_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}