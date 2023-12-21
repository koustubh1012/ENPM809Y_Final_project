#include "battery_tf_broadcaster.h"
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

// callback funtion for camera1 subcription
void group22_final::BatteryTfBroadcaster::camera_1_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg){
    if(!msg.part_poses.empty()){
        geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
        dynamic_transform_stamped.header.stamp = this->get_clock()->now();
        dynamic_transform_stamped.header.frame_id = "camera1_frame";
        dynamic_transform_stamped.child_frame_id = "battery_1_frame";
        dynamic_transform_stamped.transform.translation.x = msg.part_poses[0].pose.position.x;
        dynamic_transform_stamped.transform.translation.y = msg.part_poses[0].pose.position.y;
        dynamic_transform_stamped.transform.translation.z = msg.part_poses[0].pose.position.z;
        dynamic_transform_stamped.transform.rotation.x = msg.part_poses[0].pose.orientation.x;
        dynamic_transform_stamped.transform.rotation.y = msg.part_poses[0].pose.orientation.y;
        dynamic_transform_stamped.transform.rotation.z = msg.part_poses[0].pose.orientation.z;
        dynamic_transform_stamped.transform.rotation.w = msg.part_poses[0].pose.orientation.w;
        tf_broadcaster_battery_->sendTransform(dynamic_transform_stamped);
        // camera_1_subscription_.reset();
    }
}

// callback funtion for camera2 subcription
void group22_final::BatteryTfBroadcaster::camera_2_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg){
    if(!msg.part_poses.empty()){
        geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
        dynamic_transform_stamped.header.stamp = this->get_clock()->now();
        dynamic_transform_stamped.header.frame_id = "camera2_frame";
        dynamic_transform_stamped.child_frame_id = "battery_2_frame";
        dynamic_transform_stamped.transform.translation.x = msg.part_poses[0].pose.position.x;
        dynamic_transform_stamped.transform.translation.y = msg.part_poses[0].pose.position.y;
        dynamic_transform_stamped.transform.translation.z = msg.part_poses[0].pose.position.z;
        dynamic_transform_stamped.transform.rotation.x = msg.part_poses[0].pose.orientation.x;
        dynamic_transform_stamped.transform.rotation.y = msg.part_poses[0].pose.orientation.y;
        dynamic_transform_stamped.transform.rotation.z = msg.part_poses[0].pose.orientation.z;
        dynamic_transform_stamped.transform.rotation.w = msg.part_poses[0].pose.orientation.w;
        tf_broadcaster_battery_->sendTransform(dynamic_transform_stamped);
        // camera_2_subscription_.reset();
    }
}

// callback funtion for camera3 subcription
void group22_final::BatteryTfBroadcaster::camera_3_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg){
    if(!msg.part_poses.empty()){
        geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
        dynamic_transform_stamped.header.stamp = this->get_clock()->now();
        dynamic_transform_stamped.header.frame_id = "camera3_frame";
        dynamic_transform_stamped.child_frame_id = "battery_3_frame";
        dynamic_transform_stamped.transform.translation.x = msg.part_poses[0].pose.position.x;
        dynamic_transform_stamped.transform.translation.y = msg.part_poses[0].pose.position.y;
        dynamic_transform_stamped.transform.translation.z = msg.part_poses[0].pose.position.z;
        dynamic_transform_stamped.transform.rotation.x = msg.part_poses[0].pose.orientation.x;
        dynamic_transform_stamped.transform.rotation.y = msg.part_poses[0].pose.orientation.y;
        dynamic_transform_stamped.transform.rotation.z = msg.part_poses[0].pose.orientation.z;
        dynamic_transform_stamped.transform.rotation.w = msg.part_poses[0].pose.orientation.w;
        tf_broadcaster_battery_->sendTransform(dynamic_transform_stamped);
        // camera_3_subscription_.reset();
    }
}

// callback funtion for camera4 subcription
void group22_final::BatteryTfBroadcaster::camera_4_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg){
    if(!msg.part_poses.empty()){
        geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
        dynamic_transform_stamped.header.stamp = this->get_clock()->now();
        dynamic_transform_stamped.header.frame_id = "camera4_frame";
        dynamic_transform_stamped.child_frame_id = "battery_4_frame";
        dynamic_transform_stamped.transform.translation.x = msg.part_poses[0].pose.position.x;
        dynamic_transform_stamped.transform.translation.y = msg.part_poses[0].pose.position.y;
        dynamic_transform_stamped.transform.translation.z = msg.part_poses[0].pose.position.z;
        dynamic_transform_stamped.transform.rotation.x = msg.part_poses[0].pose.orientation.x;
        dynamic_transform_stamped.transform.rotation.y = msg.part_poses[0].pose.orientation.y;
        dynamic_transform_stamped.transform.rotation.z = msg.part_poses[0].pose.orientation.z;
        dynamic_transform_stamped.transform.rotation.w = msg.part_poses[0].pose.orientation.w;
        tf_broadcaster_battery_->sendTransform(dynamic_transform_stamped);
        // camera_4_subscription_.reset();
    }
}

// callback funtion for camera5 subcription
void group22_final::BatteryTfBroadcaster::camera_5_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg){
    if(!msg.part_poses.empty()){
        geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
        dynamic_transform_stamped.header.stamp = this->get_clock()->now();
        dynamic_transform_stamped.header.frame_id = "camera5_frame";
        dynamic_transform_stamped.child_frame_id = "battery_5_frame";
        dynamic_transform_stamped.transform.translation.x = msg.part_poses[0].pose.position.x;
        dynamic_transform_stamped.transform.translation.y = msg.part_poses[0].pose.position.y;
        dynamic_transform_stamped.transform.translation.z = msg.part_poses[0].pose.position.z;
        dynamic_transform_stamped.transform.rotation.x = msg.part_poses[0].pose.orientation.x;
        dynamic_transform_stamped.transform.rotation.y = msg.part_poses[0].pose.orientation.y;
        dynamic_transform_stamped.transform.rotation.z = msg.part_poses[0].pose.orientation.z;
        dynamic_transform_stamped.transform.rotation.w = msg.part_poses[0].pose.orientation.w;
        tf_broadcaster_battery_->sendTransform(dynamic_transform_stamped);
        // camera_5_subscription_.reset();
    }
}



int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<group22_final::BatteryTfBroadcaster>("battery_tf_broadcaster");
  rclcpp::spin(node);
  rclcpp::shutdown();
}