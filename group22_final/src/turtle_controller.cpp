#include "turtle_controller.h"
#include <rclcpp/rclcpp.hpp>

void group22_final::TurtleBot3Controller::marker_cb(ros2_aruco_interfaces::msg::ArucoMarkers msg){
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(),"Marker: " << msg.marker_ids[0]);
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
    RCLCPP_INFO_STREAM(this->get_logger(),"Inside marker_cb"<<waypoints.size());
    send_goal();
    RCLCPP_INFO_STREAM(this->get_logger(),"resetting");
    marker_sub_.reset();
}


// function to listen to aruco tranform
std::pair<float, float> group22_final::TurtleBot3Controller::battery_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;
    try
    {
        t_stamped = battery_tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 10ms);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return std::make_pair(0.0, 0.0);
    }
    pose_out.position.x = t_stamped.transform.translation.x;
    pose_out.position.y = t_stamped.transform.translation.y;
    pose_out.orientation = t_stamped.transform.rotation;
    return std::make_pair(pose_out.position.x, pose_out.position.y);
}


void group22_final::TurtleBot3Controller::battery_listen_cb_()
{
    auto temp1 = battery_listen_transform("world", "battery_1_frame");
    bat_colors_.push_back("blue");
    goal_x.push_back(temp1.first);
    goal_y.push_back(temp1.second);
    auto temp2 = battery_listen_transform("world", "battery_2_frame");
    bat_colors_.push_back("orange");
    goal_x.push_back(temp2.first);
    goal_y.push_back(temp2.second);
    auto temp3 = battery_listen_transform("world", "battery_3_frame");
    bat_colors_.push_back("purple");
    goal_x.push_back(temp3.first);
    goal_y.push_back(temp3.second);
    auto temp4 = battery_listen_transform("world", "battery_4_frame");
    bat_colors_.push_back("green");
    goal_x.push_back(temp4.first);
    goal_y.push_back(temp4.second);
    auto temp5 = battery_listen_transform("world", "battery_5_frame");
    bat_colors_.push_back("red");
    goal_x.push_back(temp5.first);
    goal_y.push_back(temp5.second);
    RCLCPP_INFO_STREAM(this->get_logger(),"Inside batter_lisetene" << bat_colors_.size());
}


void group22_final::TurtleBot3Controller::set_initial_pose() {
    auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
    message.header.frame_id = "map";
    message.pose.pose.position.x = 1;
    message.pose.pose.position.y = -1.617;
    message.pose.pose.orientation.x = 0.0023258279310553356;
    message.pose.pose.orientation.y = 0.0023451902769101034;
    message.pose.pose.orientation.z = -0.7007043986843412;
    message.pose.pose.orientation.w = 0.7134440666733559;
    initial_pose_pub_->publish(message);
    RCLCPP_INFO_STREAM(this->get_logger(),"Robot Initialised at X: "<< message.pose.pose.position.x << " and Y :"<< message.pose.pose.position.y);
}


void group22_final::TurtleBot3Controller::send_goal() {
    using namespace std::placeholders;
    battery_listen_cb_();
    RCLCPP_INFO_STREAM(this->get_logger(), "Inside Send Goal");

    if (!this->client_->wait_for_action_server()) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Inside Send Goal 1");
        RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
        rclcpp::shutdown();
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.7787;

    RCLCPP_INFO_STREAM(this->get_logger(),"Inside send goal: Waypint: "<<waypoints.size());
    RCLCPP_INFO_STREAM(this->get_logger(),"Inside send goal: Bat color: "<<bat_colors_.size());

    auto goal_msg = NavigateToWaypoints::Goal();

    for(long unsigned int i = 0; i < waypoints.size(); i++){
        for (long unsigned int j = 0; j < bat_colors_.size(); j++){
            if(waypoints.at(i) == bat_colors_.at(j)){
                pose.pose.position.x = goal_x.at(j);
                pose.pose.position.y = goal_y.at(j);
                goal_msg.poses.push_back(pose);
                RCLCPP_INFO_STREAM(this->get_logger(),"pushed back "<<waypoints[i]);
            }
        }
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Inside Send Goal 3");


    auto send_goal_options = rclcpp_action::Client<NavigateToWaypoints>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&TurtleBot3Controller::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&TurtleBot3Controller::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&TurtleBot3Controller::result_callback, this, _1);

    client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO_STREAM(this->get_logger(), "Goal Sent");

}

void group22_final::TurtleBot3Controller::goal_response_callback(std::shared_future<GoalHandleWaypoints::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        rclcpp::shutdown();
    } else {
        RCLCPP_INFO(this->get_logger(),
                    "Goal accepted by server, waiting for result");
    }
}

    //===============================================
void group22_final::TurtleBot3Controller::feedback_callback(GoalHandleWaypoints::SharedPtr, const std::shared_ptr<const NavigateToWaypoints::Feedback> feedback) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Robot is driving towards the goal: " << feedback->current_waypoint);
}

    //===============================================
void group22_final::TurtleBot3Controller::result_callback(
        const GoalHandleWaypoints::WrappedResult& result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
        break;
        case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
        case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
        default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    rclcpp::shutdown();
}


// void group22_final::TurtleBot3Controller::set_goal_poses(){

//     RCLCPP_INFO_STREAM(this->get_logger(), "set_goal_poses called: ");
//     RCLCPP_INFO_STREAM(this->get_logger(), "Waypoint: " << waypoints[a]);
//     // do{
//     //     if(waypoints[a] == bat_colors_[j]){
//     //         // pose.pose.position.x = goal_x[j];
//     //         // pose.pose.position.y = Y[j];
//     //         RCLCPP_INFO_STREAM(this->get_logger(), "goal_x and Y: " << goal_x[j] <<" "<<goal_y[j]);
//     //     }
//     // j = j + 1;
//     // // goal_msg.poses.push_back(pose);   
//     // }while(j<5);
// }


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<group22_final::TurtleBot3Controller>("turtle_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}