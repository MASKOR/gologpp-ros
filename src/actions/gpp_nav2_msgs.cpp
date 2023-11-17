#include <execution/controller.h>

#include "gologpp_agent/action_manager.h"
#include "gologpp_agent/ros_backend.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"

template <>
ActionManager<nav2_msgs::action::NavigateToPose>::GoalT ActionManager<nav2_msgs::action::NavigateToPose>::build_goal(
    const gpp::Activity& a) {
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  auto agent_node = Singleton::instance();
  goal.pose.header.stamp = rclcpp::Clock().now();
  goal.pose.header.frame_id = "map";

  goal.pose.pose.position.x = a.mapped_arg_value("px").numeric_convert<float>();
  goal.pose.pose.position.y = a.mapped_arg_value("py").numeric_convert<float>();
  goal.pose.pose.position.z = a.mapped_arg_value("pz").numeric_convert<float>();

  goal.pose.pose.orientation.x = a.mapped_arg_value("ox").numeric_convert<float>();
  goal.pose.pose.orientation.y = a.mapped_arg_value("oy").numeric_convert<float>();
  goal.pose.pose.orientation.z = a.mapped_arg_value("oz").numeric_convert<float>();
  goal.pose.pose.orientation.w = a.mapped_arg_value("ow").numeric_convert<float>();

  return goal;
}

void RosBackend::define_nav2_msgs_actions() {
  built_interface_names.push_back("nav2_msgs");

  create_ActionManager<nav2_msgs::action::NavigateToPose>("/navigate_to_pose");
}
