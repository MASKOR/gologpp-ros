#include "gologpp_agent/action_manager.h"
#include "gologpp_agent/exog_manager.h"
#include "gologpp_agent/ros_backend.h"

#include <execution/controller.h>

#include <mbf_msgs/action/move_base.hpp>


template <>
ActionManager<mbf_msgs::action::MoveBase>::GoalT ActionManager<mbf_msgs::action::MoveBase>::build_goal(
    const gpp::Activity& a) {
  auto goal = mbf_msgs::action::MoveBase::Goal();
  auto agent_node = Singleton::instance();
  goal.target_pose.header.stamp = rclcpp::Clock().now();
  goal.target_pose.header.frame_id = "map";

  goal.target_pose.pose.position.x = a.mapped_arg_value("px").numeric_convert<float>();
  goal.target_pose.pose.position.y = a.mapped_arg_value("py").numeric_convert<float>();
  goal.target_pose.pose.position.z = a.mapped_arg_value("pz").numeric_convert<float>();

  goal.target_pose.pose.orientation.x = a.mapped_arg_value("ox").numeric_convert<float>();
  goal.target_pose.pose.orientation.y = a.mapped_arg_value("oy").numeric_convert<float>();
  goal.target_pose.pose.orientation.z = a.mapped_arg_value("oz").numeric_convert<float>();
  goal.target_pose.pose.orientation.w = a.mapped_arg_value("ow").numeric_convert<float>();

  return goal;
}



void RosBackend::define_mbf_msgs_actions()
{
    	create_ActionManager<mbf_msgs::action::MoveBase>("/move_base_flex/move_base");
}