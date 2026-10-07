#include <execution/controller.h>

#include "gologpp_agent/action_manager.h"
#include "gologpp_agent/exog_manager.h"
#include "gologpp_agent/ros_backend.h"
#include "man_interfaces/action/move_delta.hpp"
#include "man_interfaces/action/move_to_frame.hpp"
#include "man_interfaces/srv/gripper.hpp"

template <>
ActionManager<man_interfaces::action::MoveDelta>::GoalT ActionManager<man_interfaces::action::MoveDelta>::build_goal(
    const gpp::Activity& a) {
  auto goal = man_interfaces::action::MoveDelta::Goal();
  goal.dx = a.mapped_arg_value("dx").numeric_convert<float>();
  goal.dy = a.mapped_arg_value("dy").numeric_convert<float>();
  goal.dz = a.mapped_arg_value("dz").numeric_convert<float>();
  goal.droll = a.mapped_arg_value("droll").numeric_convert<float>();
  goal.dpitch = a.mapped_arg_value("dpitch").numeric_convert<float>();
  goal.dyaw = a.mapped_arg_value("dyaw").numeric_convert<float>();
  goal.frame_id = std::string(a.mapped_arg_value("frame_id"));  // "" -> base_link
  goal.velocity_scaling = a.mapped_arg_value("velocity_scaling").numeric_convert<float>();  // 0 -> default
  return goal;
}

template <>
ActionManager<man_interfaces::action::MoveToFrame>::GoalT
ActionManager<man_interfaces::action::MoveToFrame>::build_goal(const gpp::Activity& a) {
  auto goal = man_interfaces::action::MoveToFrame::Goal();
  goal.target_frame = std::string(a.mapped_arg_value("target_frame"));
  goal.offset.x = a.mapped_arg_value("ox").numeric_convert<float>();
  goal.offset.y = a.mapped_arg_value("oy").numeric_convert<float>();
  goal.offset.z = a.mapped_arg_value("oz").numeric_convert<float>();
  // 0 = KEEP_CURRENT, 1 = TARGET_FRAME, 2 = FIXED
  goal.orientation_mode = static_cast<uint8_t>(a.mapped_arg_value("orientation_mode").numeric_convert<float>());
  goal.orientation.x = a.mapped_arg_value("qx").numeric_convert<float>();
  goal.orientation.y = a.mapped_arg_value("qy").numeric_convert<float>();
  goal.orientation.z = a.mapped_arg_value("qz").numeric_convert<float>();
  goal.orientation.w = a.mapped_arg_value("qw").numeric_convert<float>();
  goal.velocity_scaling = a.mapped_arg_value("velocity_scaling").numeric_convert<float>();  // 0 -> default
  return goal;
}

template <>
ServiceManager<man_interfaces::srv::Gripper>::RequestT ServiceManager<man_interfaces::srv::Gripper>::build_request(
    const gpp::Activity& a) {
  auto request = std::make_shared<man_interfaces::srv::Gripper::Request>();
  // 0 = OPEN, 1 = CLOSE, 2 = POSITION
  request->command = static_cast<uint8_t>(a.mapped_arg_value("command").numeric_convert<float>());
  request->position = a.mapped_arg_value("position").numeric_convert<float>();
  request->max_effort = a.mapped_arg_value("max_effort").numeric_convert<float>();  // 0 -> default
  return request;
}

// The service call finishes even if the gripper failed, so return success to the agent
template <>
gpp::optional<gpp::Value> ServiceManager<man_interfaces::srv::Gripper>::to_golog_constant(ResponseT result) {
  return gpp::Value(gpp::get_type<gpp::BoolType>(), static_cast<bool>(result.get()->success));
}

void RosBackend::define_man_interfaces_actions() {
  built_interface_names.push_back("man_interfaces");

  create_ActionManager<man_interfaces::action::MoveDelta>("/move_delta");
  create_ActionManager<man_interfaces::action::MoveToFrame>("/move_to_frame");
  create_ServiceManager<man_interfaces::srv::Gripper>("/gripper");
}
