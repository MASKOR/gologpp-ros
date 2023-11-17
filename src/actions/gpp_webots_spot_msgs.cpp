#include <execution/controller.h>

#include "gologpp_agent/action_manager.h"
#include "gologpp_agent/exog_manager.h"
#include "gologpp_agent/ros_backend.h"
#include "webots_spot_msgs/action/peak_and_detect_object.hpp"
#include "webots_spot_msgs/action/stack.hpp"
#include "webots_spot_msgs/srv/block_pose.hpp"
#include "webots_spot_msgs/srv/spot_height.hpp"
#include "webots_spot_msgs/srv/spot_motion.hpp"

template <>
gpp::optional<gpp::Value> ServiceManager<webots_spot_msgs::srv::BlockPose>::to_golog_constant(ResponseT result) {
  return gpp::Value(gpp::get_type<gpp::SymbolType>(), result.get()->location);
}

template <>
ServiceManager<webots_spot_msgs::srv::BlockPose>::RequestT
ServiceManager<webots_spot_msgs::srv::BlockPose>::build_request(const gpp::Activity& a) {
  auto request = std::make_shared<webots_spot_msgs::srv::BlockPose::Request>();
  request->block = std::string(a.mapped_arg_value("block"));
  return request;
}

template <>
ActionManager<webots_spot_msgs::action::Stack>::GoalT ActionManager<webots_spot_msgs::action::Stack>::build_goal(
    const gpp::Activity& a) {
  auto goal = webots_spot_msgs::action::Stack::Goal();
  goal.block = std::string(a.mapped_arg_value("block"));
  goal.location = std::string(a.mapped_arg_value("location"));
  return goal;
}

template <>
ServiceManager<webots_spot_msgs::srv::SpotMotion>::RequestT
ServiceManager<webots_spot_msgs::srv::SpotMotion>::build_request(const gpp::Activity& a) {
  auto request = std::make_shared<webots_spot_msgs::srv::SpotMotion::Request>();
  request->override = std::string(a.mapped_arg_value("override")) == "true" ? true : false;
  return request;
}

template <>
ServiceManager<webots_spot_msgs::srv::SpotHeight>::RequestT
ServiceManager<webots_spot_msgs::srv::SpotHeight>::build_request(const gpp::Activity& a) {
  auto request = std::make_shared<webots_spot_msgs::srv::SpotHeight::Request>();
  request->height = a.mapped_arg_value("height").numeric_convert<float>();
  return request;
}

template <>
ActionManager<webots_spot_msgs::action::PeakAndDetectObject>::GoalT
ActionManager<webots_spot_msgs::action::PeakAndDetectObject>::build_goal(const gpp::Activity& a) {
  auto goal = webots_spot_msgs::action::PeakAndDetectObject::Goal();
  goal.image = std::string(a.mapped_arg_value("image"));
  return goal;
}

void RosBackend::define_webots_spot_msgs_actions() {
  built_interface_names.push_back("webots_spot_msgs");

  create_ServiceManager<webots_spot_msgs::srv::BlockPose>("/get_block_pose");
  create_ActionManager<webots_spot_msgs::action::Stack>("/stack");
  create_ServiceManager<webots_spot_msgs::srv::SpotMotion>("/Spot/lie_down");
  create_ServiceManager<webots_spot_msgs::srv::SpotHeight>("/Spot/set_height");
  create_ActionManager<webots_spot_msgs::action::PeakAndDetectObject>("/detect_object_in_box");
}
