#include <execution/controller.h>

#include "gologpp_agent/action_manager.h"
#include "gologpp_agent/exog_manager.h"
#include "gologpp_agent/ros_backend.h"
#include "gpp_action_examples_interface/action/durative.hpp"
#include "gpp_action_examples_interface/action/play_audio.hpp"
#include "gpp_action_examples_interface/action/trajectory_to_frame.hpp"
#include "gpp_action_examples_interface/srv/print.hpp"
#include "gpp_action_examples_interface/srv/spot_body_pose.hpp"

template <>
ServiceManager<gpp_action_examples_interface::srv::Print>::RequestT
ServiceManager<gpp_action_examples_interface::srv::Print>::build_request(const gpp::Activity& a) {
  auto request = std::make_shared<gpp_action_examples_interface::srv::Print::Request>();
  request->request_print = std::string(a.mapped_arg_value("text"));
  return request;
}

template <>
gpp::optional<gpp::Value> ServiceManager<gpp_action_examples_interface::srv::Print>::to_golog_constant(
    ResponseT result) {
  return gpp::Value(gpp::get_type<gpp::StringType>(), result.get()->response_print);
}

template <>
ActionManager<gpp_action_examples_interface::action::TrajectoryToFrame>::GoalT
ActionManager<gpp_action_examples_interface::action::TrajectoryToFrame>::build_goal(const gpp::Activity& a) {
  auto goal = gpp_action_examples_interface::action::TrajectoryToFrame::Goal();
  goal.frame_id = std::string(a.mapped_arg_value("frame_id"));
  return goal;
}

template <>
ActionManager<gpp_action_examples_interface::action::PlayAudio>::GoalT
ActionManager<gpp_action_examples_interface::action::PlayAudio>::build_goal(const gpp::Activity& a) {
  auto goal = gpp_action_examples_interface::action::PlayAudio::Goal();
  goal.audio_file = std::string(a.mapped_arg_value("audio_file"));
  return goal;
}

template <>
ActionManager<gpp_action_examples_interface::action::Durative>::GoalT
ActionManager<gpp_action_examples_interface::action::Durative>::build_goal(const gpp::Activity& a) {
  auto goal = gpp_action_examples_interface::action::Durative::Goal();
  goal.duration.sec = a.mapped_arg_value("seconds").numeric_convert<float>();
  return goal;
}

template <>
ServiceManager<gpp_action_examples_interface::srv::SpotBodyPose>::RequestT
ServiceManager<gpp_action_examples_interface::srv::SpotBodyPose>::build_request(const gpp::Activity& a) {
  auto request = std::make_shared<gpp_action_examples_interface::srv::SpotBodyPose::Request>();
  request->pose.position.x = a.mapped_arg_value("px").numeric_convert<float>();
  request->pose.position.y = a.mapped_arg_value("py").numeric_convert<float>();
  request->pose.position.z = a.mapped_arg_value("pz").numeric_convert<float>();
  request->pose.orientation.x = a.mapped_arg_value("ox").numeric_convert<float>();
  request->pose.orientation.y = a.mapped_arg_value("oy").numeric_convert<float>();
  request->pose.orientation.z = a.mapped_arg_value("oz").numeric_convert<float>();
  request->pose.orientation.w = a.mapped_arg_value("ow").numeric_convert<float>();
  return request;
}

void RosBackend::define_gpp_action_examples_interface_actions() {
  built_interface_names.push_back("gpp_action_examples");

  create_ActionManager<gpp_action_examples_interface::action::Durative>("move_circle");
  create_ActionManager<gpp_action_examples_interface::action::TrajectoryToFrame>("trajectoryToFrame");
  create_ActionManager<gpp_action_examples_interface::action::PlayAudio>("play_audio");
  create_ServiceManager<gpp_action_examples_interface::srv::Print>("/print_string");
  create_ServiceManager<gpp_action_examples_interface::srv::SpotBodyPose>("body_pose_service");
}
