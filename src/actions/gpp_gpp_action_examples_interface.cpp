#include "action_manager.h"
#include "exog_manager.h"
#include "ros_backend.h"

#include <execution/controller.h>

#include "gpp_action_examples_interface/srv/print.hpp"
#include "gpp_action_examples_interface/action/trajectory_to_frame.hpp"

template<>
ServiceManager<gpp_action_examples_interface::srv::Print>::RequestT
ServiceManager<gpp_action_examples_interface::srv::Print>::build_request(const gpp::Activity &a)
{
	auto request = std::make_shared<gpp_action_examples_interface::srv::Print::Request>();
	request->request_print = std::string(a.mapped_arg_value("text"));
	return request;
}

template<>
gpp::optional<gpp::Value>
ServiceManager<gpp_action_examples_interface::srv::Print>::to_golog_constant(ResponseT result){
	return gpp::Value(gpp::get_type<gpp::StringType>(), result.get()->response_print);
}

template<>
ActionManager<gpp_action_examples_interface::action::TrajectoryToFrame>::GoalT
ActionManager<gpp_action_examples_interface::action::TrajectoryToFrame>::build_goal(const gpp::Activity &a)
{
	auto goal = gpp_action_examples_interface::action::TrajectoryToFrame::Goal();
	goal.frame_id = std::string(a.mapped_arg_value("frame_id"));
	return goal;
}

void RosBackend::define_action_examples_actions()
{
	create_ActionManager<gpp_action_examples_interface::action::TrajectoryToFrame>("/trajectoryToFrame");
	create_ServiceManager<gpp_action_examples_interface::srv::Print>("/print_string");
}