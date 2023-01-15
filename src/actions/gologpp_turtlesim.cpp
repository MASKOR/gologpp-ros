#include "action_manager.h"
#include "exog_manager.h"
#include "ros_backend.h"

#include <execution/controller.h>

#include "turtlesim/action/rotate_absolute.hpp"
#include "std_msgs/msg/bool.hpp"

template <>
std::unordered_map< std::string, gpp::unique_ptr<gpp::Value> >
ExogManager<std_msgs::msg::Bool>::params_to_map(const std_msgs::msg::Bool::ConstPtr& msg) {

	gpp::unique_ptr<gpp::Value> param (new gpp::Value(gpp::get_type<gpp::BoolType>(), bool(msg->data)));
	std::unordered_map< std::string, gpp::unique_ptr<gpp::Value> > params_to_map;
	params_to_map.insert({"data", std::move(param)});
	return params_to_map;
}

template<>
ActionManager<turtlesim::action::RotateAbsolute>::GoalT
ActionManager<turtlesim::action::RotateAbsolute>::build_goal(const gpp::Activity &a)
{
	auto goal = turtlesim::action::RotateAbsolute::Goal();
	goal.theta = a.mapped_arg_value("theta").numeric_convert<float>();
	return goal;
}

void RosBackend::define_turtlesim_actions()
{
	create_ActionManager<turtlesim::action::RotateAbsolute>("/turtle1/rotate_absolute");
	create_ExogManger<std_msgs::msg::Bool>("/exog_event");
}