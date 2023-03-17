#include "action_manager.h"
#include "exog_manager.h"
#include "ros_backend.h"

#include <execution/controller.h>

#include "webots_spot_msgs/action/stack.hpp"
#include "std_msgs/msg/bool.hpp"

template<>
ActionManager<webots_spot_msgs::action::Stack>::GoalT
ActionManager<webots_spot_msgs::action::Stack>::build_goal(const gpp::Activity &a)
{
	auto goal = webots_spot_msgs::action::Stack::Goal();
	goal.block = std::string(a.mapped_arg_value("block"));
	goal.location = std::string(a.mapped_arg_value("location"));
	return goal;
}

void RosBackend::define_webots_spot_msgs_actions()
{
	create_ActionManager<webots_spot_msgs::action::Stack>("/stack");
}