#include "action_manager.h"
#include "exog_manager.h"
#include "ros_backend.h"

#include <execution/controller.h>


#include "turtlesim/action/rotate_absolute.hpp"


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
}