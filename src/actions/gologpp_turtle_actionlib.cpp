#include "action_manager.h"
#include "exog_manager.h"
#include "ros_backend.h"

#include <execution/controller.h>

#include <turtle_actionlib/ShapeAction.h>


template<>
ActionManager<turtle_actionlib::ShapeAction>::GoalT
ActionManager<turtle_actionlib::ShapeAction>::build_goal(const gpp::Activity &a)
{
	turtle_actionlib::ShapeGoal goal;
	goal.edges = a.mapped_arg_value("edges").numeric_convert<int>();
	goal.radius = a.mapped_arg_value("radius").numeric_convert<float>();
	return goal;
}


void RosBackend::define_turtle_actions()
{ create_ActionManager<turtle_actionlib::ShapeAction>("turtle_shape"); }
