#include "action_manager.h"
#include "exog_manager.h"
#include "ros_backend.h"

#include <execution/controller.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <darknet_action_msgs/obj_detectionAction.h>

namespace darknet = darknet_action_msgs;

template<>
gpp::optional<gpp::Value>
ActionManager<darknet::obj_detectionAction>::to_golog_constant(ResultT result) {
	return gpp::Value(gpp::get_type<gpp::NumberType>(), result->obj_pos);
}

template<>
ActionManager<darknet::obj_detectionAction>::GoalT
ActionManager<darknet::obj_detectionAction>::build_goal(const gpp::Activity &a)
{
	darknet::obj_detectionGoal goal;
	goal.to_detected_obj = std::string(a.mapped_arg_value("to_detected_obj"));
	return goal;
}

void RosBackend::define_darknet_actions()
{

	create_ActionManager<darknet::obj_detectionAction>("/yolo_obj_detection_position_server");

}
