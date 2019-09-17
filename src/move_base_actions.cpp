#ifdef MOVE_BASE_MSGS_PKG
#include "action_manager.h"
#include "exog_manager.h"
#include "ros_backend.h"

#include <model/execution.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <move_base_msgs/MoveBaseAction.h>


template<>
ActionManager<move_base_msgs::MoveBaseAction>::GoalT
ActionManager<move_base_msgs::MoveBaseAction>::build_goal(const gpp::Activity &a)
{
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = std::string(a.mapped_arg_value("frame_id"));
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = int(a.mapped_arg_value("x"));
	goal.target_pose.pose.position.y = int(a.mapped_arg_value("y"));
	goal.target_pose.pose.orientation.w = int(a.mapped_arg_value("w"));;
	return goal;
}

void RosBackend::define_move_base_actions()
{
	create_ActionManager<move_base_msgs::MoveBaseAction>("move_base");
}

#endif //MOVE_BASE_MSGS_PKG
