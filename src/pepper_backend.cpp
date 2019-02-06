#include "pepper_backend.h"
#include <iostream>
#include <typeinfo>
#include <string>
#include <geometry_msgs/PoseStamped.h>


namespace gologpp {

Pepper_Backend::Pepper_Backend()
: animated_say_client("/naoqi_animatedSay_server/animatedSay", true)
, move_base_client("move_base", true)
, action_clients { animated_say_client, move_base_client}
{
	//ROS_INFO("Waiting for action server to start.");
	//animated_say_client.waitForServer(); //will wait for infinite time
	//  doput_client.waitForServer();
	//  ROS_INFO("Action server started.");
}


Pepper_Backend::~Pepper_Backend()
{}


void Pepper_Backend::execute_activity(shared_ptr<Activity> a)
{
	ROS_INFO("Execute Transition. Action: %s arg: %s", a->action()->name().c_str(), a->args().at(0)->str().c_str());
	a->action()->mapping().name();
	if(a->action()->mapping().name() == "stack"){
		//trans.action().mapping().args[0];

	}else if(a->action()->mapping().name() == "say"){
		naoqi_wrapper_msgs::NaoQi_animatedSayGoal say_goal;
		say_goal.animatedMessage.data =  a->args().at(0)->str();
		execute_transition_wrapper<naoqi_wrapper_msgs::NaoQi_animatedSayAction>(say_goal, a);
		ROS_INFO("Sending goal");

	}else if(a->action()->mapping().name() == "movetoframe"){
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = a->args().at(0)->str().c_str();
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = 0;
		goal.target_pose.pose.position.y = 0;
		goal.target_pose.pose.orientation.w = 1;
		execute_transition_wrapper<move_base_msgs::MoveBaseAction>(goal, a);
	}
	else{
		ROS_INFO("No Action is matching.");
	}
}
void Pepper_Backend::preempt_activity(shared_ptr<Transition> trans)
{
	//handles case when gpp-agent preempts action.
	if(trans->action()->name() == "movetoframe")
		move_base_client.cancelAllGoals();
	else if(trans->action()->name() == "say")
		animated_say_client.cancelAllGoals();
}
Clock::time_point Pepper_Backend::time() const noexcept
{
	//Clock::duration rv = std::chrono::steady_clock::now().time_since_epoch();
	//return Clock::time_point(rv);
	return Clock::time_point();
}

} //namespace gologpp
