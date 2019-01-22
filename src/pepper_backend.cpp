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


void Pepper_Backend::execute_transition(shared_ptr <Transition> trans)
{
	ROS_INFO("Execute Transition. Action: %s arg: %s", trans->action()->name().c_str(), trans->args().at(0)->str().c_str());
	trans->action()->mapping().name();
	if(trans->action()->mapping().name() == "stack"){
		//trans.action().mapping().args[0];

	}else if(trans->action()->mapping().name() == "say"){
		naoqi_actions::NaoQi_animatedSayGoal say_goal;
		say_goal.animatedMessage.data = "Ziel erreicht, Schulnoten";
		execute_transition_wrapper<naoqi_actions::NaoQi_animatedSayAction>(say_goal, trans);
		ROS_INFO("Sending goal");

	}else if(trans->action()->mapping().name() == "movetoframe"){
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = trans->args().at(0)->str().c_str();
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = 0;
		goal.target_pose.pose.position.y = 0;
		goal.target_pose.pose.orientation.w = 1;
		execute_transition_wrapper<move_base_msgs::MoveBaseAction>(goal, trans);
	}
	else{
		ROS_INFO("No Action is matching.");
	}
}


} //namespace gologpp
