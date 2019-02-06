#include "pepper_backend.h"
#include <iostream>
#include <typeinfo>
#include <string>
#include <geometry_msgs/PoseStamped.h>


namespace gologpp {

Pepper_Backend::Pepper_Backend()
: move_base_client("move_base", true)
, animated_say_client("/naoqi_animatedSay_server/animatedSay", true)
, animation_client("/naoqi_animation_server/naoqi_animation", true)
, dialog_client("/naoqi_dialog_server", true)
//, facetracking_client("/naoqi_wrapper/naoqi_faceTracking_service", true)
, lookAt_client("/naoqi_lookAt_server/lookAt", true)
, openWebsite_client("/naoqi_openWebsite_server", true)
, say_client("/naoqi_say_server/naoqi_say", true)
, subscribe_client("/naoqi_subscribe_server/subscribe", true)
, action_clients {move_base_client, animated_say_client, animation_client, dialog_client, /*facetracking_client,*/ lookAt_client, openWebsite_client, say_client, subscribe_client}
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
	ROS_INFO("Execute Transition. Action: %s arg: %s", a->target()->name().c_str(), a->args().at(0)->str().c_str());
	a->target()->mapping().name();
	if(a->target()->mapping().name() == "stack"){
		//trans.target().mapping().args[0];

	}else if(a->target()->mapping().name() == "animatedsay"){
		naoqi_wrapper_msgs::NaoQi_animatedSayGoal say_goal;
		say_goal.animatedMessage.data =  a->args().at(0)->str();
		execute_transition_wrapper<naoqi_wrapper_msgs::NaoQi_animatedSayAction>(say_goal, a);
		ROS_INFO("Sending goal");

	}else if(a->target()->mapping().name() == "movetoframe"){
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = a->args().at(0)->str().c_str();
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = 0;
		goal.target_pose.pose.position.y = 0;
		goal.target_pose.pose.orientation.w = 1;
		execute_transition_wrapper<move_base_msgs::MoveBaseAction>(goal, a);

	}else if(a->target()->mapping().name() == "animation"){
		naoqi_wrapper_msgs::NaoQi_animationGoal goal;
		goal.animation.data = a->args().at(0)->str().c_str();
		execute_transition_wrapper<naoqi_wrapper_msgs::NaoQi_animationAction>(goal, a);

	} else if(a->target()->mapping().name() == "dialog"){
		naoqi_wrapper_msgs::NaoQi_dialogGoal goal;
		goal.dialogTopicFile.data = a->args().at(0)->str().c_str();
		execute_transition_wrapper<naoqi_wrapper_msgs::NaoQi_dialogAction>(goal, a);

	} else if(a->target()->mapping().name() == "lookAt"){
		naoqi_wrapper_msgs::NaoQi_lookAtGoal goal;
		//goal.frame =  a->args().at(0)->str().c_str();
		execute_transition_wrapper<naoqi_wrapper_msgs::NaoQi_lookAtAction>(goal, a);

	} else if(a->target()->mapping().name() == "openWebsite"){
		naoqi_wrapper_msgs::NaoQi_openWebsiteGoal goal;
		goal.url.data = a->args().at(0)->str().c_str();
		execute_transition_wrapper<naoqi_wrapper_msgs::NaoQi_openWebsiteAction>(goal, a);

	} else if(a->target()->mapping().name() == "say"){
		naoqi_wrapper_msgs::NaoQi_sayGoal goal;
		goal.message.data =  a->args().at(0)->str().c_str();
		execute_transition_wrapper<naoqi_wrapper_msgs::NaoQi_sayAction>(goal, a);

	} else if(a->target()->mapping().name() == "subscribe"){
		naoqi_wrapper_msgs::NaoQi_subscribeGoal goal;
		goal.eventName.data = a->args().at(0)->str().c_str();
		execute_transition_wrapper<naoqi_wrapper_msgs::NaoQi_subscribeAction>(goal, a);
	}
	else{
		ROS_INFO("No Action is matching.");
	}
}
void Pepper_Backend::preempt_activity(shared_ptr<Transition> trans)
{
	//handles case when gpp-agent preempts action.
	if(trans->target()->name() == "movetoframe")
		move_base_client.cancelAllGoals();
	else if(trans->target()->name() == "say")
		animated_say_client.cancelAllGoals();
}
Clock::time_point Pepper_Backend::time() const noexcept
{
	//Clock::duration rv = std::chrono::steady_clock::now().time_since_epoch();
	//return Clock::time_point(rv);
	return Clock::time_point();
}

} //namespace gologpp
