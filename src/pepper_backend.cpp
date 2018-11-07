#include "pepper_backend.h"
#include <iostream>


namespace gologpp {

Pepper_Backend::Pepper_Backend()
: animated_say_client("/naoqi_animatedSay_server/animatedSay", true)
, action_clients { animated_say_client }
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
	ROS_INFO("Execute Transition.");
	trans->action().mapping().name();
	ROS_INFO("Action: %s", trans->action().name().c_str());
	if(trans->action().mapping().name() == "stack"){
		ROS_INFO("Start Put-Action.");
		//trans.action().mapping().args[0];

	}else if(trans->action().mapping().name() == "say"){
		ROS_INFO("Start Say-Action.");
		pepper_smach::NaoQi_animatedSayGoal say_goal;
		//say_goal.animatedMessage.data = trans.action().mapping().args().at(0);
		execute_transition_wrapper<pepper_smach::NaoQi_animatedSayAction>(say_goal, trans);
		//actionlib::SimpleActionClient<pepper_smach::NaoQi_animatedSayAction>::Goal::

		ROS_INFO("Message sent.");

	}else{
		ROS_INFO("No Action is matching.");
	}
	//rosrun actionlib_tutorials calc_client
	//system("bash -i -c 'rosrun actionlib_tutorials calc_client'");
}


} //namespace gologpp
