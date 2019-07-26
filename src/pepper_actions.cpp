#include "action_manager.h"
#include "ros_backend.h"


#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <darknet_actions_msgs/obj_detectionAction.h>

#include <naoqi_wrapper_msgs/NaoQi_animatedSayAction.h>
#include <naoqi_wrapper_msgs/NaoQi_animationAction.h>
#include <naoqi_wrapper_msgs/NaoQi_dialogAction.h>
#include <naoqi_wrapper_msgs/FaceTracking.h>
#include <naoqi_wrapper_msgs/NaoQi_lookAtAction.h>
#include <naoqi_wrapper_msgs/NaoQi_openWebsiteAction.h>
#include <naoqi_wrapper_msgs/NaoQi_sayAction.h>
#include <naoqi_wrapper_msgs/NaoQi_subscribeAction.h>


namespace naoqi = naoqi_wrapper_msgs;
namespace darknet = darknet_actions_msgs;


template<>
gpp::Value *
ActionManager<naoqi::NaoQi_dialogAction>::to_golog_constant(
	naoqi::NaoQi_dialogResultConstPtr result
) {
	return new gpp::Value(gpp::StringType::name(), result->outcome);
}


template<>
gpp::Value *
ActionManager<darknet::obj_detectionAction>::to_golog_constant(
	darknet::obj_detectionResultConstPtr result
) {
	return new gpp::Value(gpp::NumberType::name(),result->obj_pos);
}


template<>
gpp::Value *
ActionManager<naoqi::NaoQi_openWebsiteAction>::to_golog_constant(
	naoqi_wrapper_msgs::NaoQi_openWebsiteResultConstPtr result
) {
	return new gpp::Value(gpp::StringType::name(), result->command);
}


template<>
ActionManager<naoqi::NaoQi_dialogAction>::GoalT
ActionManager<naoqi::NaoQi_dialogAction>::build_goal(const gpp::Activity &a)
{
	naoqi_wrapper_msgs::NaoQi_dialogGoal goal;
	goal.dialogTopicFile.data = std::string(a.mapped_arg_value("topic_file"));
	return goal;
}


void RosBackend::define_actions()
{
	define_action_client<naoqi::NaoQi_dialogAction>("TODO: TOPIC_NAME");
}


/*
	ROS_INFO("Execute Transition. Action: %s arg: %s", a->target()->name().c_str(), a->args().at(0)->str().c_str());
	a->target()->mapping().name();
	if(a->target()->mapping().name() == "animatedsay"){
		naoqi_wrapper_msgs::NaoQi_animatedSayGoal say_goal;
		say_goal.animatedMessage.data =  a->args().at(0)->str();
		execute_transition_wrapper<naoqi_wrapper_msgs::NaoQi_animatedSayAction>(say_goal, a);

	}else if(a->target()->mapping().name() == "movetoframe"){
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = a->args().at(0)->str();
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = 0;
		goal.target_pose.pose.position.y = 0;
		goal.target_pose.pose.orientation.w = 1;
		execute_transition_wrapper<move_base_msgs::MoveBaseAction>(goal, a);

	}else if(a->target()->mapping().name() == "animation"){
		naoqi_wrapper_msgs::NaoQi_animationGoal goal;
		goal.animation.data = a->args().at(0)->str();
		execute_transition_wrapper<naoqi_wrapper_msgs::NaoQi_animationAction>(goal, a);

	} else if(a->target()->mapping().name() == "dialog"){
		naoqi_wrapper_msgs::NaoQi_dialogGoal goal;
		goal.dialogTopicFile.data = a->args().at(0)->str();
		execute_transition_wrapper<naoqi_wrapper_msgs::NaoQi_dialogAction>(goal, a);

	} else if(a->target()->mapping().name() == "lookAt"){
		naoqi_wrapper_msgs::NaoQi_lookAtGoal goal;
		goal.position.push_back(a->target()->mapping().value_for("x"));
		goal.position.push_back(double(*a->args().at(1)));
		goal.position.push_back(double(*a->args().at(2)));
		goal.frame = 2;
		goal.fractionMaxSpeed = 1;
		goal.useWholeBody = false;

		//goal.position.push_back(int(*a->args().at(1)));
		//goal.position.push_back(int(*a->args().at(2)));
		execute_transition_wrapper<naoqi_wrapper_msgs::NaoQi_lookAtAction>(goal, a);

	} else if(a->target()->mapping().name() == "openWebsite"){
		naoqi_wrapper_msgs::NaoQi_openWebsiteGoal goal;
		goal.url.data = a->args().at(0)->str();
		goal.waitForWebCommand = bool(*a->args().at(1));
		execute_transition_wrapper<naoqi_wrapper_msgs::NaoQi_openWebsiteAction>(goal, a);

	} else if(a->target()->mapping().name() == "say"){
		naoqi_wrapper_msgs::NaoQi_sayGoal goal;
		goal.message.data =  a->args().at(0)->str();
		execute_transition_wrapper<naoqi_wrapper_msgs::NaoQi_sayAction>(goal, a);

	} else if(a->target()->mapping().name() == "subscribe"){
		naoqi_wrapper_msgs::NaoQi_subscribeGoal goal;
		goal.eventName.data = a->args().at(0)->str();
		execute_transition_wrapper<naoqi_wrapper_msgs::NaoQi_subscribeAction>(goal, a);

    } else if (a->target()->mapping().name() == "detect_position") {
		darknet_actions_msgs::obj_detectionGoal goal;
		goal.to_detected_obj = a->args().at(0)->str();
		execute_transition_wrapper<darknet_actions_msgs::obj_detectionAction>(goal, a);

    } else if (a->target()->mapping().name() == "face_tracking") {
		std::thread service_thread( [&] (bool enable, shared_ptr<Activity> activity) {
			naoqi_wrapper_msgs::FaceTracking srv;
			srv.request.enableFaceTracking = enable;
			if (facetracking_client.call(srv)) {
				update_activity(activity->transition(Transition::Hook::FINISH));
			} else {
				update_activity(activity->transition(Transition::Hook::FAIL));
			}
		}, bool(*a->args().at(0)), a);
		service_thread.detach();

	} else if (a->target()->mapping().name() == "logger"){
		std::thread logger_thread( [&] (shared_ptr<Activity> activity) {
			for(size_t i = 0; i < activity->args().size(); i++) {
				ROS_INFO_STREAM(activity->args().at(i)->str());
			}
			update_activity(activity->transition(Transition::Hook::FINISH));
		},a );
		logger_thread.detach();
	}
	else{
		ROS_INFO("No Action is matching.");
	}
*/