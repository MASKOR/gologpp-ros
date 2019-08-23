#include "action_manager.h"
#include "ros_backend.h"
#include "exog_manager.h"

#include <model/action.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <darknet_action_msgs/obj_detectionAction.h>

#include <naoqi_wrapper_msgs/NaoQi_animatedSayAction.h>
#include <naoqi_wrapper_msgs/NaoQi_animationAction.h>
#include <naoqi_wrapper_msgs/NaoQi_dialogAction.h>
#include <naoqi_wrapper_msgs/FaceTracking.h>
#include <naoqi_wrapper_msgs/NaoQi_lookAtAction.h>
#include <naoqi_wrapper_msgs/NaoQi_openWebsiteAction.h>
#include <naoqi_wrapper_msgs/NaoQi_sayAction.h>
#include <naoqi_wrapper_msgs/NaoQi_subscribeAction.h>

#include <naoqi_bridge_msgs/Bumper.h>

namespace naoqi = naoqi_wrapper_msgs;
namespace darknet = darknet_action_msgs;


template<>
gpp::Value *
ActionManager<naoqi::NaoQi_dialogAction>::to_golog_constant(ResultT result){
	return new gpp::Value(gpp::StringType::name(), result->outcome);
}


template<>
gpp::Value *
ActionManager<darknet::obj_detectionAction>::to_golog_constant(ResultT result) {
	return new gpp::Value(gpp::NumberType::name(),result->obj_pos);
}


template<>
gpp::Value *
ActionManager<naoqi::NaoQi_openWebsiteAction>::to_golog_constant(ResultT result) {
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

template<>
ActionManager<naoqi::NaoQi_sayAction>::GoalT
ActionManager<naoqi::NaoQi_sayAction>::build_goal(const gpp::Activity &a)
{
	naoqi_wrapper_msgs::NaoQi_sayGoal goal;
	goal.message.data = std::string(a.mapped_arg_value("say_string"));
	return goal;
}

template<>
ActionManager<naoqi::NaoQi_lookAtAction>::GoalT
ActionManager<naoqi::NaoQi_lookAtAction>::build_goal(const gpp::Activity &a)
{
	naoqi_wrapper_msgs::NaoQi_lookAtGoal goal;
	goal.position.push_back(double(a.mapped_arg_value("x")));
	goal.position.push_back(double(a.mapped_arg_value("y")));
	goal.position.push_back(double(a.mapped_arg_value("z")));
	goal.frame = int(a.mapped_arg_value("frame"));
	goal.fractionMaxSpeed = int(a.mapped_arg_value("fractionMaxSpeed"));
	goal.useWholeBody = bool(a.mapped_arg_value("useWholeBody"));
	return goal;
}

template<>
ActionManager<move_base_msgs::MoveBaseAction>::GoalT
ActionManager<move_base_msgs::MoveBaseAction>::build_goal(const gpp::Activity &a)
{
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = std::string(a.mapped_arg_value("frame_id"));
	goal.target_pose.header.stamp =  ros::Time::now();
	goal.target_pose.pose.position.x = int(a.mapped_arg_value("x"));
	goal.target_pose.pose.position.y = int(a.mapped_arg_value("y"));
	goal.target_pose.pose.orientation.w = int(a.mapped_arg_value("w"));;
	return goal;
}

template<>
ActionManager<naoqi::NaoQi_openWebsiteAction>::GoalT
ActionManager<naoqi::NaoQi_openWebsiteAction>::build_goal(const gpp::Activity &a)
{
	naoqi_wrapper_msgs::NaoQi_openWebsiteGoal goal;
	goal.url.data = std::string(a.mapped_arg_value("url"));
	goal.waitForWebCommand = bool(a.mapped_arg_value("waitForWebCommand"));
	return goal;
}

template<>
ActionManager<darknet::obj_detectionAction>::GoalT
ActionManager<darknet::obj_detectionAction>::build_goal(const gpp::Activity &a)
{
	darknet::obj_detectionGoal goal;
	goal.to_detected_obj = std::string(a.mapped_arg_value("to_detected_obj"));
	return goal;
}

template<>
ActionManager<naoqi::NaoQi_animatedSayAction>::GoalT
ActionManager<naoqi::NaoQi_animatedSayAction>::build_goal(const gpp::Activity &a)
{
	naoqi_wrapper_msgs::NaoQi_animatedSayGoal goal;
	goal.animatedMessage.data = std::string(a.mapped_arg_value("animatedMessage"));
	return goal;
}

template<>
ActionManager<naoqi::NaoQi_animationAction>::GoalT
ActionManager<naoqi::NaoQi_animationAction>::build_goal(const gpp::Activity &a)
{
	naoqi_wrapper_msgs::NaoQi_animationGoal goal;
	goal.animation.data = std::string(a.mapped_arg_value("animation"));
	return goal;
}

template<>
ActionManager<naoqi::NaoQi_subscribeAction>::GoalT
ActionManager<naoqi::NaoQi_subscribeAction>::build_goal(const gpp::Activity &a)
{
	naoqi_wrapper_msgs::NaoQi_subscribeGoal goal;
	goal.eventName.data = std::string(a.mapped_arg_value("eventName"));
	return goal;
}

void RosBackend::define_actions()
{
	define_action_client<naoqi::NaoQi_dialogAction>("/naoqi_dialog_server");
	define_action_client<naoqi::NaoQi_sayAction>("/naoqi_say_server/naoqi_say");
	define_action_client<naoqi::NaoQi_lookAtAction>("/naoqi_lookAt_server/lookAt");
	define_action_client<move_base_msgs::MoveBaseAction>("move_base");
	define_action_client<naoqi::NaoQi_openWebsiteAction>("/naoqi_openWebsite_server/openWebsite");
	define_action_client<darknet::obj_detectionAction>("/yolo_obj_detection_position_server");
	define_action_client<naoqi::NaoQi_animatedSayAction>("/naoqi_animatedSay_server/animatedSay");
	define_action_client<naoqi::NaoQi_animationAction>("/naoqi_animation_server/naoqi_animation");
	define_action_client<naoqi::NaoQi_subscribeAction>("/naoqi_subscribe_server/subscribe");
}

void bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr& msg)
{
	ROS_INFO_STREAM("I heard: " << int(msg->statePressed));
	for (std::vector<std::shared_ptr<gpp::Global>>::iterator it = gpp::global_scope().globals().begin();
			it != gpp::global_scope().globals().end(); ++it){
		if(gpp::shared_ptr< gpp::ExogAction> exog = std::dynamic_pointer_cast< gpp::ExogAction>(*it)){
			if(exog->mapping().backend_name() == "/pepper_robot/naoqi_driver/bumper") {
				gpp::Value *param =new gpp::Value(gpp::NumberType::name(), int(msg->statePressed));
				gpp::vector< gpp::unique_ptr<gpp::Value> > args;
				args.emplace_back(param);
				gpp::ExogEvent* ev = new gpp::ExogEvent(exog, std::move(args));
			}
		}
	}
}

void RosBackend::init_exog_event()
{
	sub_exog_event<naoqi_bridge_msgs::Bumper, naoqi_bridge_msgs::Bumper::ConstPtr>(
		"/pepper_robot/naoqi_driver/bumper",
		bumperCallback,
		1000
	);
}

