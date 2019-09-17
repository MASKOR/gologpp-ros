#ifdef NAOQI_WRAPPER_MSGS_PKG
#include "action_manager.h"
#include "exog_manager.h"
#include "ros_backend.h"

#include <model/execution.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <naoqi_wrapper_msgs/NaoQi_animatedSayAction.h>
#include <naoqi_wrapper_msgs/NaoQi_animationAction.h>
#include <naoqi_wrapper_msgs/NaoQi_dialogAction.h>
#include <naoqi_wrapper_msgs/FaceTracking.h>
#include <naoqi_wrapper_msgs/NaoQi_lookAtAction.h>
#include <naoqi_wrapper_msgs/NaoQi_openWebsiteAction.h>
#include <naoqi_wrapper_msgs/NaoQi_sayAction.h>
#include <naoqi_wrapper_msgs/NaoQi_subscribeAction.h>

#include <naoqi_wrapper_msgs/FaceTracking.h>

namespace naoqi = naoqi_wrapper_msgs;

template<>
gpp::Value *
ActionManager<naoqi::NaoQi_dialogAction>::to_golog_constant(ResultT result){
	return new gpp::Value(gpp::StringType::name(), result->outcome);
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
ActionManager<naoqi::NaoQi_openWebsiteAction>::GoalT
ActionManager<naoqi::NaoQi_openWebsiteAction>::build_goal(const gpp::Activity &a)
{
	naoqi_wrapper_msgs::NaoQi_openWebsiteGoal goal;
	goal.url.data = std::string(a.mapped_arg_value("url"));
	goal.waitForWebCommand = bool(a.mapped_arg_value("waitForWebCommand"));
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

template<>
ServiceManager<naoqi::FaceTracking>::RequestT
ServiceManager<naoqi::FaceTracking>::build_request(const gpp::Activity &a)
{
	naoqi::FaceTrackingRequest req;
	req.enableFaceTracking = bool(a.mapped_arg_value("enable"));
	return req;
}

void RosBackend::define_naoqi_wrapper_actions()
{
	create_ActionManager<naoqi::NaoQi_dialogAction>("/naoqi_dialog_server");
	create_ActionManager<naoqi::NaoQi_sayAction>("/naoqi_say_server/naoqi_say");
	create_ActionManager<naoqi::NaoQi_lookAtAction>("/naoqi_lookAt_server/lookAt");
	create_ActionManager<naoqi::NaoQi_openWebsiteAction>("/naoqi_openWebsite_server/openWebsite");
	create_ActionManager<naoqi::NaoQi_animatedSayAction>("/naoqi_animatedSay_server/animatedSay");
	create_ActionManager<naoqi::NaoQi_animationAction>("/naoqi_animation_server/naoqi_animation");
	create_ActionManager<naoqi::NaoQi_subscribeAction>("/naoqi_subscribe_server/subscribe");
}
#endif // NAOQI_WRAPPER_MSGS_PKG
