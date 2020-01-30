#include "exog_manager.h"
#include "action_manager.h"
#include "ros_backend.h"

#include <model/execution.h>

#include <opencv_apps/FaceArrayStamped.h>
#include <opencv_apps_action_msgs/LearnFaceAction.h>

template <>
std::unordered_map< std::string, gpp::unique_ptr<gpp::Value> >
ExogManager<opencv_apps::FaceArrayStamped>::params_to_map(const opencv_apps::FaceArrayStamped::ConstPtr& msg) {

	gpp::unique_ptr<gpp::Value> param_label (new gpp::Value(gpp::get_type<gpp::StringType>(), std::string(msg->faces[0].label)));
	gpp::unique_ptr<gpp::Value> param_confidence (new gpp::Value(gpp::get_type<gpp::NumberType>(), int(msg->faces[0].confidence)));
	std::unordered_map< std::string, gpp::unique_ptr<gpp::Value> > params_to_map;
	params_to_map.insert({"name", std::move(param_label)});
	params_to_map.insert({"confidence", std::move(param_confidence)});
	return params_to_map;
}

template<>
ActionManager<opencv_apps_action_msgs::LearnFaceAction>::GoalT
ActionManager<opencv_apps_action_msgs::LearnFaceAction>::build_goal(const gpp::Activity &a)
{
	opencv_apps_action_msgs::LearnFaceGoal goal;
	goal.name = std::string(a.mapped_arg_value("learn_name"));
	return goal;
}

void RosBackend::define_opencv_apps_actions()
{
	create_ActionManager<opencv_apps_action_msgs::LearnFaceAction>(
		"/face_recognition_trainer_actionserver"
	);
	create_ExogManger<opencv_apps::FaceArrayStamped>(
		"/face_recognition/output"
	);
}

