#ifdef NAOQI_BRIDGE_MSGS_PKG
#include "exog_manager.h"
#include "ros_backend.h"
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/HeadTouch.h>



template <>
std::unordered_map< std::string, gpp::unique_ptr<gpp::Value> >
ExogManager<naoqi_bridge_msgs::Bumper>::params_to_map(const naoqi_bridge_msgs::Bumper::ConstPtr& msg) {

	gpp::unique_ptr<gpp::Value> param (new gpp::Value(gpp::get_type<gpp::BoolType>(), bool(msg->statePressed)));
	std::unordered_map< std::string, gpp::unique_ptr<gpp::Value> > params_to_map;
	params_to_map.insert({"pressed", std::move(param)});
	return params_to_map;
}

template <>
std::unordered_map< std::string, gpp::unique_ptr<gpp::Value> >
ExogManager<naoqi_bridge_msgs::HeadTouch>::params_to_map(const naoqi_bridge_msgs::HeadTouch::ConstPtr& msg) {

	gpp::unique_ptr<gpp::Value> param (new gpp::Value(gpp::get_type<gpp::BoolType>(), bool(msg->statePressed)));
	std::unordered_map< std::string, gpp::unique_ptr<gpp::Value> > params_to_map;
	params_to_map.insert({"pressed", std::move(param)});
	return params_to_map;
}

void RosBackend::define_naoqi_bridge_actions()
{
	create_ExogManger<naoqi_bridge_msgs::Bumper>(
		"/pepper_robot/naoqi_driver/bumper"
	);
	create_ExogManger<naoqi_bridge_msgs::HeadTouch>(
		"/pepper_robot/naoqi_driver/head_touch"
	);
}


#endif //NAOQI_BRIDGE_MSGS_PKG

