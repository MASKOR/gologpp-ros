#ifdef NAOQI_BRIDGE_MSGS_PKG
#include "exog_manager.h"
#include "ros_backend.h"
#include <naoqi_bridge_msgs/Bumper.h>



template<>
void
ExogManager<naoqi_bridge_msgs::Bumper>::topic_cb(const naoqi_bridge_msgs::Bumper::ConstPtr& msg)
{
	ROS_INFO_STREAM("I heard: " << bool(msg->statePressed));
	gpp::unique_ptr<gpp::Value> param (new gpp::Value(gpp::BoolType::name(), bool(msg->statePressed)));
	exog_event_to_queue(
		std::move(param)
	);
}

void RosBackend::init_naoqi_bridge_exog()
{
	create_exogManger<naoqi_bridge_msgs::Bumper>(
		"/pepper_robot/naoqi_driver/bumper"
	);
}


#endif //NAOQI_BRIDGE_MSGS_PKG

