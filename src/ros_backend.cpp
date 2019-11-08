#include "action_manager.h"
#include "exog_manager.h"
#include "ros_backend.h"


RosBackend::RosBackend()
{
#ifdef NAOQI_WRAPPER_MSGS_PKG
	define_naoqi_wrapper_actions();
#endif

#ifdef MOVE_BASE_MSGS_PKG
	define_move_base_actions();
#endif

#ifdef DARKNET_ACTION_MSGS_PKG
	define_darknet_actions();
#endif

#ifdef NAOQI_BRIDGE_MSGS_PKG
	define_naoqi_bridge_actions();
#endif

	spin_exog_thread();
}


RosBackend::~RosBackend()
{}



void RosBackend::preempt_activity(gpp::shared_ptr<gpp::Transition> trans)
{
	// TODO: Tell ActionManager to preempt
	gpp::shared_ptr<gpp::Activity> a = std::make_shared<gpp::Activity>(trans);
	get_ActionManager(a).preempt(a);
}


void RosBackend::execute_activity(gpp::shared_ptr<gpp::Activity> a)
{
	// TODO: Find AbstractActionManager for a->mapped_name()
	//       and execute it, e.g.:
	//       action_manager.execute(a);
	get_ActionManager(a).execute(a);
}

AbstractActionManager& RosBackend::get_ActionManager(gpp::shared_ptr<gpp::Activity> a)
{
	return *action_managers_.find(std::string(a->mapped_name()))->second;
}

void RosBackend::spin_exog_thread()
{
	std::thread spin_thread( [&] () {
				ros::spin();
			});
	spin_thread.detach();
}

gpp::Clock::time_point RosBackend::time() const noexcept
{
	gpp::Clock::duration rv = std::chrono::steady_clock::now().time_since_epoch();
	return gpp::Clock::time_point(rv);
}

