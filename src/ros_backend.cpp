#include "ros_backend.h"
#include "action_manager.h"


RosBackend::RosBackend()
{
	define_actions();
}


RosBackend::~RosBackend()
{}



void RosBackend::preempt_activity(gpp::shared_ptr<gpp::Transition> trans)
{
	// TODO: Tell ActionManager to preempt
}


void RosBackend::execute_activity(gpp::shared_ptr<gpp::Activity> a)
{
	// TODO: Find AbstractActionManager for a->mapped_name()
	//       and execute it, e.g.:
	//       action_manager.execute(a);
}


gpp::Clock::time_point RosBackend::time() const noexcept
{
	gpp::Clock::duration rv = std::chrono::steady_clock::now().time_since_epoch();
	return gpp::Clock::time_point(rv);
}
