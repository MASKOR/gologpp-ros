#include "action_manager.h"
#include "ros_backend.h"

AbstractActionManager::AbstractActionManager(RosBackend &backend)
: backend(backend)
{}


void AbstractActionManager::execute(gpp::shared_ptr<gpp::Activity>a) {
	// TODO: Set current_activity_, call execute_current_activity()
	current_activity = a;
	execute_current_activity();
}

void AbstractActionManager::preempt(gpp::shared_ptr<gpp::Activity>a) {
	current_activity = a;
	preempt_current_activity();
}

