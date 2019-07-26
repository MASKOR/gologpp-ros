#include "action_manager.h"
#include "ros_backend.h"

AbstractActionManager::AbstractActionManager(RosBackend &backend)
: backend(backend)
{}


void AbstractActionManager::execute(gpp::shared_ptr<gpp::Activity>) {
	// TODO: Set current_activity_, call execute_current_activity()
}


