#ifndef GOLOGPP_AGENT_EXOG_MANAGER_H_
#define GOLOGPP_AGENT_EXOG_MANAGER_H_

#include "ros_backend.h"

class RosBackend;

template<class M, class C>
void RosBackend::sub_exog_event(
	const std::string &topic,
	const boost::function< void(C)> & callback,
	int msgs_queue_size
) {
	exog_subs_.push_back(nh_.subscribe<M>(topic, msgs_queue_size, callback));
}

#endif // EXOGMANAGER_H

