#include "ros_backend.h"

template<class M, class C>
void RosBackend::sub_exog_event(
	const std::string &topic,
	const boost::function< void(C)> & callback,
	int msgs_queue_size
) {
	exog_subs_.push_back(nh_.subscribe<M>(topic, msgs_queue_size, &callback));
}

void RosBackend::spin_exog_thread()
{
	std::thread spin_thread( [&] () {

				ros::spin();
			});
	spin_thread.detach();
}

