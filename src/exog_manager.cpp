#include "exog_manager.h"


void RosBackend::spin_exog_thread()
{
	std::thread spin_thread( [&] () {

				ros::spin();
			});
	spin_thread.detach();
}

