
#include "ros_backend.h"
#include "action_manager.h"
#include "exog_manager.h"



RosBackend::RosBackend()
{
	this->ctx_ready = false;
#ifdef naoqi_wrapper_msgs_FOUND
	define_naoqi_wrapper_actions();
#endif

#ifdef move_base_msgs_FOUND
	define_move_base_actions();
#endif

#ifdef darknet_action_msgs_FOUND
	define_darknet_actions();
#endif

#ifdef naoqi_bridge_msgs_FOUND
	define_naoqi_bridge_actions();
#endif

#ifdef opencv_apps_msgs_FOUND
	define_opencv_apps_actions();
#endif

#ifdef turtle_actionlib_FOUND
	define_turtle_actions();
#endif
	spin_exog_thread();
}


RosBackend::~RosBackend()
{}



void RosBackend::preempt_activity(shared_ptr<Activity> trans)
{
	// TODO: Tell ActionManager to preempt
	//gpp::shared_ptr<gpp::Activity> a = std::make_shared<gpp::Activity>(trans);
	//get_ActionManager(a).preempt(a);
}


void RosBackend::execute_activity(shared_ptr<Activity> a)
{
	// TODO: Find AbstractActionManager for a->mapped_name()
	//       and execute it, e.g.:
	//       action_manager.execute(a);
	get_ActionManager(a).execute(a);
}


AbstractActionManager& RosBackend::get_ActionManager(shared_ptr<Activity> a)
{
	return *action_managers_.find(std::string(a->mapped_name()))->second;
}


void RosBackend::spin_exog_thread()
{
	std::thread spin_thread( [&] () {
				//ros::spin();
				auto agent_node = Singleton::instance();
				rclcpp::spin(agent_node);
			});
	spin_thread.detach();
}


gpp::Clock::time_point RosBackend::time() const noexcept
{
	Clock::duration rv = std::chrono::steady_clock::now().time_since_epoch();
	return Clock::time_point(rv);
}


gpp::Value RosBackend::eval_exog_function(
	const Type &return_type,
	const string &backend_name,
	const std::unordered_map<string, Value> &args
) {
	if (backend_name == "sense_result") {
		string act_name = static_cast<string>(args.at("ros_action_name"));

		AbstractActionManager &act_mgr = *action_managers_.at(act_name);

		if (!act_mgr.current_activity()->target()->senses())
			throw gologpp::UserError(
				backend_name + ": " + act_mgr.current_activity()->target()->str()
				+ " is not a sensing action"
			);

		auto opt_result = act_mgr.result();

		if (!opt_result)
			throw gologpp::UserError(
				backend_name + ": " + act_mgr.current_activity()->str()
				+ " has not provided a sensing result"
			);

		if (opt_result.value().type() <= return_type)
			return opt_result.value();
		else
			throw gologpp::TypeError(opt_result.value(), return_type);
	}
	else
		throw gologpp::UserError(
			"No exog_function '" + backend_name + "'. "
			"Only 'sense_result(ros_action_name, ros_action_args...)' is currently supported"
		);
}

void RosBackend::terminate_()
{

}

