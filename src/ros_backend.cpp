#include "ros_backend.h"
#include "action_manager.h"


RosBackend::RosBackend()
{
	define_actions();
	init_exog_event();
	spin_exog_thread();
}


RosBackend::~RosBackend()
{}



void RosBackend::preempt_activity(gpp::shared_ptr<gpp::Transition> trans)
{
	// TODO: Tell ActionManager to preempt
	gpp::shared_ptr<gpp::Activity> a = std::make_shared<gpp::Activity>(trans);
	get_action_client(a).preempt(a);
}


void RosBackend::execute_activity(gpp::shared_ptr<gpp::Activity> a)
{
	// TODO: Find AbstractActionManager for a->mapped_name()
	//       and execute it, e.g.:
	//       action_manager.execute(a);
	get_action_client(a).execute(a);
}

AbstractActionManager& RosBackend::get_action_client(gpp::shared_ptr<gpp::Activity> a)
{
	return *action_managers_.find(std::string(a->mapped_name()))->second;
}

void RosBackend::exog_event_to_queue(const std::string &map_name, gpp::unique_ptr<gpp::Value> param)
{
	std::shared_ptr<gpp::ExogEvent> ev;
	for (std::vector<std::shared_ptr<gpp::Global>>::iterator it = gpp::global_scope().globals().begin();
			it != gpp::global_scope().globals().end(); ++it){
		if(gpp::shared_ptr< gpp::ExogAction> exog = std::dynamic_pointer_cast< gpp::ExogAction>(*it)){
			if(exog->mapping().backend_name() == map_name) {
				gpp::vector< gpp::unique_ptr<gpp::Value> > args;
				args.emplace_back(std::move(param));
				std::shared_ptr<gpp::ExogEvent> ev(new gpp::ExogEvent(exog, std::move(args)));
				break;
			}
		}
	}
	gpp::ReadylogContext &ctx = gpp::ReadylogContext::instance();
	ctx.exog_queue_push(ev);
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

