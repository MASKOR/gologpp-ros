#ifndef ROSBACKEND_H
#define ROSBACKEND_H

// Remove spurious clang code model error
#ifdef Q_CREATOR_RUN
#undef __GCC_ASM_FLAG_OUTPUTS__
#endif

#include <model/transition.h>
#include <model/platform_backend.h>

#include <semantics/readylog/execution.h>

#include <ros/ros.h>

#include <unordered_map>

namespace gpp = gologpp;


class AbstractActionManager;


class RosBackend : public gpp::PlatformBackend
{
public:
	RosBackend();
	virtual ~RosBackend() override;
	virtual void execute_activity(gpp::shared_ptr<gpp::Activity> a) override;
	virtual void preempt_activity(gpp::shared_ptr<gpp::Transition> trans) override;
	virtual gpp::Clock::time_point time() const noexcept override;

	static void exog_event_to_queue(const std::string &map_name, gpp::unique_ptr<gpp::Value> param);

private:
	// Implemented in pepper_actions.cpp:
	// Fill action_containers_ by calling define_action_client for
	// each action that should be made available.
	void define_actions();

	template<class ActionT>
	void define_action_client(const std::string &name);

	AbstractActionManager &get_action_client(gpp::shared_ptr<gpp::Activity>);

	void init_exog_event();
	// M: msgs type; C: callback parameter type
	template<class M>
	void sub_exog_event(
		const std::string &,
		const boost::function< void(typename M::ConstPtr)> &,
		int msgs_queue_size = 1000
	);

	void spin_exog_thread();

	ros::NodeHandle nh_;
	std::vector<ros::Subscriber> exog_subs_;

	std::unordered_map<
		std::string,
		std::unique_ptr<AbstractActionManager>
	> action_managers_;
};





#endif // ROSBACKEND_H
