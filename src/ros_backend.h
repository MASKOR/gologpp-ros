#ifndef ROSBACKEND_H
#define ROSBACKEND_H

// Remove spurious clang code model error
#ifdef Q_CREATOR_RUN
#undef __GCC_ASM_FLAG_OUTPUTS__
#endif

#include <model/transition.h>
#include <model/platform_backend.h>

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

private:
	// Implemented in pepper_actions.cpp:
	// Fill action_containers_ by calling define_action_client for
	// each action that should be made available.
	void define_actions();

	template<class ActionT>
	void define_action_client(const std::string &name);

	AbstractActionManager &get_action_client(gpp::shared_ptr<gpp::Activity>);

	ros::NodeHandle nh_;

	std::unordered_map<
		std::string,
		std::unique_ptr<AbstractActionManager>
	> action_managers_;
};


template<class ActionT>
void RosBackend::define_action_client(const std::string &name)
{
	// TODO: Create ActionContainer<ActionT> and put in action_containers_
}


#endif // ROSBACKEND_H
