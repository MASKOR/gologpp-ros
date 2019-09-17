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
class AbstractExogManager;

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
	void define_naoqi_wrapper_actions();
	void define_move_base_actions();
	void define_darknet_actions();
	void init_naoqi_bridge_exog();

	//create actionManager
	template<class ActionT>
	void create_ActionManager(const std::string &name);

	AbstractActionManager &get_ActionManager(gpp::shared_ptr<gpp::Activity>);

	// create exogManager
	template<class ExogT>
	void create_ExogManger(
		const std::string &
	);

	void spin_exog_thread();

	template<class ServiceT>
	void create_ServiceManager(
		const std::string &
	);

	std::vector<
		std::unique_ptr<AbstractExogManager>
	> exog_managers_;

	std::unordered_map<
		std::string,
		std::unique_ptr<AbstractActionManager>
	> action_managers_;

	std::unordered_map<
		std::string,
		std::unique_ptr<AbstractActionManager>
	> service_managers_;
};

#endif // ROSBACKEND_H
