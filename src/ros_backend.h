#ifndef ROSBACKEND_H
#define ROSBACKEND_H

// Remove spurious clang code model error
#ifdef Q_CREATOR_RUN
#undef __GCC_ASM_FLAG_OUTPUTS__
#endif

#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <execution/transition.h>
#include <execution/platform_backend.h>

#include <semantics/readylog/execution.h>

#include <ros/ros.h>

#include <unordered_map>


class AbstractActionManager;
class AbstractExogManager;

class RosBackend : public gologpp::PlatformBackend
{
private:
	using string = gologpp::string;
	using Value = gologpp::Value;
	using Type = gologpp::Type;
	using Activity = gologpp::Activity;
	using Clock = gologpp::Clock;
	template<class T> using shared_ptr = gologpp::shared_ptr<T>;

public:
	RosBackend();
	virtual ~RosBackend() override;
	virtual void execute_activity(shared_ptr<Activity> a) override;
	virtual void preempt_activity(shared_ptr<Activity> trans) override;
	virtual Clock::time_point time() const noexcept override;

	virtual Value eval_exog_function(
		const Type &return_type,
		const string &backend_name,
		const std::unordered_map<string, Value> &args
	) override;

	//std::mutex exog_mutex;
	std::atomic<bool> ctx_ready;

private:
	virtual void terminate_() override;

	// May have an implementation iff the corresponding package has been found
	void define_naoqi_wrapper_actions();
	void define_move_base_actions();
	void define_darknet_actions();
	void define_naoqi_bridge_actions();
	void define_opencv_apps_actions();
	void define_turtle_actions();

	template<class ActionT>
	void create_ActionManager(const std::string &name);

	AbstractActionManager &get_ActionManager(shared_ptr<Activity>);

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
};

#endif // ROSBACKEND_H
