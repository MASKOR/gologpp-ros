#ifndef GOLOGPP_AGENT_ACTION_MANAGER_H_
#define GOLOGPP_AGENT_ACTION_MANAGER_H_

// Remove spurious clang code model error
#ifdef Q_CREATOR_RUN
#undef __GCC_ASM_FLAG_OUTPUTS__
#endif

#include "ros_backend.h"

#include <execution/transition.h>
#include <execution/platform_backend.h>
#include <execution/activity.h>

// Add ros2 action
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::placeholders;

namespace gpp = gologpp;


class AbstractActionManager {
public:
	AbstractActionManager(RosBackend &backend);
	virtual ~AbstractActionManager() = default;

	/* Set current_activity_, call execute_current_activity()
	 * */
	void execute(gpp::shared_ptr<gpp::Activity>);
	void preempt(gpp::shared_ptr<gpp::Activity>);

	virtual void execute_current_activity() = 0;
	virtual void preempt_current_activity() = 0;

	gpp::optional<gpp::Value> result();
	void set_result(gpp::optional<gpp::Value> &&v);

	gpp::shared_ptr<gpp::Activity> current_activity();

protected:
	RosBackend &backend_;
	gpp::shared_ptr<gpp::Activity> current_activity_;
	gpp::optional<gpp::Value> result_;
};


template<class ActionT>
class ActionManager : public AbstractActionManager {
public:

	// Goal Handle and client
	using GoalT =  typename ActionT::Goal;
	using ResultT = typename rclcpp_action::ClientGoalHandle<ActionT>;
	using ClientT = typename rclcpp_action::Client<ActionT>::SharedPtr;

	ActionManager(const std::string &, RosBackend &backend);

	virtual void execute_current_activity() override;
	virtual void preempt_current_activity() override;

	// Specialized for every action type in e.g. pepper_actions.cpp
	GoalT build_goal(const gpp::Activity &);

	// Result callback to transit action output to gpp agent
	void result_callback(const typename ResultT::WrappedResult &result);
	// ResultT::WrappedResult should be enough but rclp_action also before ::ResultT
	gpp::optional<gpp::Value> to_golog_constant(typename ResultT::WrappedResult);

private:
	ClientT action_client_;
	GoalT current_goal_;
};


template<class ServiceT>
class ServiceManager : public AbstractActionManager {
	// TODO
public:
	using RequestT = typename ServiceT::Request::SharedPtr;
	using ResponseT = typename rclcpp::Client<ServiceT>::SharedFuture;
	using Client = typename rclcpp::Client<ServiceT>::SharedPtr;

	virtual void execute_current_activity() override;
	virtual void preempt_current_activity() override;

	ServiceManager(const std::string &, RosBackend &backend);

	RequestT build_request(const gpp::Activity&);
	gpp::optional<gpp::Value> to_golog_constant(ResponseT);

private:
	Client service_client_;
	RequestT current_request_;
	ResponseT current_response_;
};


template<class ServiceT>
ServiceManager<ServiceT>::ServiceManager(const std::string &topic_name, RosBackend &backend)
: AbstractActionManager (backend)
{
	auto agent_node = Singleton::instance();
	service_client_ = agent_node->create_client<ServiceT>(topic_name);
}


template<class ActionT>
ActionManager<ActionT>::ActionManager(const std::string &topic_name, RosBackend &backend)
: AbstractActionManager(backend)
{
	auto agent_node = Singleton::instance();
	action_client_ = rclcpp_action::create_client<ActionT>(agent_node, topic_name);

}


template<class ServiceT>
void ServiceManager<ServiceT>::execute_current_activity() {
	current_request_ = build_request(*current_activity_);

	while (!service_client_->wait_for_service(std::chrono::seconds(1))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
	}

	auto response_received_callback = [this](ResponseT future_response_) {
        if (future_response_.valid()) {
			current_activity_->update(gpp::Transition::Hook::FINISH);
			set_result(to_golog_constant(future_response_));
		} else {
			current_activity_->update(gpp::Transition::Hook::FAIL);
			set_result(to_golog_constant(future_response_));
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
		}
	};
	auto future_result = service_client_->async_send_request(current_request_, response_received_callback);
}


template<class ServiceT>
void ServiceManager<ServiceT>::preempt_current_activity() {}

template<class ActionT>
void ActionManager<ActionT>::preempt_current_activity()
{
	//Cancel goal
	//action_client_.cancelGoal();
}


template<class ActionT>
void ActionManager<ActionT>::execute_current_activity()
{
	current_goal_ = build_goal(*current_activity_);
	//ClientT is shared ptr otherwise useable for SendGoalOption
	auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();

    send_goal_options.result_callback =
      std::bind(&ActionManager<ActionT>::result_callback, this, _1);
	action_client_->async_send_goal(current_goal_, send_goal_options);
}


template<class ActionT>
void ActionManager<ActionT>::result_callback(const typename ResultT::WrappedResult &result) {
	switch(result.code) {
	case rclcpp_action::ResultCode::SUCCEEDED:
		current_activity_->update(gpp::Transition::Hook::FINISH);
		set_result(to_golog_constant(result));
		break;
	case rclcpp_action::ResultCode::ABORTED:
		current_activity_->update(gpp::Transition::Hook::FAIL);
		set_result(to_golog_constant(result));
		break;
	case rclcpp_action::ResultCode::CANCELED:
	default:
		RCLCPP_ERROR(Singleton::instance()->get_logger(), "Unknown result code")
		;
	}
}


template<class ActionT>
gpp::optional<gpp::Value> ActionManager<ActionT>::to_golog_constant(typename ResultT::WrappedResult)
{
	return gpp::nullopt;
}

template<class ServiceT>
gpp::optional<gpp::Value> ServiceManager<ServiceT>::to_golog_constant(ServiceManager<ServiceT>::ResponseT)
{
	return gpp::nullopt;
}

template<class ActionT>
void RosBackend::create_ActionManager(const std::string &topic_name)
{
	// TODO: Create ActionContainer<ActionT> and put in action_containers_
	action_managers_.emplace(
		topic_name,
		std::unique_ptr<AbstractActionManager>(new ActionManager<ActionT>(topic_name, *this))
	);
}

template<class ServiceT>
void RosBackend::create_ServiceManager(const std::string &topic_name)
{
	action_managers_.emplace(
		topic_name,
		std::unique_ptr<AbstractActionManager>(new ServiceManager<ServiceT>(topic_name, *this))
	);
}

#endif
