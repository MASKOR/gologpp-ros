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
	// using GoalT = typename ActionT::_action_goal_type::_goal_type;
	// using ResultT = typename ActionT::_action_result_type::_result_type::ConstPtr;
	// using ClientT = typename actionlib::SimpleActionClient<ActionT>;
	using ActionName = typename ActionT::_action_type::Name;
	// Goal Handle
	using ResultT = typename ActionT::_action_result_type::_result_type::ConstPtr;
	using GoalT =  typename rclcpp_action::ClientGoalHandle<ActionName>;
	using ClientT = typename rclcpp_action::Client<ActionT>::SharedPtr;

	ActionManager(const std::string &, RosBackend &backend);

	virtual void execute_current_activity() override;
	virtual void preempt_current_activity() override;

	// Specialized for every action type in e.g. pepper_actions.cpp
	GoalT build_goal(const gpp::Activity &);

	// Result callback
	void doneCb(const typename GoalT::WrappedResult &result);
	gpp::optional<gpp::Value> to_golog_constant(ResultT);

private:
	ClientT action_client_;
	GoalT current_goal_;
};


template<class ServiceT>
class ServiceManager : public AbstractActionManager {
	// TODO
public:
	using RequestT = typename ServiceT::Request;
	using ResponseT = typename ServiceT::Response;
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
	//ros::NodeHandle nh("~");
	//service_client_ = nh.serviceClient<ServiceT>(topic_name);
	auto agent_node = Singleton::instance();
	service_client_ = agent_node->create_client<ServiceT>(topic_name);
}


template<class ActionT>
ActionManager<ActionT>::ActionManager(const std::string &topic_name, RosBackend &backend)
: AbstractActionManager(backend)
, action_client_(topic_name, true)
{}


template<class ServiceT>
void ServiceManager<ServiceT>::execute_current_activity() {
	current_request_ = build_request(*current_activity_);

	std::thread service_thread( [&] (
	RequestT current_request,
	ResponseT current_response,
	std::shared_ptr<gpp::Activity> current_activity
	) {
		if(service_client_.call(current_request, current_response)) {
			current_activity->update(gpp::Transition::Hook::FINISH);
			set_result(to_golog_constant(current_response));
		} else {
			current_activity->update(gpp::Transition::Hook::FAIL);
			set_result(to_golog_constant(current_response));
		}
	},current_request_, current_response_, current_activity_);
	service_thread.detach();
}


template<class ServiceT>
void ServiceManager<ServiceT>::preempt_current_activity() {}

template<class ActionT>
void ActionManager<ActionT>::preempt_current_activity()
{ action_client_.cancelGoal(); }


template<class ActionT>
void ActionManager<ActionT>::execute_current_activity()
{
	current_goal_ = build_goal(*current_activity_);
	action_client_.sendGoal(current_goal_, std::bind(
		&ActionManager<ActionT>::doneCb,
		this, _1, _2
	) );
}


template<class ActionT>
void ActionManager<ActionT>::doneCb(const typename GoalT::WrappedResult  &result) {
	ROS_INFO("Finished in state [%s]", result.toString().c_str());
	switch(result.code) {
	case rclcpp_action::ResultCode::SUCCEEDED:
		current_activity_->update(gpp::Transition::Hook::FINISH);
		set_result(to_golog_constant(result.result));
		break;
	case rclcpp_action::ResultCode::ABORTED:
		current_activity_->update(gpp::Transition::Hook::FAIL);
		set_result(to_golog_constant(result.result));
		break;
	case rclcpp_action::ResultCode::CANCELED:
	default:
		RCLCPP_ERROR(this->get_logger(), "Unknown result code")
		;
	}
}


template<class ActionT>
gpp::optional<gpp::Value> ActionManager<ActionT>::to_golog_constant(ActionManager<ActionT>::ResultT)
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
