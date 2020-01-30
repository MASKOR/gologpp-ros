#ifndef GOLOGPP_AGENT_ACTION_MANAGER_H_
#define GOLOGPP_AGENT_ACTION_MANAGER_H_

// Remove spurious clang code model error
#ifdef Q_CREATOR_RUN
#undef __GCC_ASM_FLAG_OUTPUTS__
#endif

#include <model/transition.h>
#include <model/platform_backend.h>
#include <model/activity.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "ros_backend.h"


namespace gpp = gologpp;

class RosBackend;


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

protected:
	gpp::shared_ptr<gpp::Activity> current_activity_;
	RosBackend &backend;
};


template<class ActionT>
class ActionManager : public AbstractActionManager {
public:
	using GoalT = typename ActionT::_action_goal_type::_goal_type;
	using ResultT = typename ActionT::_action_result_type::_result_type::ConstPtr;
	using ClientT = typename actionlib::SimpleActionClient<ActionT>;

	ActionManager(const std::string &, RosBackend &backend);

	virtual void execute_current_activity() override;
	virtual void preempt_current_activity() override;

	// Specialized for every action type in e.g. pepper_actions.cpp
	GoalT build_goal(const gpp::Activity &);

	void doneCb(const actionlib::SimpleClientGoalState &state, ResultT result);
	boost::optional<gpp::Value> to_golog_constant(ResultT);

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
	using Client = ros::ServiceClient;

	virtual void execute_current_activity() override;
	virtual void preempt_current_activity() override;

	ServiceManager(const std::string &, RosBackend &backend);

	RequestT build_request(const gpp::Activity&);
	boost::optional<gpp::Value> to_golog_constant(ResponseT);

private:
	Client service_client_;
	RequestT current_request_;
	ResponseT current_response_;
};


template<class ServiceT>
ServiceManager<ServiceT>::ServiceManager(const std::string &topic_name, RosBackend &backend)
: AbstractActionManager (backend)
{
	ros::NodeHandle nh("~");
	service_client_ = nh.serviceClient<ServiceT>(topic_name);
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
			current_activity->update(
				(gpp::Transition::Hook::FINISH),
				to_golog_constant(current_response)
			);
		} else {
			current_activity->update(
				(gpp::Transition::Hook::FAIL),
				to_golog_constant(current_response)
			);
		}
	},current_request_, current_response_, current_activity_);
	service_thread.detach();
}

template<class ServiceT>
void ServiceManager<ServiceT>::preempt_current_activity() {}

template<class ActionT>
void ActionManager<ActionT>::preempt_current_activity() {
	action_client_.cancelGoal();
}

template<class ActionT>
void ActionManager<ActionT>::execute_current_activity() {
	current_goal_ = build_goal(*current_activity_);
	action_client_.sendGoal(current_goal_, boost::bind(
	&ActionManager<ActionT>::doneCb,
	this, _1, _2
	) );
}


template<class ActionT>
void ActionManager<ActionT>::doneCb(const actionlib::SimpleClientGoalState &state, ResultT result) {
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	switch(state.state_) {
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		current_activity_->update(
			gpp::Transition::Hook::FINISH,
			std::move(to_golog_constant(result))
		);
		break;
	case actionlib::SimpleClientGoalState::PREEMPTED:
		// TODO
		break;
	case actionlib::SimpleClientGoalState::ABORTED:
		current_activity_->update(
			(gpp::Transition::Hook::FAIL),
			std::move(to_golog_constant(result))
		);
		break;
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::ACTIVE:
	case actionlib::SimpleClientGoalState::RECALLED:
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::LOST:
		;
	}
}


template<class ActionT>
boost::optional<gpp::Value> ActionManager<ActionT>::to_golog_constant(ActionManager<ActionT>::ResultT)
{
	return boost::optional<gpp::Value>();
}

template<class ServiceT>
boost::optional<gpp::Value> ServiceManager<ServiceT>::to_golog_constant(ServiceManager<ServiceT>::ResponseT)
{
	return boost::optional<gpp::Value>();
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
