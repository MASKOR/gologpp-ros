#ifndef GOLOGPP_AGENT_ACTION_MANAGER_H_
#define GOLOGPP_AGENT_ACTION_MANAGER_H_

#include <model/transition.h>
#include <model/platform_backend.h>

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

	virtual void execute_current_activity() = 0;

protected:
	gpp::shared_ptr<gpp::Activity> current_activity;
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

	// Specialized for every action type in e.g. pepper_actions.cpp
	GoalT build_goal(const gpp::Activity &);

	void doneCb(const actionlib::SimpleClientGoalState &state, ResultT result);
	gpp::Value *to_golog_constant(ResultT);

private:
	ClientT action_client_;
	GoalT current_goal_;
};


template<class ServiceT>
class ServiceManager : public AbstractActionManager {
	// TODO
};

template<class ActionT>
ActionManager<ActionT>::ActionManager(const std::string &topic_name, RosBackend &backend)
: AbstractActionManager(backend)
, action_client_(topic_name, true)
{}


template<class ActionT>
void ActionManager<ActionT>::execute_current_activity() {
	current_goal_(build_goal(current_activity));
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
		backend.update_activity(
		current_activity->transition(gpp::Transition::Hook::FINISH),
		to_golog_constant(result)
		);
		break;
	case actionlib::SimpleClientGoalState::PREEMPTED:
		// TODO
		break;
	case actionlib::SimpleClientGoalState::ABORTED:
		backend.update_activity(
		current_activity->transition(gpp::Transition::Hook::FAIL),
		nullptr
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
gpp::Value *ActionManager<ActionT>::to_golog_constant(ActionManager<ActionT>::ResultT)
{
	return nullptr;
}

template<class ActionT>
void RosBackend::define_action_client(const std::string &topic_name)
{
	// TODO: Create ActionContainer<ActionT> and put in action_containers_
	action_managers_.emplace(
	topic_name,
	std::unique_ptr<AbstractActionManager>(new ActionManager<ActionT>(topic_name, *this))
	);
}

#endif