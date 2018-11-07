#ifndef ROSBACKEND_H
#define ROSBACKEND_H

#include <model/execution.h>
// Remove spurious clang code model error
#ifdef Q_CREATOR_RUN
#undef __GCC_ASM_FLAG_OUTPUTS__
#endif

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <pepper_smach/NaoQi_animatedSayAction.h>

#include <ros/ros.h>

#include <tuple>


namespace gologpp {

class Pepper_Backend : public AExecutionBackend
{
public:
	Pepper_Backend();
	virtual ~Pepper_Backend() override;
	virtual void execute_transition(shared_ptr <Transition> trans)  override;

	template < typename ActionT > void execute_transition_wrapper(typename ActionT::_action_goal_type::_goal_type &goal, shared_ptr <Transition> trans) {
		actionlib::SimpleActionClient < ActionT > &client = std::get < actionlib::SimpleActionClient < ActionT > &>(action_clients);
		if (trans->action().blocking()) {
			client.sendGoalAndWait(goal, ros::Duration(30.0));
			trans->state = Action::RUNNING;
			auto state = client.getState();
			ROS_INFO ("Action finished: %s", state.toString().c_str());

		} else {
			client.sendGoal(goal, boost::bind(
				&Pepper_Backend::doneCb<typename ActionT::_action_result_type::_result_type::ConstPtr>,
				this, trans, _1, _2
			) );
			trans->state = Action::RUNNING;
			auto state = client.getState();
		}
	}

	template < typename ResultConstPtrT > void doneCb(shared_ptr <Transition> &trans, const actionlib::SimpleClientGoalState &state, ResultConstPtrT result) {

		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		running_transitions_.erase(trans);
		switch(state.state_){
			case actionlib::SimpleClientGoalState::SUCCEEDED: trans->state = Action::SUCCEEDED; break;
			case actionlib::SimpleClientGoalState::PREEMPTED: trans->state = Action::PREEMPTED; break;
			case actionlib::SimpleClientGoalState::ABORTED:   trans->state = Action::ABORTED; break;
			case actionlib::SimpleClientGoalState::PENDING: ; break;
			case actionlib::SimpleClientGoalState::ACTIVE: ; break;
			case actionlib::SimpleClientGoalState::RECALLED: ; break;
			case actionlib::SimpleClientGoalState::REJECTED: ; break;
			case actionlib::SimpleClientGoalState::LOST: ; break;
		}
		done_transitions_.push(trans);
		//trans.action().ser;
		//ROS_INFO("Finished in state [%s]", action_state_);
	}


private:
	actionlib::SimpleActionClient<pepper_smach::NaoQi_animatedSayAction> animated_say_client;

	std::tuple <
		actionlib::SimpleActionClient<pepper_smach::NaoQi_animatedSayAction> &
	> action_clients;


	//std::unordered_map<actionlib::SimpleActionClient<actionlib_test::DoPutAction>,Transition> action_map;
};
} //namespace gologpp
#endif // ROSBACKEND_H
