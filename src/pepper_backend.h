#ifndef ROSBACKEND_H
#define ROSBACKEND_H

#include <model/execution.h>
#include <model/platform_backend.h>
// Remove spurious clang code model error
#ifdef Q_CREATOR_RUN
#undef __GCC_ASM_FLAG_OUTPUTS__
#endif

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <naoqi_actions/NaoQi_animatedSayAction.h>

#include <ros/ros.h>

#include <tuple>


namespace gologpp {

class Pepper_Backend : public PlatformBackend
{
public:
	Pepper_Backend();
	virtual ~Pepper_Backend() override;
	virtual void execute_activity(shared_ptr<Activity> a)  override;
	virtual void preempt_activity(shared_ptr<Transition> trans) override;

	template < typename ActionT > void execute_transition_wrapper(typename ActionT::_action_goal_type::_goal_type &goal, shared_ptr<Activity> a) {

		actionlib::SimpleActionClient < ActionT > &client = std::get < actionlib::SimpleActionClient < ActionT > &>(action_clients);
		if (true) {
			client.sendGoalAndWait(goal, ros::Duration(30.0));
			ROS_INFO("Sending goal");
			auto state = client.getState();
			ROS_INFO ("Action finished: %s", state.toString().c_str());

		} else {
			client.sendGoal(goal, boost::bind(
				&Pepper_Backend::doneCb<typename ActionT::_action_result_type::_result_type::ConstPtr>,
				this, a, _1, _2
			) );
		}
	}

	template < typename ResultConstPtrT > void doneCb(shared_ptr<Activity> a, const actionlib::SimpleClientGoalState &state, ResultConstPtrT result) {

		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		switch(state.state_){
			case actionlib::SimpleClientGoalState::SUCCEEDED:  update_activity(a->transition(Transition::Hook::FINISH)) ; break;
			case actionlib::SimpleClientGoalState::PREEMPTED:; break;
			case actionlib::SimpleClientGoalState::ABORTED:    update_activity(a->transition(Transition::Hook::FAIL)); break;
			case actionlib::SimpleClientGoalState::PENDING:	 ; break;
			case actionlib::SimpleClientGoalState::ACTIVE:   ; break;
			case actionlib::SimpleClientGoalState::RECALLED: ; break;
			case actionlib::SimpleClientGoalState::REJECTED: ; break;
			case actionlib::SimpleClientGoalState::LOST: ; break;
		}
	}


private:
	actionlib::SimpleActionClient<naoqi_actions::NaoQi_animatedSayAction> animated_say_client;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;

	std::tuple <
		actionlib::SimpleActionClient<naoqi_actions::NaoQi_animatedSayAction> & ,
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &
	> action_clients;


	//std::unordered_map<actionlib::SimpleActionClient<actionlib_test::DoPutAction>,Transition> action_map;
};
} //namespace gologpp
#endif // ROSBACKEND_H
