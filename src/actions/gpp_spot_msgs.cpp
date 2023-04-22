#include "action_manager.h"
#include "exog_manager.h"
#include "ros_backend.h"

#include <execution/controller.h>

#include "tf2/LinearMath/Quaternion.h"
#include "spot_msgs/action/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "builtin_interfaces/msg/duration.h"
#include "builtin_interfaces/msg/time.hpp"


template<>
ActionManager<spot_msgs::action::Trajectory>::GoalT
ActionManager<spot_msgs::action::Trajectory>::build_goal(const gpp::Activity &a)
{
    auto agent_node = Singleton::instance();
	auto goal = spot_msgs::action::Trajectory::Goal();
    auto target_pose = geometry_msgs::msg::PoseStamped();

    builtin_interfaces::msg::Time time = agent_node->get_clock()->now();
    //auto time_msg = time.to_msg();
    target_pose.header.stamp = time;
    target_pose.header.frame_id = std::string(a.mapped_arg_value("frame_id"));
    target_pose.pose.position.x = a.mapped_arg_value("posX").numeric_convert<float>();
    target_pose.pose.position.y = a.mapped_arg_value("posY").numeric_convert<float>();
    target_pose.pose.position.z = 0.0; // is not used

    tf2::Quaternion q;
    q.setRPY(0.0,0.0,0.0);
    target_pose.pose.orientation.x = q.x();
    target_pose.pose.orientation.y = q.y();
    target_pose.pose.orientation.z = q.z();
    target_pose.pose.orientation.w = q.w();

    auto duration = builtin_interfaces::msg::Duration();
    duration.sec = 10;
    duration.nanosec = 0.0;
    goal.target_pose = target_pose;
    goal.duration = duration;
    goal.precise_positioning = false;
	return goal;
}


void RosBackend::define_spot_actions()
{
	create_ActionManager<spot_msgs::action::Trajectory>("/trajectory");
}