#include <execution/controller.h>

#include "builtin_interfaces/msg/duration.h"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gologpp_agent/action_manager.h"
#include "gologpp_agent/exog_manager.h"
#include "gologpp_agent/ros_backend.h"
#include "spot_msgs/action/trajectory.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

template <>
ActionManager<spot_msgs::action::Trajectory>::GoalT ActionManager<spot_msgs::action::Trajectory>::build_goal(
    const gpp::Activity& a) {
  auto agent_node = Singleton::instance();
  auto goal = spot_msgs::action::Trajectory::Goal();
  auto target_pose = geometry_msgs::msg::PoseStamped();
  auto frame = std::string(a.mapped_arg_value("frame_id"));
  builtin_interfaces::msg::Time time = agent_node->get_clock()->now();
  tf2::Quaternion q;
  float yaw = 0;
  auto duration = builtin_interfaces::msg::Duration();

  target_pose.header.stamp = time;
  duration.sec = 10;
  duration.nanosec = 0.0;

  goal.duration = duration;
  goal.precise_positioning = false;  // bool(a.mapped_arg_value("precise_positioning"));

  if (frame == "body") {
    target_pose.header.frame_id = std::string(a.mapped_arg_value("frame_id"));
    target_pose.pose.position.x = a.mapped_arg_value("posX").numeric_convert<float>();
    target_pose.pose.position.y = a.mapped_arg_value("posY").numeric_convert<float>();
    target_pose.pose.position.z = 0.0;  // is not used

    yaw = a.mapped_arg_value("yaw").numeric_convert<float>();

    q.setRPY(0.0, 0.0, yaw);

    target_pose.pose.orientation.x = q.x();
    target_pose.pose.orientation.y = q.y();
    target_pose.pose.orientation.z = q.z();
    target_pose.pose.orientation.w = q.w();
    goal.target_pose = target_pose;

    return goal;
  } else {
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(agent_node->get_clock());
    geometry_msgs::msg::TransformStamped t;
    // Look up for the transformation between target_frame and body frames
    // and send goal relativ from body
    try {
      t = tf_buffer_->lookupTransform("body", frame, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_INFO(LOGGER, "Could not transform %s to %s: %s", frame.c_str(), "body", ex.what());
      target_pose.header.frame_id = "ABORT_ACTION";
      return goal;
    } catch (tf2::ConnectivityException& e) {
      RCLCPP_WARN(LOGGER, e.what());
      target_pose.header.frame_id = "ABORT_ACTION";
      return goal;
    } catch (tf2::ExtrapolationException& e) {
      RCLCPP_WARN(LOGGER, e.what());
      target_pose.header.frame_id = "ABORT_ACTION";
      return goal;
    } catch (tf2::LookupException& e) {
      RCLCPP_WARN(LOGGER, e.what());
      target_pose.header.frame_id = "ABORT_ACTION";
      return goal;
    }

    target_pose.header.frame_id = "body";
    target_pose.pose.position.x = t.transform.translation.x;
    target_pose.pose.position.y = t.transform.translation.y;
    target_pose.pose.position.z = 0.0;  // is not used

    target_pose.pose.orientation.x = 0.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = t.transform.rotation.w;

    goal.target_pose = target_pose;

    return goal;
  }
}

template <>
ServiceManager<std_srvs::srv::Trigger>::RequestT ServiceManager<std_srvs::srv::Trigger>::build_request(
    const gpp::Activity& a) {
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  return request;
}

template <>
ServiceManager<std_srvs::srv::SetBool>::RequestT ServiceManager<std_srvs::srv::SetBool>::build_request(
    const gpp::Activity& a) {
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = static_cast<bool>(a.mapped_arg_value("data"));
  return request;
}

void RosBackend::define_spot_msgs_actions() {
  built_interface_names.push_back("spot_msgs");

  create_ActionManager<spot_msgs::action::Trajectory>("/trajectory");

  create_ServiceManager<std_srvs::srv::Trigger>("/claim");
  create_ServiceManager<std_srvs::srv::Trigger>("/release");
  create_ServiceManager<std_srvs::srv::Trigger>("/stop");
  create_ServiceManager<std_srvs::srv::Trigger>("/self_right");
  create_ServiceManager<std_srvs::srv::Trigger>("/sit");
  create_ServiceManager<std_srvs::srv::Trigger>("/stand");
  create_ServiceManager<std_srvs::srv::Trigger>("/power_on");
  create_ServiceManager<std_srvs::srv::Trigger>("/power_off");
  create_ServiceManager<std_srvs::srv::Trigger>("/estop/hard");
  create_ServiceManager<std_srvs::srv::Trigger>("/estop/gentle");
  create_ServiceManager<std_srvs::srv::Trigger>("/estop/release");

  create_ServiceManager<std_srvs::srv::SetBool>("/clear_behavior_fault");

  create_ExogManger<std_msgs::msg::Bool>("/exog_event_next_TF");
  create_ExogManger<std_msgs::msg::Bool>("/exog_event_next_Lap");
}
