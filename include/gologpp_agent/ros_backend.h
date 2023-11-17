#ifndef ROSBACKEND_H
#define ROSBACKEND_H

// Remove spurious clang code model error
#ifdef Q_CREATOR_RUN
#undef __GCC_ASM_FLAG_OUTPUTS__
#endif

#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <execution/platform_backend.h>
#include <execution/transition.h>
#include <semantics/readylog/execution.h>

#include <chrono>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("gpp_ros_backend");

class AbstractActionManager;
class AbstractExogManager;

class Singleton {
 public:
  Singleton(Singleton const&) = delete;
  Singleton& operator=(Singleton const&) = delete;
  // std::shared_ptr<rclcpp::Node> node =
  // rclcpp::Node::make_shared("gologpp_agent");

  static std::shared_ptr<rclcpp::Node> instance() {
    static std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("gologpp_agent");
    return node;
  }

 private:
  Singleton() {}
};

class RosBackend : public gologpp::PlatformBackend {
 private:
  using string = gologpp::string;
  using Value = gologpp::Value;
  using Type = gologpp::Type;
  using Activity = gologpp::Activity;
  using Clock = gologpp::Clock;
  template <class T>
  using shared_ptr = gologpp::shared_ptr<T>;

 public:
  RosBackend();
  ~RosBackend() override;
  void execute_activity(shared_ptr<Activity> a) override;
  void preempt_activity(shared_ptr<Activity> trans) override;
  Clock::time_point time() const noexcept override;

  Value eval_exog_function(const Type& return_type, const string& backend_name,
                           const std::unordered_map<string, Value>& args) override;

  // std::mutex exog_mutex;
  std::atomic<bool> ctx_ready;

 private:
  void terminate_() override;

  std::vector<std::string> built_interface_names;

  // May have an implementation if the corresponding package has been found
  void define_turtlesim_actions();
  void define_gpp_action_examples_interface_actions();
  void define_webots_spot_msgs_actions();
  void define_nav2_msgs_actions();
  void define_spot_msgs_actions();

  template <class ActionT>
  void create_ActionManager(const std::string& name);

  AbstractActionManager& get_ActionManager(shared_ptr<Activity>);

  // create exogManager
  template <class ExogT>
  void create_ExogManger(const std::string&);

  void spin_exog_thread();

  template <class ServiceT>
  void create_ServiceManager(const std::string&);

  std::vector<std::unique_ptr<AbstractExogManager>> exog_managers_;

  std::unordered_map<std::string, std::unique_ptr<AbstractActionManager>> action_managers_;
};

#endif  // ROSBACKEND_H
