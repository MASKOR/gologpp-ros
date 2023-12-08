#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gologpp_agent/ros_backend.h"

namespace gpp = gologpp;

using std::placeholders::_1;

class RosBackend;

class AbstractExogManager {
 public:
  explicit AbstractExogManager(const RosBackend& backend);
  virtual ~AbstractExogManager() = default;

 protected:
  const RosBackend& backend;
};

template <class ExogT>
class ExogManager : public AbstractExogManager {
 public:
  ExogManager(const RosBackend& backend, const std::string& topic, int msgs_queue_size = 1000);

  typename rclcpp::Subscription<ExogT>::SharedPtr exog_subscriber_;

  void topic_cb(const typename ExogT::ConstPtr&);
  void exog_event_to_queue(std::unordered_map<std::string, gpp::unique_ptr<gpp::Value>>&& params_to_map);
  std::unordered_map<std::string, gpp::unique_ptr<gpp::Value>> params_to_map(const typename ExogT::ConstPtr& msg);

 private:
  gpp::shared_ptr<gpp::ExogAction> exog_;
};

template <class ExogT>
ExogManager<ExogT>::ExogManager(const RosBackend& backend, const std::string& topic, int msgs_queue_size)
    : AbstractExogManager(backend) {
  auto agent_node = Singleton::instance();
  exog_subscriber_ =
      agent_node->create_subscription<ExogT>(topic, 10, std::bind(&ExogManager<ExogT>::topic_cb, this, _1));

  gpp::shared_ptr<gpp::ExogAction> exog;
  std::vector<std::shared_ptr<gpp::Global>> global_vec = gpp::global_scope().globals();

  for (std::vector<std::shared_ptr<gpp::Global>>::iterator it = global_vec.begin(); it != global_vec.end(); ++it) {
    if ((exog = std::dynamic_pointer_cast<gpp::ExogAction>(*it))) {
      if (exog->mapping().backend_name() == this->exog_subscriber_->get_topic_name()) {
        exog_ = exog;
        break;
      }
    }
  }
}

template <class ExogT>
void ExogManager<ExogT>::topic_cb(const typename ExogT::ConstPtr& msg) {
  if (backend.ctx_ready) {
    // Topic msgs map to exog Event and pushed to Exog Queue
    exog_event_to_queue(params_to_map(msg));
  }
}

template <class ExogT>
void ExogManager<ExogT>::exog_event_to_queue(
    std::unordered_map<std::string, gpp::unique_ptr<gpp::Value>>&& params_to_map) {
  gpp::Binding param_binding;

  for (auto it = params_to_map.begin(); it != params_to_map.end(); it++) {
    gpp::unique_ptr<gpp::Reference<gpp::Variable>> param_ref{exog_->param_ref(it->first)};
    param_binding.bind(param_ref->target(), std::move(it->second));
  }

  gpp::shared_ptr<gpp::ExogEvent> ev{new gpp::ExogEvent(exog_, std::move(param_binding))};

  gpp::ReadylogContext& ctx = gpp::ReadylogContext::instance();
  ctx.exog_queue_push(ev);
}

template <class ExogT>
void RosBackend::create_ExogManger(const std::string& topic) {
  exog_managers_.push_back(std::unique_ptr<AbstractExogManager>(new ExogManager<ExogT>(*this, topic)));
}
