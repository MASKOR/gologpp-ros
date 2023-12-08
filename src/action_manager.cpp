#include "gologpp_agent/action_manager.h"

AbstractActionManager::AbstractActionManager(const RosBackend& backend) : backend_(backend), result_() {}

void AbstractActionManager::execute(gpp::shared_ptr<gpp::Activity> a) {
  current_activity_ = a;
  execute_current_activity();
}

void AbstractActionManager::preempt(gpp::shared_ptr<gpp::Activity> a) {
  current_activity_ = a;
  preempt_current_activity();
}

gpp::optional<gologpp::Value> AbstractActionManager::result() {
  auto rv{std::move(result_)};
  result_.reset();
  return rv;
}

void AbstractActionManager::set_result(gpp::optional<gologpp::Value>&& v) {
  result_ = std::move(v);
}

gpp::shared_ptr<gologpp::Activity> AbstractActionManager::current_activity() {
  return current_activity_;
}
