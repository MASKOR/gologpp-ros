#include <model/action.h>
#include <model/effect_axiom.h>
#include <model/fluent.h>
#include <model/formula.h>
#include <model/procedural.h>
#include <model/reference.h>

#include <atomic>
#include <iostream>
#include <memory>

#include "gologpp_agent/ros_backend.h"

// #ifdef GOLOGPP_TEST_READYLOG
#include <semantics/readylog/execution.h>
// #endif

// #ifdef GOLOGPP_TEST_PARSER
#include <parser/parser.h>
// #endif
#include "rclcpp/rclcpp.hpp"

namespace gpp = gologpp;

void load_n_exec_program(std::string program) {
  gpp::parser::parse_file(program);

  gpp::shared_ptr<gpp::Reference<gpp::Procedure>> mainproc{
      gpp::global_scope().lookup_global<gpp::Procedure>("main")->make_ref({})};

  gpp::eclipse_opts options;
  options.trace = false;
  options.toplevel = false;
  options.guitrace = false;
  RosBackend* rosbackend = new RosBackend();

  gpp::ReadylogContext::init(options, gpp::unique_ptr<gpp::PlatformBackend>(rosbackend));
  gpp::ReadylogContext& ctx = gpp::ReadylogContext::instance();

  mainproc->attach_semantics(ctx.semantics_factory());

  rosbackend->ctx_ready = true;
  ctx.run(*mainproc);
  rosbackend->ctx_ready = false;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::string param;

  auto agent_node = Singleton::instance();
  std::string gpp_code = "";

  if ((argc <= 1) || (argv[argc - 1] == NULL)) {
    // there is NO input...
    gpp_code = "turtlesim_example";
    std::cerr << "No agent provided in the argument! Turtlesim example started!" << std::endl;
  } else {
    // there is an input...
    gpp_code = argv[argc - 1];
    RCLCPP_INFO_STREAM(agent_node->get_logger(), "Starting Golog++ agent: " << gpp_code);
  }

  gpp_code = SOURCE_DIR "/agents/" + gpp_code + ".gpp";

  if (!agent_node->has_parameter("gpp_code")) agent_node->declare_parameter("gpp_code", gpp_code);
  agent_node->get_parameter("gpp_code", gpp_code);

  load_n_exec_program(gpp_code);

  rclcpp::shutdown();
  gpp::ReadylogContext::shutdown();

  return 0;
}
