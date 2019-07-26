#include <memory>
#include <iostream>
#include <atomic>

#include "ros_backend.h"

#include <model/formula.h>
#include <model/reference.h>

#include <model/action.h>
#include <model/effect_axiom.h>
#include <model/fluent.h>
#include <model/procedural.h>

//#ifdef GOLOGPP_TEST_READYLOG
#include <semantics/readylog/execution.h>
//#endif

//#ifdef GOLOGPP_TEST_PARSER
#include <parser/parser.h>
//#endif
#include <ros/ros.h>


namespace gpp = gologpp;



void load_n_exec_program(std::string program)
{
	gpp::Expression *mainproc = gpp::parser::parse_file(SOURCE_DIR "/"+program+".gpp").release();

	gpp::eclipse_opts options;
	options.trace = false;
	options.toplevel = false;
	options.guitrace = false;
	gpp::ReadylogContext::init(
	options, gpp::unique_ptr<gpp::PlatformBackend>(new RosBackend())
	);
	gpp::ReadylogContext &ctx = gpp::ReadylogContext::instance();

	ctx.run(gpp::Block(
	new gpp::Scope(gpp::global_scope()),
	{ mainproc }
	));

	gpp::ReadylogContext::shutdown();
}




int main(int argc, char **argv)
{
	std::string param;
	ros::init(argc, argv, "gologpp_agent");
	ros::NodeHandle nh("~");
	if (nh.getParam("program", param)) {
		ROS_INFO("Got parameter: %s", param.c_str());
		nh.deleteParam("program");
		load_n_exec_program(param.c_str());

	} else {
		ROS_INFO("Default program");
		nh.deleteParam("program");
		load_n_exec_program("move_base_example");

	}
	return 0;
}



