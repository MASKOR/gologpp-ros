#include <memory>
#include <iostream>
#include <atomic>

#include "pepper_backend.h"

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

using namespace gologpp;



void test_parser()
{
	VoidExpression *mainproc = parser::parse_file(SOURCE_DIR "/move_base_example.gpp").release();

	gologpp::shared_ptr<NumericFluent> on = global_scope().lookup_global<NumericFluent>("on", 1);
	gologpp::shared_ptr<Action> put = global_scope().lookup_global<Action>("stack", 2);
	gologpp::shared_ptr<BooleanFunction> goal = global_scope().lookup_global<BooleanFunction>("goal", 0);

	if (on && put && goal)
		std::cout << on->name() << " " << put->name() << " " << goal->name() << std::endl;

	eclipse_opts options;
	options.trace = false;
	options.toplevel = false;
	options.guitrace = false;

	ReadylogContext::init(options);
	ReadylogContext &ctx = ReadylogContext::instance();

	ctx.run(Block(
		new Scope(global_scope()),
		{ mainproc }
	));

	ReadylogContext::shutdown();
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "gologpp_agent");
	test_parser();
	return 0;
}





