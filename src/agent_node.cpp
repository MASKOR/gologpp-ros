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
	std::cout<<"Test_parser"<<std::endl;
	unique_ptr<Statement> mainproc = parser::parse_file(SOURCE_DIR "/example.gpp");
	eclipse_opts options;
	options.trace = false;
	options.guitrace = false;

	ReadylogContext::init(unique_ptr<AExecutionBackend>(new Pepper_Backend), options);
	ReadylogContext &ctx = ReadylogContext::instance();

	ctx.run(Block(
	new Scope(global_scope()),
	{ mainproc.release() }
	));

	ReadylogContext::shutdown();
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "gologpp_agent");
	test_parser();
	return 0;
}





