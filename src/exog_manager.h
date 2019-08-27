#pragma once

#include "ros_backend.h"

namespace gpp = gologpp;

class RosBackend;

class AbstractExogManager {
public:
	AbstractExogManager(RosBackend &backend);
	virtual ~AbstractExogManager() = default;

protected:
	RosBackend &backend;

};

template<class ExogT>
class ExogManager : public AbstractExogManager {

public:
	ExogManager(RosBackend &backend, const std::string& topic, int msgs_queue_size = 1000);

	ros::Subscriber exog_subscriber_;

	void topic_cb(const typename ExogT::ConstPtr&);

	void exog_event_to_queue(gpp::unique_ptr<gpp::Value> param);
};

template<class ExogT>
ExogManager<ExogT>::ExogManager(RosBackend &backend, const std::string& topic, int msgs_queue_size)
: AbstractExogManager (backend)
{
	ros::NodeHandle nh("~");
	exog_subscriber_ = nh.subscribe<ExogT>(topic, msgs_queue_size, boost::bind(&ExogManager<ExogT>::topic_cb, this, _1));
}

template<class ExogT>
void ExogManager<ExogT>::exog_event_to_queue(gpp::unique_ptr<gpp::Value> param)
{
	std::shared_ptr<gpp::ExogEvent> ev;
	std::vector< std::shared_ptr <gpp::Global> > global_vec = gpp::global_scope().globals();
	for (std::vector<std::shared_ptr<gpp::Global>>::iterator it = global_vec.begin();
			it != global_vec.end(); ++it){
		if(gpp::shared_ptr< gpp::ExogAction> exog = std::dynamic_pointer_cast< gpp::ExogAction>(*it)){
			if(exog->mapping().backend_name() == this->exog_subscriber_.getTopic()) {
				gpp::vector< gpp::unique_ptr<gpp::Value> > args;
				args.emplace_back(std::move(param));
				ev = std::make_shared<gpp::ExogEvent>(exog, std::move(args));
				break;
			}
		}
	}
	gpp::ReadylogContext &ctx = gpp::ReadylogContext::instance();
	ctx.exog_queue_push(ev);
}

template<class ExogT>
void RosBackend::sub_exog_event(const std::string &topic
	//const boost::function<void(typename ExogT::ConstPtr)> &callback,
	//int msgs_queue_size
)
{
	exog_managers_.push_back(
		std::unique_ptr<AbstractExogManager>(new ExogManager<ExogT>(*this, topic))
	);
}
