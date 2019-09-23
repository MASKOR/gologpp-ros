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
	void exog_event_to_queue(std::unordered_map< std::string, gpp::unique_ptr<gpp::Value> > params_to_map);
	 std::unordered_map< std::string, gpp::unique_ptr<gpp::Value> > params_to_map(const typename ExogT::ConstPtr& msg);
private:
	gpp::shared_ptr< gpp::ExogAction> exog_;
};

template<class ExogT>
ExogManager<ExogT>::ExogManager(RosBackend &backend, const std::string& topic, int msgs_queue_size)
: AbstractExogManager (backend)
{
	ros::NodeHandle nh("~");
	exog_subscriber_ = nh.subscribe<ExogT>(topic, msgs_queue_size, boost::bind(&ExogManager<ExogT>::topic_cb, this, _1));

	gpp::shared_ptr< gpp::ExogAction> exog;
	std::vector< std::shared_ptr <gpp::Global> > global_vec = gpp::global_scope().globals();

	for (std::vector<std::shared_ptr<gpp::Global>>::iterator it = global_vec.begin();
			it != global_vec.end(); ++it){
		if((exog = std::dynamic_pointer_cast< gpp::ExogAction>(*it))){

			if(exog->mapping().backend_name() == this->exog_subscriber_.getTopic()) {
				exog_ = exog;
				break;
			}
		}
	}
}

template<class ExogT>
void
ExogManager<ExogT>::topic_cb(const typename ExogT::ConstPtr& msg)
{
	ROS_INFO_STREAM("I heard: " << bool(msg->statePressed));
	exog_event_to_queue(
		params_to_map(msg)
	);
}

template<class ExogT>
void ExogManager<ExogT>::exog_event_to_queue( std::unordered_map< std::string, gpp::unique_ptr<gpp::Value> > params_to_map)
{
	std::shared_ptr<gpp::ExogEvent> ev;
	std::unordered_map< gpp::Reference <gpp::Variable>, gpp::unique_ptr<gpp::Value> > params_to_args;

				for (auto it = params_to_map.begin(); it != params_to_map.end(); it++){
					gpp::Reference<gpp::Variable> *var_of_target = exog_->param_ref(it->first);
					params_to_args.emplace(std::move(*var_of_target), std::move(it->second));
				}
				ev = std::make_shared<gpp::ExogEvent>(exog_, std::move(params_to_args));

	gpp::ReadylogContext &ctx = gpp::ReadylogContext::instance();
	ctx.exog_queue_push(ev);
}

template<class ExogT>
void RosBackend::create_ExogManger(const std::string &topic)
{
	exog_managers_.push_back(
		std::unique_ptr<AbstractExogManager>(new ExogManager<ExogT>(*this, topic))
	);
}
