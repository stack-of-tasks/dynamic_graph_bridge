#include <stdexcept>

#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command.h>

#include "ros_import.hh"

namespace dynamicgraph
{
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosImport, "RosImport");

  namespace command
  {
    namespace rosImport
    {
      Clear::Clear
      (RosImport& entity, const std::string& docstring)
	: Command
	  (entity,
	   std::vector<Value::Type> (),
	   docstring)
      {}

      Value Clear::doExecute ()
      {
	RosImport& entity =
	  static_cast<RosImport&> (owner ());

	entity.clear ();
	return Value ();
      }

      List::List
      (RosImport& entity, const std::string& docstring)
	: Command
	  (entity,
	   std::vector<Value::Type> (),
	   docstring)
      {}

      Value List::doExecute ()
      {
	RosImport& entity =
	  static_cast<RosImport&> (owner ());
	entity.list ();
	return Value ();
      }

      Add::Add
      (RosImport& entity, const std::string& docstring)
	: Command
	  (entity,
	   boost::assign::list_of
	   (Value::STRING) (Value::STRING) (Value::STRING),
	   docstring)
      {}

      Value Add::doExecute ()
      {
	RosImport& entity =
	  static_cast<RosImport&> (owner ());
	std::vector<Value> values = getParameterValues ();

	const std::string& type = values[0].value ();
	const std::string& signal = values[1].value ();
	const std::string& topic = values[2].value ();

	if (type == "double")
	  entity.add<double> (signal, topic);
	else if (type == "matrix")
	  entity.add<ml::Matrix> (signal, topic);
	else if (type == "vector")
	  entity.add<ml::Vector> (signal, topic);
	else
	  throw std::runtime_error("bad type");
	return Value ();
      }

      Rm::Rm
      (RosImport& entity, const std::string& docstring)
	: Command
	  (entity,
	   boost::assign::list_of (Value::STRING),
	   docstring)
      {}

      Value Rm::doExecute ()
      {
	RosImport& entity =
	  static_cast<RosImport&> (owner ());
	std::vector<Value> values = getParameterValues ();
	const std::string& signal = values[0].value ();
	entity.rm (signal);
	return Value ();
      }
    } // end of errorEstimator.
  } // end of namespace command.


  const char* rosInit()
  {
    int argc = 1;
    char* arg0 = strdup("ros_import");
    char* argv[] = {arg0, 0};
    ros::init(argc, argv, "ros_import");
    free (arg0);
    return "dynamic_graph";
  }

  RosImport::RosImport (const std::string& n)
    : dynamicgraph::Entity(n),
      nh_ (rosInit ()),
      bindedSignal_ (),
      trigger_ (boost::bind (&RosImport::trigger, this, _1, _2),
		sotNOSIGNAL,
		MAKE_SIGNAL_STRING(name, true, "int", "trigger"))
  {
    signalRegistration (trigger_);
    trigger_.setNeedUpdateFromAllChildren (true);

    std::string docstring;
    addCommand ("add",
		new command::rosImport::Add
		(*this, docstring));
    addCommand ("rm",
		new command::rosImport::Rm
		(*this, docstring));
    addCommand ("clear",
		new command::rosImport::Clear
		(*this, docstring));
    addCommand ("list",
		new command::rosImport::List
		(*this, docstring));
  }

  RosImport::~RosImport ()
  {
  }

  void RosImport::display (std::ostream& os) const
  {
    os << CLASS_NAME << std::endl;
  }

  void RosImport::rm (const std::string& signal)
  {
    bindedSignal_.erase (signal);
  }

  void RosImport::list ()
  {
    std::cout << CLASS_NAME << std::endl;
  }

  void RosImport::clear ()
  {
    bindedSignal_.clear ();
  }

  int& RosImport::trigger (int& dummy, int t)
  {
    typedef std::map<std::string, bindedSignal_t>::iterator iterator_t;

    for (iterator_t it = bindedSignal_.begin ();
	 it != bindedSignal_.end (); ++it)
      {
	boost::get<2>(it->second) (t);
      }
    return dummy;
  }

} // end of namespace dynamicgraph.
