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

#include "dynamic_graph_bridge/ros_init.hh"
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
	return Value (entity.list ());
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
	else if (type == "vector3")
	  entity.add<specific::Vector3> (signal, topic);
	else if (type == "vector3Stamped")
	  entity.add<std::pair<specific::Vector3, ml::Vector> > (signal, topic);
	else if (type == "matrixHomo")
	  entity.add<sot::MatrixHomogeneous> (signal, topic);
	else if (type == "matrixHomoStamped")
	  entity.add<std::pair<sot::MatrixHomogeneous, ml::Vector> >
	    (signal, topic);
        else if (type == "Trajectory")
          entity.add<sot::Trajectory>(signal,topic);
	else if (type == "twist")
	  entity.add<specific::Twist> (signal, topic);
	else if (type == "twistStamped")
	  entity.add<std::pair<specific::Twist, ml::Vector> > (signal, topic);
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

  const std::string RosImport::docstring_
  ("Import ROS topics as dynamic-graph signals.\n"
   "\n"
   "  Use command \"add\" to import a new ROS topic.\n");

  RosImport::RosImport (const std::string& n)
    : dynamicgraph::Entity(n),
      // rosImport do not use callback so do not create a useless spinner.
      nh_ (rosInit (false)),
      bindedSignal_ (),
      trigger_ (boost::bind (&RosImport::trigger, this, _1, _2),
		sotNOSIGNAL,
		MAKE_SIGNAL_STRING(name, true, "int", "trigger")),
      rate_ (ROS_JOINT_STATE_PUBLISHER_RATE),
      lastPublicated_ ()
  {
    try {
      lastPublicated_ = ros::Time::now ();
    } catch (const std::exception& exc) {
      throw std::runtime_error ("Failed to call ros::Time::now ():" +
				std::string (exc.what ()));
    }
    signalRegistration (trigger_);
    trigger_.setNeedUpdateFromAllChildren (true);

    std::string docstring =
      "\n"
      "  Add a signal writing data to a ROS topic\n"
      "\n"
      "  Input:\n"
      "    - type: string among ['double', 'matrix', 'vector', 'vector3',\n"
      "                          'vector3Stamped', 'matrixHomo', 'matrixHomoStamped',\n"
      "                          'twist', 'twistStamped'],\n"
      "    - signal: the signal name in dynamic-graph,\n"
      "    - topic:  the topic name in ROS.\n"
      "\n";
    addCommand ("add",
		new command::rosImport::Add
		(*this, docstring));
    docstring =
      "\n"
      "  Remove a signal writing data to a ROS topic\n"
      "\n"
      "  Input:\n"
      "    - name of the signal to remove (see method list for the list of signals).\n"
      "\n";
    addCommand ("rm",
		new command::rosImport::Rm
		(*this, docstring));
    docstring =
      "\n"
      "  Remove all signals writing data to a ROS topic\n"
      "\n"
      "  No input:\n"
      "\n";
    addCommand ("clear",
		new command::rosImport::Clear
		(*this, docstring));
    docstring =
      "\n"
      "  List signals writing data to a ROS topic\n"
      "\n"
      "  No input:\n"
      "\n";
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

  std::string RosImport::list () const
  {
    std::string result("[");
    for (std::map<std::string, bindedSignal_t>::const_iterator it =
	   bindedSignal_.begin (); it != bindedSignal_.end (); it++) {
      result += "'" + it->first + "',";
    }
    result += "]";
    return result;
 }

  void RosImport::clear ()
  {
    bindedSignal_.clear ();
  }

  int& RosImport::trigger (int& dummy, int t)
  {
    typedef std::map<std::string, bindedSignal_t>::iterator iterator_t;

    ros::Duration dt = ros::Time::now () - lastPublicated_;
    if (dt < rate_)
      return dummy;

    for (iterator_t it = bindedSignal_.begin ();
	 it != bindedSignal_.end (); ++it)
      {
	boost::get<1>(it->second) (t);
      }
    return dummy;
  }

  std::string RosImport::getDocString () const
  {
    return docstring_;
  }

} // end of namespace dynamicgraph.
