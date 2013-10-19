#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <dynamic-graph/factory.h>

#include "dynamic_graph_bridge/ros_init.hh"
#include "ros_export.hh"

namespace dynamicgraph
{
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosExport, "RosExport");

  namespace command
  {
    namespace rosExport
    {
      Clear::Clear
      (RosExport& entity, const std::string& docstring)
	: Command
	  (entity,
	   std::vector<Value::Type> (),
	   docstring)
      {}

      Value Clear::doExecute ()
      {
	RosExport& entity =
	  static_cast<RosExport&> (owner ());

	entity.clear ();
	return Value ();
      }

      List::List
      (RosExport& entity, const std::string& docstring)
	: Command
	  (entity,
	   std::vector<Value::Type> (),
	   docstring)
      {}

      Value List::doExecute ()
      {
	RosExport& entity =
	  static_cast<RosExport&> (owner ());
	return Value (entity.list ());
      }

      Add::Add
      (RosExport& entity, const std::string& docstring)
	: Command
	  (entity,
	   boost::assign::list_of
	   (Value::STRING) (Value::STRING) (Value::STRING),
	   docstring)
      {}

      Value Add::doExecute ()
      {
	RosExport& entity =
	  static_cast<RosExport&> (owner ());
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
        else if (type== "Trajectory")
          entity.add<sot::Trajectory >
	    (signal, topic);
	else if (type == "twist")
	  entity.add<specific::Twist> (signal, topic);
	else if (type == "twistStamped")
	  entity.add<std::pair<specific::Twist, ml::Vector> >
	    (signal, topic);
	else
	  throw std::runtime_error("bad type");
	return Value ();
      }

      Rm::Rm
      (RosExport& entity, const std::string& docstring)
	: Command
	  (entity,
	   boost::assign::list_of (Value::STRING),
	   docstring)
      {}

      Value Rm::doExecute ()
      {
	RosExport& entity =
	  static_cast<RosExport&> (owner ());
	std::vector<Value> values = getParameterValues ();
	const std::string& signal = values[0].value ();
	entity.rm (signal);
	return Value ();
      }
    } // end of errorEstimator.
  } // end of namespace command.

  const std::string RosExport::docstring_
  ("Export dynamic-graph signals as ROS topics.\n"
   "\n"
   "  Use command \"add\" to export a new signal.\n");

  RosExport::RosExport (const std::string& n)
    : dynamicgraph::Entity(n),
      nh_ (rosInit (true)),
      bindedSignal_ ()
  {
    std::string docstring =
      "\n"
      "  Add a signal reading data from a ROS topic\n"
      "\n"
      "  Input:\n"
      "    - type: string among ['double', 'matrix', 'vector', 'vector3',\n"
      "                          'vector3Stamped', 'matrixHomo', 'matrixHomoStamped',\n"
      "                          'trajectory','twist', 'twistStamped'],\n"
      "    - signal: the signal name in dynamic-graph,\n"
      "    - topic:  the topic name in ROS.\n"
      "\n";
    addCommand ("add",
		new command::rosExport::Add
		(*this, docstring));
    docstring =
      "\n"
      "  Remove a signal reading data from a ROS topic\n"
      "\n"
      "  Input:\n"
      "    - name of the signal to remove (see method list for the list of signals).\n"
      "\n";
    addCommand ("rm",
		new command::rosExport::Rm
		(*this, docstring));
    docstring =
      "\n"
      "  Remove all signals reading data from a ROS topic\n"
      "\n"
      "  No input:\n"
      "\n";
    addCommand ("clear",
		new command::rosExport::Clear
		(*this, docstring));
    docstring =
      "\n"
      "  List signals reading data from a ROS topic\n"
      "\n"
      "  No input:\n"
      "\n";
    addCommand ("list",
		new command::rosExport::List
		(*this, docstring));
  }

  RosExport::~RosExport ()
  {}

  void RosExport::display (std::ostream& os) const
  {
    os << CLASS_NAME << std::endl;
  }

  void RosExport::rm (const std::string& signal)
  {
    bindedSignal_.erase (signal);
  }

  std::string RosExport::list ()
  {
    std::string result("[");
    for (std::map<std::string, bindedSignal_t>::const_iterator it =
	   bindedSignal_.begin (); it != bindedSignal_.end (); it++) {
      result += "'" + it->first + "',";
    }
    result += "]";
    return result;
  }

  void RosExport::clear ()
  {
    bindedSignal_.clear ();
  }

  std::string RosExport::getDocString () const
  {
    return docstring_;
  }
} // end of namespace dynamicgraph.
