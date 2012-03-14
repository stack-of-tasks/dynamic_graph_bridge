#include <limits>

#include <boost/bind.hpp>

#include <ros/ros.h>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/null-ptr.hh>
#include <jrl/dynamics/urdf/parser.hh>

#include "dynamic_graph_bridge/ros_init.hh"
#include "robot_model.hh"

namespace dynamicgraph
{
  namespace dg = dynamicgraph;

  namespace command
  {
    LoadFromParameterServer::LoadFromParameterServer
    (RosRobotModel& entity,
     const std::string& docstring)
      : Command (entity, std::vector<Value::Type>(), docstring)
    {}

    Value
    LoadFromParameterServer::doExecute()
    {
      RosRobotModel& entity =
	static_cast<RosRobotModel&>(owner ());
      entity.loadFromParameterServer ();
      return Value ();
    }

    LoadUrdf::LoadUrdf(RosRobotModel& entity, const std::string& docstring)
      : Command (entity, std::vector<Value::Type>(), docstring)
    {}

    Value
    LoadUrdf::doExecute()
    {
      RosRobotModel& entity =
	static_cast<RosRobotModel&>(owner ());

      const std::vector<Value>& values = getParameterValues ();
      std::string resourceName = values[0].value ();

      entity.loadUrdf (resourceName);
      return Value ();
    }
  } // end of namespace command.

  RosRobotModel::RosRobotModel (const std::string& name)
    : Entity(name),
      robot_ (0),
      lastComputation_ (std::numeric_limits<int>::min()),
      q_ (dynamicgraph::nullptr,
	  "RosRobotModel(" + name + ")::input(vector)::q"),
      dq_ (dynamicgraph::nullptr,
	   "RosRobotModel(" + name + ")::input(vector)::dq"),
      ddq_ (dynamicgraph::nullptr,
	    "RosRobotModel(" + name + ")::input(vector)::ddq")
  {
    signalRegistration(q_);
    signalRegistration(dq_);
    signalRegistration(ddq_);

    std::string docstring;

    docstring =
      "\n"
      "  Load the robot model from the parameter server.\n"
      "\n"
      "  This is the recommended method.\n"
      "\n";
    addCommand ("loadFromParameterServer",
		new command::LoadFromParameterServer (*this, docstring));

    docstring =
      "\n"
      "  Load the robot model from an URDF file.\n"
      "\n";
    addCommand ("loadUrdf", new command::LoadUrdf (*this, docstring));
  }

  RosRobotModel::~RosRobotModel ()
  {}

  void
  RosRobotModel::loadUrdf (const std::string& filename)
  {
    jrl::dynamics::urdf::Parser parser;
    robot_ = parser.parse(filename, "base_footprint_joint");
    buildSignals ();
  }

  void
  RosRobotModel::loadFromParameterServer ()
  {
    jrl::dynamics::urdf::Parser parser;

    ros::NodeHandle& nh = rosInit (false);
    std::string robotDescription;
    ros::param::param<std::string>
      ("robot_description", robotDescription, "");
    if (robotDescription.empty ())
      throw std::runtime_error
	("No model available as ROS parameter. Fail.");
    robot_ = parser.parseStream (robotDescription, "base_footprint_joint");
    buildSignals ();
  }

  void
  RosRobotModel::buildSignals ()
  {
    // iterate on tree nodes and add signals
    typedef dg::SignalTimeDependent<MatrixHomogeneous, int> signal_t;
    ::std::vector<CjrlJoint*> jointsVect = robot_->jointVector();

    for (uint i=0; i<jointsVect.size(); ++i)
      {
	CjrlJoint* currentJoint = jointsVect[i];
	std::string signame = currentJoint->getName();

	signal_t* sig
	  = new signal_t
	  (boost::bind
	   (&RosRobotModel::computeBodyPosition, this, currentJoint, _1, _2),
	   0,
	   "RosRobotModel(" + getName () + ")::output(matrix)::" + signame);
	sig->addDependency (q_);
	sig->addDependency (dq_);
	sig->addDependency (ddq_);
	genericSignalRefs_.push_back (sig);
	signalRegistration (*sig);
      }
    // force recomputation as the model has changed.
    lastComputation_ = std::numeric_limits<int>::min();
  }

  RosRobotModel::MatrixHomogeneous&
  RosRobotModel::computeBodyPosition (CjrlJoint* joint,
				      MatrixHomogeneous& position,
				      int t)
  {
    update (t);
    for(unsigned i = 0; i < position.nbRows(); ++i)
      for(unsigned j = 0; j < position.nbCols(); ++j)
	position.elementAt(i,j) =
	  joint->currentTransformation().m[i * position.nbCols() + j];
    return position;
  }

  namespace
  {
    vectorN convertVector (const ml::Vector& v)
    {
      vectorN res (v.size());
      for (unsigned i = 0; i < v.size(); ++i)
	res[i] = v(i);
      return res;
    }
  } // end of anonymous namespace.

  void
  RosRobotModel::update (int t)
  {
    if (t <= lastComputation_)
      return;

    vectorN q = convertVector (q_ (t));
    vectorN dq = convertVector (dq_ (t));
    vectorN ddq = convertVector (dq_ (t));

    if (!robot_->currentConfiguration(q))
      throw std::runtime_error ("failed to update current configuration");
    if (!robot_->currentVelocity(dq))
      throw std::runtime_error ("failed to update current velocity");
    if (!robot_->currentAcceleration(ddq))
      throw std::runtime_error ("failed to update current acceleration");
    robot_->computeForwardKinematics();
    lastComputation_ = t;
  }

  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosRobotModel, "RosRobotModel");
} // end of namespace dynamicgraph.
