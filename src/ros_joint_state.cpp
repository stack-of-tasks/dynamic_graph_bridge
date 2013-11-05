#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>

#include <dynamic-graph/command.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>
#include <sot-dynamic/dynamic.h>

#include "dynamic_graph_bridge/ros_init.hh"
#include "ros_joint_state.hh"
#include "sot_to_ros.hh"

namespace dynamicgraph
{
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosJointState, "RosJointState");
  const double RosJointState::ROS_JOINT_STATE_PUBLISHER_RATE = 0.01;

  namespace command
  {
    using ::dynamicgraph::command::Command;
    using ::dynamicgraph::command::Value;

    class RetrieveJointNames : public Command
    {
    public:
      RetrieveJointNames (RosJointState& entity,
			  const std::string& docstring);
      virtual Value doExecute ();
    };

    RetrieveJointNames::RetrieveJointNames
    (RosJointState& entity, const std::string& docstring)
      : Command (entity, boost::assign::list_of (Value::STRING), docstring)
    {}

    namespace
    {
      void
      buildJointNames (sensor_msgs::JointState& jointState, CjrlJoint* joint)
      {
	if (!joint)
	  return;
	// Ignore anchors.
	if (joint->numberDof() != 0)
	  {
	    // If we only have one dof, the dof name is the joint name.
	    if (joint->numberDof() == 1)
	      {
		jointState.name[joint->rankInConfiguration()] =
		  joint->getName();
	      }
	    // ...otherwise, the dof name is the joint name on which
	    // the dof id is appended.
	    else
	      for (unsigned i = 0; i < joint->numberDof(); ++i)
		{
		  boost::format fmt("%1%_%2%");
		  fmt % joint->getName();
		  fmt % i;
		  jointState.name[joint->rankInConfiguration() + i] =
		    fmt.str();
		}
	  }
	for (unsigned i = 0; i < joint->countChildJoints (); ++i)
	  buildJointNames (jointState, joint->childJoint (i));
      }
    } // end of anonymous namespace

    Value RetrieveJointNames::doExecute ()
    {
      RosJointState& entity = static_cast<RosJointState&> (owner ());

      std::vector<Value> values = getParameterValues ();
      std::string name = values[0].value ();

      if (!dynamicgraph::PoolStorage::getInstance ()->existEntity (name))
	{
	  std::cerr << "invalid entity name" << std::endl;
	  return Value ();
	}

      dynamicgraph::sot::Dynamic* dynamic =
	dynamic_cast<dynamicgraph::sot::Dynamic*>
	(&dynamicgraph::PoolStorage::getInstance ()->getEntity (name));
      if (!dynamic)
	{
	  std::cerr << "entity is not a Dynamic entity" << std::endl;
	  return Value ();
	}

      CjrlHumanoidDynamicRobot* robot = dynamic->m_HDR;
      if (!robot)
	{
	  std::cerr << "no robot in the dynamic entity" << std::endl;
	  return Value ();
	}

      entity.jointState ().name.resize (robot->numberDof());
      buildJointNames (entity.jointState (), robot->rootJoint());
      return Value ();
    }
  } // end of namespace command.

  RosJointState::RosJointState (const std::string& n)
    : Entity (n),
      // do not use callbacks, so do not create a useless spinner
      nh_ (rosInit (false)),
      state_ (0, MAKE_SIGNAL_STRING(name, true, "Vector", "state")),
      publisher_ (nh_, "dynamic_graph/joint_states", 5),
      jointState_ (),
      trigger_ (boost::bind (&RosJointState::trigger, this, _1, _2),
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
    signalRegistration (state_ << trigger_);
    trigger_.setNeedUpdateFromAllChildren (true);

    // Fill header.
    jointState_.header.seq = 0;
    jointState_.header.stamp.sec = 0;
    jointState_.header.stamp.nsec = 0;
    jointState_.header.frame_id = "";

    std::string docstring =
      "\n"
      "  Retrieve joint names using robot model contained in a Dynamic entity\n"
      "\n"
      "  Input:\n"
      "    - dynamic entity name (i.e. robot.dynamic.name)\n"
      "\n";
    addCommand ("retrieveJointNames",
		new command::RetrieveJointNames (*this, docstring));
  }

  RosJointState::~RosJointState ()
  {}

  int&
  RosJointState::trigger (int& dummy, int t)
  {
    ros::Duration dt = ros::Time::now () - lastPublicated_;
    if (dt > rate_ && publisher_.trylock ())
      {
	lastPublicated_ = ros::Time::now ();

	// State size without the free floating.
	std::size_t s = state_.access (t).size ();

	// Safety check: if data are inconsistent, clear
	// the joint names to avoid sending erroneous data.
	// This should not happen unless you change
	// the robot model at run-time.
	if (s != jointState_.name.size())
	  jointState_.name.clear();

	// Update header.
	++jointState_.header.seq;

	ros::Time now = ros::Time::now ();
	jointState_.header.stamp.sec = now.sec;
	jointState_.header.stamp.nsec = now.nsec;

	// Fill position.
	jointState_.position.resize (s);
	for (std::size_t i = 0; i < s; ++i)
	  jointState_.position[i] = state_.access (t) (i);

	publisher_.msg_ = jointState_;
	publisher_.unlockAndPublish ();
      }
    return dummy;
  }
} // end of namespace dynamicgraph.
