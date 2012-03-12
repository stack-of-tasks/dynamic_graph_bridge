#include <boost/bind.hpp>
#include <boost/format.hpp>

#include <dynamic-graph/factory.h>

#include "dynamic_graph_bridge/ros_init.hh"
#include "ros_joint_state.hh"
#include "sot_to_ros.hh"

//FIXME: this is very very wrong but we need to update abstract-robot-dynamics
// to publish joint names.
static const char* dof_names[] =
  {
    "RLEG_JOINT0", "RLEG_JOINT1", "RLEG_JOINT2",
    "RLEG_JOINT3", "RLEG_JOINT4", "RLEG_JOINT5",
    "LLEG_JOINT0", "LLEG_JOINT1", "LLEG_JOINT2",
    "LLEG_JOINT3", "LLEG_JOINT4", "LLEG_JOINT5",
    "CHEST_JOINT0", "CHEST_JOINT1",
    "HEAD_JOINT0", "HEAD_JOINT1",
    "RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2",
    "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", "RARM_JOINT6",
    "LARM_JOINT0", "LARM_JOINT1", "LARM_JOINT2",
    "LARM_JOINT3", "LARM_JOINT4", "LARM_JOINT5", "LARM_JOINT6",
    "RHAND_JOINT0", "RHAND_JOINT1",
    "RHAND_JOINT2", "RHAND_JOINT3", "RHAND_JOINT4",
    "LHAND_JOINT0", "LHAND_JOINT1",
    "LHAND_JOINT2", "LHAND_JOINT3", "LHAND_JOINT4",
    0
  };

// Number of dofs in left and right robot hands.
//
// This part is very poorly written: currently, dynamic-graph uses a
// lighter robot model without the hands. This model do not have the
// dofs corresponding to the robot hand dofs.  Therefore we put them
// manually to zero to please ROS.  If this is not the case, some
// packages such as rviz will behave badly.
static const std::size_t handsDofsCount = 10;

namespace dynamicgraph
{
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosJointState, "RosJointState");

  RosJointState::RosJointState (const std::string& n)
    : Entity (n),
      // do not use callbacks, so do not create a useless spinner
      nh_ (rosInit (false)),
      state_ (0, MAKE_SIGNAL_STRING(name, true, "Vector", "state")),
      publisher_ (nh_, "joint_states", 5),
      jointState_ (),
      trigger_ (boost::bind (&RosJointState::trigger, this, _1, _2),
		sotNOSIGNAL,
		MAKE_SIGNAL_STRING(name, true, "int", "trigger")),
      rate_ (ROS_JOINT_STATE_PUBLISHER_RATE),
      lastPublicated_ (ros::Time::now () - rate_ - rate_)
  {
    signalRegistration (state_ << trigger_);
    trigger_.setNeedUpdateFromAllChildren (true);

    // Fill header.
    jointState_.header.seq = 0;
    jointState_.header.stamp.sec = 0;
    jointState_.header.stamp.nsec = 0;
    jointState_.header.frame_id = "odom";
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
	std::size_t s = state_.access (t).size () - 6;

	// Update header.
	++jointState_.header.seq;

	ros::Time now = ros::Time::now ();
	jointState_.header.stamp.sec = now.sec;
	jointState_.header.stamp.nsec = now.nsec;

	// Fill names if needed.
	if (jointState_.name.size () != s + handsDofsCount)
	  {
	    jointState_.name.resize (s + handsDofsCount);
	    for (std::size_t i = 0; i < s + handsDofsCount; ++i)
	      {
		if (dof_names[i] == 0)
		  break;
		jointState_.name[i] = dof_names[i];
	      }
	  }

	// Fill position.
	jointState_.position.resize (s + handsDofsCount);
	for (std::size_t i = 0; i < s; ++i)
	  jointState_.position[i] = state_.access (t) (i + 6);
	for (std::size_t i = 0; i < handsDofsCount; ++i)
	  jointState_.position[i + s] = 0.;

	publisher_.msg_ = jointState_;
	publisher_.unlockAndPublish ();
      }
    return dummy;
  }
} // end of namespace dynamicgraph.
