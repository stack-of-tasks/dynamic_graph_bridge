#include <boost/bind.hpp>
#include <boost/format.hpp>

#include <dynamic-graph/factory.h>

#include "ros_joint_state.hh"
#include "sot_to_ros.hh"

//FIXME: this is very very wrong but we need to update abstract-robot-dynamics
// to publish joint names.
static const char* dof_names[] =
  {
    "RLEG_JOINT0", "RLEG_JOINT1", "RLEG_JOINT2", "RLEG_JOINT3", "RLEG_JOINT4", "RLEG_JOINT5",
    "LLEG_JOINT0", "LLEG_JOINT1", "LLEG_JOINT2", "LLEG_JOINT3", "LLEG_JOINT4", "LLEG_JOINT5",
    "CHEST_JOINT0", "CHEST_JOINT1",
    "HEAD_JOINT0", "HEAD_JOINT1",
    "RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2", "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", "RARM_JOINT6",
    "LARM_JOINT0", "LARM_JOINT1", "LARM_JOINT2", "LARM_JOINT3", "LARM_JOINT4", "LARM_JOINT5", "LARM_JOINT6",
    0
  };

namespace dynamicgraph
{
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosJointState, "RosJointState");

  static const char* rosInit()
  {
    int argc = 1;
    char* arg0 = strdup("ros_joint_state");
    char* argv[] = {arg0, 0};
    ros::init(argc, argv, "ros_joint_state");
    free (arg0);
    return "dynamic_graph";
  }

  RosJointState::RosJointState (const std::string& n)
    : Entity (n),
      nh_ (rosInit ()),
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
	if (jointState_.name.size () != s)
	  {
	    jointState_.name.resize (s);
	    for (std::size_t i = 0; i < s; ++i)
	      {
		if (dof_names[i] == 0)
		  break;
		jointState_.name[i] = dof_names[i];
	      }
	  }

	// Fill position.
	jointState_.position.resize (s);
	for (std::size_t i = 0; i < s; ++i)
	  jointState_.position[i] = state_.access (t) (i + 6);

	publisher_.msg_ = jointState_;
	publisher_.unlockAndPublish ();
      }
    return dummy;
  }
} // end of namespace dynamicgraph.
