#include <boost/bind.hpp>
#include <boost/format.hpp>

#include <dynamic-graph/factory.h>

#include "ros_joint_state.hh"
#include "sot_to_ros.hh"

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
      publisher_ (nh_, "jointState", 5),
      jointState_ (),
      trigger_ (boost::bind (&RosJointState::trigger, this, _1, _2),
		sotNOSIGNAL,
		MAKE_SIGNAL_STRING(name, true, "int", "trigger"))
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
    if (publisher_.trylock ())
      {
	std::size_t s = state_.access (t).size ();

	// Update header.
	++jointState_.header.seq;

	ros::Time now = ros::Time::now ();
	jointState_.header.stamp.sec = now.sec;
	jointState_.header.stamp.nsec = now.nsec;

	// Fill names if needed.
	if (jointState_.name.size () != s)
	  {
	    boost::format fmtFreeFlyer ("free-flyer-%s");
	    boost::format fmtJoint ("joint-%s");
	    jointState_.name.resize (s);
	    for (std::size_t i = 0; i < s; ++i)
	      if (i < 6)
		jointState_.name[i] = (fmtFreeFlyer % i).str ();
	      else
		jointState_.name[i] = (fmtJoint % (i - 6)).str ();
	  }

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
