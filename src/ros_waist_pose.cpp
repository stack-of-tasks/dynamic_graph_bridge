#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>
#include <sot-dynamic/dynamic.h>

#include "dynamic_graph_bridge/ros_init.hh"
#include "ros_waist_pose.hh"
#include "sot_to_ros.hh"

#include <tf/transform_broadcaster.h>

namespace dynamicgraph
{
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosWaistPose, "RosWaistPose");

  RosWaistPose::RosWaistPose (const std::string& n)
    : Entity (n),
      // do not use callbacks, so do not create a useless spinner
      nh_ (rosInit (false)),
      state_ (0, MAKE_SIGNAL_STRING(name, true, "Vector", "state")),
      publisher_ (nh_, "dynamic_graph/waist_pose", 5),
      waistPose_ (),
      trigger_ (boost::bind (&RosWaistPose::trigger, this, _1, _2),
		sotNOSIGNAL,
		MAKE_SIGNAL_STRING(name, true, "int", "trigger")),
      rate_ (ROS_WAIST_POSE_PUBLISHER_RATE),
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
    waistPose_.header.seq = 0;
    waistPose_.header.stamp.sec = 0;
    waistPose_.header.stamp.nsec = 0;
    waistPose_.header.frame_id = "odom";
    waistPose_.child_frame_id = "base_link";
  }

  RosWaistPose::~RosWaistPose ()
  {}

  int&
  RosWaistPose::trigger (int& dummy, int t)
  {
    ros::Duration dt = ros::Time::now () - lastPublicated_;
    if (dt > rate_ && publisher_.trylock ())
      {
	lastPublicated_ = ros::Time::now ();

	// State size without the free floating.
	std::size_t s = state_.access (t).size ();

	// Safety check
	if (s<6) return dummy;

	// Update header.
	++waistPose_.header.seq;

	ros::Time now = ros::Time::now ();
	waistPose_.header.stamp.sec = now.sec;
	waistPose_.header.stamp.nsec = now.nsec;

        // Fill pose
	waistPose_.transform.translation.x = state_.access(t)(0);
	waistPose_.transform.translation.y = state_.access(t)(1);
	waistPose_.transform.translation.z = state_.access(t)(2);
	waistPose_.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(
			state_.access(t)(3),
			state_.access(t)(4),
			state_.access(t)(5));

	publisher_.msg_ = waistPose_;
	publisher_.unlockAndPublish ();
      }
    return dummy;
  }
} // end of namespace dynamicgraph.
