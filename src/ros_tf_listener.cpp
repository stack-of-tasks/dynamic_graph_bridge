#include "ros_tf_listener.hh"

#include <dynamic-graph/factory.h>

#include <pinocchio/fwd.hpp>

#include "dynamic_graph_bridge/ros_init.hh"

namespace dynamicgraph {
namespace internal {
sot::MatrixHomogeneous& TransformListenerData::getTransform(
    sot::MatrixHomogeneous& res, int time) {
  availableSig.recompute(time);

  bool available = availableSig.accessCopy();

  if (!available) {
    failbackSig.recompute(time);
    res = failbackSig.accessCopy();
    return res;
  }

  const geometry_msgs::TransformStamped::_transform_type::_rotation_type& quat =
      transform.transform.rotation;
  const geometry_msgs::TransformStamped::_transform_type::_translation_type&
      trans = transform.transform.translation;
  res.linear() = sot::Quaternion(quat.w, quat.x, quat.y, quat.z).matrix();
  res.translation() << trans.x, trans.y, trans.z;
  return res;
}

bool& TransformListenerData::isAvailable(bool& available, int time) {
  static const ros::Time origin(0);
  available = false;
  ros::Duration elapsed;
  std::string msg;

  if (buffer.canTransform(toFrame, fromFrame, origin, &msg)) {
    transform = buffer.lookupTransform(toFrame, fromFrame, origin);
    if (transform.header.stamp == origin) {
      // This is likely a TF2 static transform.
      available = true;
    } else {
      elapsed = ros::Time::now() - transform.header.stamp;
      available = (elapsed <= max_elapsed);
    }

    if (!available) {
      std::ostringstream oss;
      oss << "Use failback " << signal.getName() << " at time " << time
          << ". Time since last update of the transform: " << elapsed;
      entity->SEND_INFO_STREAM_MSG(oss.str());
    }
  } else {
    std::ostringstream oss;
    oss << "Unable to get transform " << signal.getName() << " at time " << time
        << ": " << msg;
    entity->SEND_WARNING_STREAM_MSG(oss.str());
    available = false;
  }
  return available;
}

}  // namespace internal

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosTfListener, "RosTfListener");
}  // namespace dynamicgraph
