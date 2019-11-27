#include "dynamic_graph_bridge/ros_init.hh"
#include "ros_tf_listener.hh"

#include <dynamic-graph/factory.h>

namespace dynamicgraph {
namespace internal {
sot::MatrixHomogeneous& TransformListenerData::getTransform(
    sot::MatrixHomogeneous& res, int time) {
  static const ros::Time rosTime(0);
  try {
    transform = buffer.lookupTransform(toFrame, fromFrame, rosTime);
  } catch (const tf2::TransformException& ex) {
    res.setIdentity();
    std::ostringstream oss;
    oss << "Enable to get transform at time " << time << ": " << ex.what();
    entity->SEND_WARNING_STREAM_MSG(oss.str());
    return res;
  }

  const geometry_msgs::TransformStamped::_transform_type::_rotation_type&
    quat = transform.transform.rotation;
  const geometry_msgs::TransformStamped::_transform_type::_translation_type&
    trans = transform.transform.translation;
  res.linear() = sot::Quaternion(quat.w, quat.x, quat.y, quat.z).matrix();
  res.translation() << trans.x, trans.y, trans.z;
  return res;
}
}  // namespace internal

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosTfListener, "RosTfListener");
}  // namespace dynamicgraph
