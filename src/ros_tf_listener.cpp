#include "dynamic_graph_bridge/ros_init.hh"
#include "ros_tf_listener.hh"

#include <dynamic-graph/factory.h>

namespace dynamicgraph {
namespace internal {
sot::MatrixHomogeneous& TransformListenerData::getTransform(sot::MatrixHomogeneous& res, int time) {
  static const ros::Time rosTime(0);
  try {
    listener.lookupTransform(toFrame, fromFrame, rosTime, transform);
  } catch (const tf::TransformException& ex) {
    res.setIdentity();
    std::ostringstream oss;
    oss << "Enable to get transform at time " << time << ": " << ex.what();
    entity->SEND_WARNING_STREAM_MSG(oss.str());
    return res;
  }
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) res.linear()(r, c) = transform.getBasis().getRow(r)[c];
    res.translation()[r] = transform.getOrigin()[r];
  }
  return res;
}
}  // namespace internal

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosTfListener, "RosTfListener");
}  // namespace dynamicgraph
