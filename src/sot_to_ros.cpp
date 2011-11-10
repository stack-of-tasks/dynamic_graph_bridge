#include "sot_to_ros.hh"

namespace dynamicgraph
{

  const char* SotToRos<double>::signalTypeName = "Double";
  const char* SotToRos<ml::Matrix>::signalTypeName = "Matrix";
  const char* SotToRos<ml::Vector>::signalTypeName = "Vector";
  const char* SotToRos<sot::MatrixHomogeneous>::signalTypeName = "MatrixHomo";
  const char* SotToRos
  <std::pair<sot::MatrixHomogeneous, ml::Vector> >::signalTypeName
  = "MatrixHomo";

} // end of namespace dynamicgraph.
