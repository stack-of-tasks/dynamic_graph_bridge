#include "sot_to_ros.hh"

namespace dynamicgraph
{

  const char* SotToRos<double>::signalTypeName = "Double";
  const char* SotToRos<ml::Matrix>::signalTypeName = "Matrix";
  const char* SotToRos<ml::Vector>::signalTypeName = "Vector";
  const char* SotToRos<specific::Vector3>::signalTypeName = "Vector3";
  const char* SotToRos<sot::MatrixHomogeneous>::signalTypeName = "MatrixHomo";
  const char* SotToRos<specific::Twist>::signalTypeName = "Twist";
  const char* SotToRos<sot::Trajectory>::signalTypeName = "Trajectory";
  const char* SotToRos
  <std::pair<specific::Vector3, ml::Vector> >::signalTypeName
  = "Vector3Stamped";
  const char* SotToRos
  <std::pair<sot::MatrixHomogeneous, ml::Vector> >::signalTypeName
  = "MatrixHomo";
  const char* SotToRos
  <std::pair<specific::Twist, ml::Vector> >::signalTypeName
  = "Twist";

} // end of namespace dynamicgraph.
