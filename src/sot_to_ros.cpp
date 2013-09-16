#include "sot_to_ros.hh"
# include <dynamic-graph/linear-algebra.h>

namespace dynamicgraph
{

  const char* SotToRos<double>::signalTypeName = "Double";
  const char* SotToRos<unsigned int>::signalTypeName = "Unsigned";
  const char* SotToRos<dynamicgraph::Matrix>::signalTypeName = "Matrix";
  const char* SotToRos<dynamicgraph::Vector>::signalTypeName = "Vector";
  const char* SotToRos<specific::Vector3>::signalTypeName = "Vector3";
  const char* SotToRos<sot::MatrixHomogeneous>::signalTypeName = "MatrixHomo";
  const char* SotToRos<specific::Twist>::signalTypeName = "Twist";
  const char* SotToRos
  <std::pair<specific::Vector3, dynamicgraph::Vector> >::signalTypeName
  = "Vector3Stamped";
  const char* SotToRos
  <std::pair<sot::MatrixHomogeneous, dynamicgraph::Vector> >::signalTypeName
  = "MatrixHomo";
  const char* SotToRos
  <std::pair<specific::Twist, dynamicgraph::Vector> >::signalTypeName
  = "Twist";

} // end of namespace dynamicgraph.
