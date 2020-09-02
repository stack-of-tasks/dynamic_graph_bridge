#include <dynamic-graph/python/module.hh>
#include "ros_publish.hh"

namespace dg = dynamicgraph;


BOOST_PYTHON_MODULE(wrap)
{
  bp::import("dynamic_graph");

  dg::python::exposeEntity<dg::RosPublish, bp::bases<dg::Entity>, dg::python::AddCommands>() ;
}
