#include <dynamic-graph/python/module.hh>
#include "ros_publish.hh"

namespace dg = dynamicgraph;


BOOST_PYTHON_MODULE(wrap)
{
  bp::import("dynamic_graph");

  dg::python::exposeEntity<dg::RosPublish, bp::bases<dg::Entity>, dg::python::AddCommands>()
    .def("clear", &dg::RosPublish::clear, "Remove all signals writing data to a ROS topic")
    .def("rm", &dg::RosPublish::rm, "Remove a signal writing data to a ROS topic",
        bp::args("signal_name"))
    .def("list", &dg::RosPublish::list, "List signals writing data to a ROS topic")
    ;
}
