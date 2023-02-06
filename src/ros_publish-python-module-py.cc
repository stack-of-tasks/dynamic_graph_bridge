#include <dynamic-graph/python/module.hh>

#include "ros_publish.hpp"

namespace dgb = dynamic_graph_bridge;
namespace dg = dynamicgraph;

BOOST_PYTHON_MODULE(wrap) {
  bp::import("dynamic_graph");

  dg::python::exposeEntity<dgb::RosPublish, bp::bases<dg::Entity>,
                           dg::python::AddCommands>()
      .def("clear", &dgb::RosPublish::clear,
           "Remove all signals writing data to a ROS topic")
      .def("rm", &dgb::RosPublish::rm,
           "Remove a signal writing data to a ROS topic",
           bp::args("signal_name"))
      .def("list", &dgb::RosPublish::list,
           "List signals writing data to a ROS topic");
}
