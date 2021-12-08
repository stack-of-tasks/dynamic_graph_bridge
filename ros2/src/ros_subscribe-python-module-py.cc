#include <dynamic-graph/python/module.hh>
#include "ros_subscribe.hpp"

namespace dgb = dynamic_graph_bridge;
namespace dg = dynamicgraph;

BOOST_PYTHON_MODULE(wrap)
{
  bp::import("dynamic_graph");

  dg::python::exposeEntity<dgb::RosSubscribe, bp::bases<dg::Entity>,
                           dg::python::AddCommands>()
      .def("clear", &dgb::RosSubscribe::clear,
           "Remove all signals reading data from a ROS topic")
      .def("rm", &dgb::RosSubscribe::rm,
           "Remove a signal reading data from a ROS topic",
           bp::args("signal_name"))
      .def("list", &dgb::RosSubscribe::list,
           "List signals reading data from a ROS topic");
}
