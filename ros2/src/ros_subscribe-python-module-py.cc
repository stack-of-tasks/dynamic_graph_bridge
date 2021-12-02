#include <dynamic-graph/python/module.hh>
#include "ros_subscribe.hh"

namespace dg = dynamicgraph;

BOOST_PYTHON_MODULE(wrap)
{
  bp::import("dynamic_graph");

  dg::python::exposeEntity<dg::RosSubscribe, bp::bases<dg::Entity>,
                           dg::python::AddCommands>()
      .def("clear", &dg::RosSubscribe::clear,
           "Remove all signals reading data from a ROS topic")
      .def("rm", &dg::RosSubscribe::rm,
           "Remove a signal reading data from a ROS topic",
           bp::args("signal_name"))
      .def("list", &dg::RosSubscribe::list,
           "List signals reading data from a ROS topic");
}
