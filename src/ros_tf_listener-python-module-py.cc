#include <dynamic-graph/python/module.hh>

#include "ros_tf_listener.hh"

namespace dg = dynamicgraph;

BOOST_PYTHON_MODULE(wrap) {
  bp::import("dynamic_graph");

  dg::python::exposeEntity<dg::RosTfListener, bp::bases<dg::Entity>,
                           dg::python::AddCommands>()
      .def("add", &dg::RosTfListener::add,
           "Add a signal containing the transform between two frames.",
           bp::args("to_frame_name", "from_frame_name", "out_signal_name"))
      .def("setMaximumDelay", &dg::RosTfListener::setMaximumDelay,
           "Set the maximum time delay of the transform obtained from tf.",
           bp::args("signal_name", "delay_seconds"));
}
