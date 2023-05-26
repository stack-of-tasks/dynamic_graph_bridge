#include <dynamic-graph/python/module.hh>

#include "ros_queued_subscribe.hh"

namespace dg = dynamicgraph;

BOOST_PYTHON_MODULE(wrap) {
  bp::import("dynamic_graph");

  dg::python::exposeEntity<dg::RosQueuedSubscribe, bp::bases<dg::Entity>,
                           dg::python::AddCommands>()
      .def("clear", &dg::RosQueuedSubscribe::clear,
           "Remove all signals reading data from a ROS topic")
      .def("rm", &dg::RosQueuedSubscribe::rm,
           "Remove a signal reading data from a ROS topic",
           bp::args("signal_name"))
      .def("list", &dg::RosQueuedSubscribe::list,
           "List signals reading data from a ROS topic")
      .def("listTopics", &dg::RosQueuedSubscribe::listTopics,
           "List subscribed topics from ROS in the same order as list command")
      .def("clearQueue", &dg::RosQueuedSubscribe::clearQueue,
           "Empty the queue of a given signal", bp::args("signal_name"))
      .def("queueSize", &dg::RosQueuedSubscribe::queueSize,
           "Return the queue size of a given signal", bp::args("signal_name"))
      .def("readQueue", &dg::RosQueuedSubscribe::readQueue,
           "Whether signals should read values from the queues, and when.",
           bp::args("start_time"))
      .def("queueReceivedData", &dg::RosQueuedSubscribe::queueReceivedData,
           "Check whether the queue of a given signal has received atleast one data point", 
           bp::args("signal_name"))
      .def("setQueueReceivedData", &dg::RosQueuedSubscribe::setQueueReceivedData,
           "Set the data reception status of the queue corresponding to a given signal", 
           bp::args("signal_name","signal_data_acq_status"));
}
