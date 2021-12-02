#include <stdexcept>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include "dynamic_graph_bridge/ros_init.hh"

namespace dynamicgraph {
struct GlobalRos {
  ~GlobalRos() {
    if (spinner) spinner->stop();
    if (nodeHandle) nodeHandle->shutdown();
  }

  boost::shared_ptr<ros::NodeHandle> nodeHandle;
  boost::shared_ptr<ros::AsyncSpinner> spinner;
  boost::shared_ptr<ros::MultiThreadedSpinner> mtSpinner;
};
GlobalRos ros;

ros::NodeHandle& rosInit(bool createAsyncSpinner, bool createMultiThreadedSpinner) {
  if (!ros.nodeHandle) {
    int argc = 1;
    char* arg0 = strdup("dynamic_graph_bridge");
    char* argv[] = {arg0, 0};
    ros::init(argc, argv, "dynamic_graph_bridge");
    free(arg0);

    ros.nodeHandle = boost::make_shared<ros::NodeHandle>("");
  }
  if (!ros.spinner && createAsyncSpinner) {
    ros.spinner = boost::make_shared<ros::AsyncSpinner>(4);

    // Change the thread's scheduler from real-time to normal and reduce its
    // priority
    int oldThreadPolicy, newThreadPolicy;
    struct sched_param oldThreadParam, newThreadParam;
    if (pthread_getschedparam(pthread_self(), &oldThreadPolicy, &oldThreadParam) == 0) {
      newThreadPolicy = SCHED_OTHER;
      newThreadParam = oldThreadParam;
      newThreadParam.sched_priority -= 5;  // Arbitrary number, TODO: choose via param/file?
      if (newThreadParam.sched_priority < sched_get_priority_min(newThreadPolicy))
        newThreadParam.sched_priority = sched_get_priority_min(newThreadPolicy);

      pthread_setschedparam(pthread_self(), newThreadPolicy, &newThreadParam);
    }

    // AsyncSpinners are created with the reduced priority
    ros.spinner->start();

    // Switch the priority of the parent thread (this thread) back to real-time.
    pthread_setschedparam(pthread_self(), oldThreadPolicy, &oldThreadParam);
  } else {
    if (!ros.mtSpinner && createMultiThreadedSpinner) {
      // Seems not to be used.
      // If we need to reduce its threads priority, it needs to be done before
      // calling the MultiThreadedSpinner::spin() method
      ros.mtSpinner = boost::make_shared<ros::MultiThreadedSpinner>(4);
    }
  }
  return *ros.nodeHandle;
}

ros::AsyncSpinner& spinner() {
  if (!ros.spinner) throw std::runtime_error("spinner has not been created");
  return *ros.spinner;
}

ros::MultiThreadedSpinner& mtSpinner() {
  if (!ros.mtSpinner) throw std::runtime_error("spinner has not been created");
  return *ros.mtSpinner;
}

}  // end of namespace dynamicgraph.
