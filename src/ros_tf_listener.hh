#ifndef DYNAMIC_GRAPH_ROS_TF_LISTENER_HH
#define DYNAMIC_GRAPH_ROS_TF_LISTENER_HH

#include <boost/bind.hpp>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/command-bind.h>

#include <sot/core/matrix-geometry.hh>

#include <dynamic_graph_bridge/ros_init.hh>

namespace dynamicgraph {
class RosTfListener;

namespace internal {
struct TransformListenerData {
  typedef SignalTimeDependent<bool, int> AvailableSignal_t;
  typedef SignalTimeDependent<sot::MatrixHomogeneous, int> MatrixSignal_t;
  typedef SignalPtr<sot::MatrixHomogeneous, int> DefaultSignal_t;

  RosTfListener* entity;
  tf2_ros::Buffer& buffer;
  const std::string toFrame, fromFrame;
  geometry_msgs::TransformStamped transform;
  ros::Duration max_elapsed;
  AvailableSignal_t availableSig;
  MatrixSignal_t signal;
  DefaultSignal_t failbackSig;

  TransformListenerData(RosTfListener* e, tf2_ros::Buffer& b,
                        const std::string& to, const std::string& from,
                        const std::string& signame)
      : entity(e)
      , buffer(b)
      , toFrame(to)
      , fromFrame(from)
      , max_elapsed(0.5)
      , availableSig(signame+"_available")
      , signal(signame)
      , failbackSig(NULL, signame+"_failback")
  {
    signal.setFunction(
        boost::bind(&TransformListenerData::getTransform, this, _1, _2));

    availableSig.setFunction(
        boost::bind(&TransformListenerData::isAvailable, this, _1, _2));
    availableSig.setNeedUpdateFromAllChildren(true);

    failbackSig.setConstant (sot::MatrixHomogeneous::Identity());
    signal.addDependencies(failbackSig << availableSig);
  }

  sot::MatrixHomogeneous& getTransform(sot::MatrixHomogeneous& res, int time);

  bool& isAvailable(bool& isAvailable, int time);
};
}  // namespace internal

class RosTfListener : public Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  typedef internal::TransformListenerData TransformListenerData;

  RosTfListener(const std::string& _name)
    : Entity(_name)
    , buffer()
    , listener(buffer, rosInit(), false)
  {
    std::string docstring =
        "\n"
        "  Add a signal containing the transform between two frames.\n"
        "\n"
        "  Input:\n"
        "    - to  : frame name\n"
        "    - from: frame name,\n"
        "    - signalName: the signal name in dynamic-graph"
        "\n";
    addCommand("add", command::makeCommandVoid3(*this, &RosTfListener::add, docstring));
    docstring =
        "\n"
        "  Set the maximum time delay of the transform obtained from tf.\n"
        "\n"
        "  Input:\n"
        "    - signalName: the signal name,\n"
        "    - delay  : in seconds\n"
        "\n";
    addCommand("setMaximumDelay", command::makeCommandVoid2(*this, &RosTfListener::setMaximumDelay, docstring));
  }

  ~RosTfListener()
  {
    for (Map_t::const_iterator _it = listenerDatas.begin(); _it != listenerDatas.end(); ++_it) delete _it->second;
  }

  void add(const std::string& to, const std::string& from, const std::string& signame)
  {
    if (listenerDatas.find(signame) != listenerDatas.end())
      throw std::invalid_argument("A signal " + signame + " already exists in RosTfListener " + getName());

    boost::format signalName("RosTfListener(%1%)::output(MatrixHomo)::%2%");
    signalName % getName() % signame;

    TransformListenerData* tld =
        new TransformListenerData(this, buffer, to, from, signalName.str());
    signalRegistration(tld->signal << tld->availableSig << tld->failbackSig);
    listenerDatas[signame] = tld;
  }

  void setMaximumDelay(const std::string& signame, const double& max_elapsed)
  {
    if (listenerDatas.count(signame) == 0)
      throw std::invalid_argument("No signal " + signame + " in RosTfListener " + getName());
    listenerDatas[signame]->max_elapsed = ros::Duration(max_elapsed);
  }

 private:
  typedef std::map<std::string, TransformListenerData*> Map_t;
  Map_t listenerDatas;
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener;
};
}  // end of namespace dynamicgraph.

#endif  // DYNAMIC_GRAPH_ROS_TF_LISTENER_HH
