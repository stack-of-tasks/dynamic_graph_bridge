#ifndef DYNAMIC_GRAPH_ROS_TF_LISTENER_HH
# define DYNAMIC_GRAPH_ROS_TF_LISTENER_HH

# include <boost/bind.hpp>

# include <tf/transform_listener.h>

# include <dynamic-graph/entity.h>
# include <dynamic-graph/signal.h>
# include <dynamic-graph/command-bind.h>

# include <sot/core/matrix-geometry.hh>

namespace dynamicgraph {
  class RosTfListener;

  namespace internal
  {
    struct TransformListenerData {
      typedef Signal<sot::MatrixHomogeneous, int> signal_t;

      RosTfListener* entity;
      tf::TransformListener& listener;
      const std::string toFrame, fromFrame;
      tf::StampedTransform transform;
      signal_t signal;

      TransformListenerData (RosTfListener* e,
          tf::TransformListener& l,
          const std::string& to, const std::string& from,
          const std::string& signame)
        : entity (e)
        , listener (l)
        , toFrame (to)
        , fromFrame (from)
        , signal (signame)
      {
        signal.setFunction (boost::bind(&TransformListenerData::getTransform, this, _1, _2));
      }

      sot::MatrixHomogeneous& getTransform (sot::MatrixHomogeneous& res, int time);
    };
  } // end of internal namespace.

  class RosTfListener : public Entity
  {
    DYNAMIC_GRAPH_ENTITY_DECL();

    public:
      typedef internal::TransformListenerData TransformListenerData; 

      RosTfListener (const std::string& name) : Entity (name)
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
        addCommand ("add",
            command::makeCommandVoid3(*this, &RosTfListener::add, docstring));
      }

      ~RosTfListener ()
      {
        for (Map_t::const_iterator _it = listenerDatas.begin(); _it != listenerDatas.end(); ++_it)
          delete _it->second;
      }

      void add (const std::string& to, const std::string& from, const std::string& signame)
      {
        if (listenerDatas.find(signame) != listenerDatas.end())
          throw std::invalid_argument ("A signal " + signame
              + " already exists in RosTfListener " + getName());

        boost::format signalName ("RosTfListener(%1%)::output(MatrixHomo)::%2%");
        signalName % getName () % signame;

        TransformListenerData* tld = new TransformListenerData (
            this, listener, to, from, signalName.str());
        signalRegistration (tld->signal);
        listenerDatas[signame] = tld;
      }

    private:
      typedef std::map<std::string, TransformListenerData*> Map_t;
      Map_t listenerDatas;
      tf::TransformListener listener;
  };
} // end of namespace dynamicgraph.

#endif // DYNAMIC_GRAPH_ROS_TF_LISTENER_HH
