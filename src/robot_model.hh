#ifndef DYNAMIC_GRAPH_BRIDGE_ROBOT_MODEL_HH
# define DYNAMIC_GRAPH_BRIDGE_ROBOT_MODEL_HH

# include <string>

#include <sot-dynamic/dynamic.h>
#include <dynamic-graph/linear-algebra.h>
#include "XmlRpcValue.h"

namespace dynamicgraph
{
  class RosRobotModel;

  /// \brief This entity load either the current model available in
  /// the robot_description parameter or a specified file and build
  /// a Dynamic entity
  ///
  /// This relies on pinocchio urdf parser to load the model and pinocchio
  /// to realize the computation.
  class RosRobotModel : public sot::Dynamic
  {
    DYNAMIC_GRAPH_ENTITY_DECL();
  public:
    RosRobotModel(const std::string& n);
    virtual ~RosRobotModel();

    void loadUrdf(const std::string& filename);
    void setNamespace (const std::string& ns);
    void loadFromParameterServer();
    Vector curConf() const;

    void addJointMapping(const std::string& link, const std::string& repName);

  protected:

    unsigned getDimension () const
    {
      if (!m_data)
	throw std::runtime_error ("no robot loaded");
      //TODO: Configuration vector dimension or the dof?
      return m_model.nv;
      //return m_model.nq;
    }


  private:
    /// \brief Name of the parameter where the joints list will be published
    std::string jointsParameterName_;

    /// \brief Name of the controller namespace
    std::string ns_;

    /// \brief Special joints map for the parser
    std::map<std::string, std::string> specialJoints_;
  };
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_BRIDGE_ROBOT_MODEL_HH
