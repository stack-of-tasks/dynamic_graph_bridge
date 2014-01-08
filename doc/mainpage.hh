/// \mainpage
///
/// \section Introduction
///
/// This package implements a bridge between ROS and the stack of tasks.
/// The main features are
///   \li A service <c>Run_command</c> that enables users to communicate with
///       a remote python interpreter in order to build and control a graph,
///   \li A class <c>dynamicgraph::RosRobotModel</c> deriving from
///       <c>dynamicgraph::sot::Dynamic</c> that implements an entity computing
///       forward kinematics and dynamics based on a urdf file.  for
///       more information, type in a python terminal
/// \code
/// from dynamic_graph.ros.robot_model import RosRobotModel
/// model = RosRobotModel ("model")
/// model.help ()
/// \endcode
