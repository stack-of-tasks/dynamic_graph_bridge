/*
 * Copyright 2016,
 * Olivier Stasse,
 *
 * CNRS
 *
 */
/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#include "dynamic_graph_bridge/sot_loader_basic.hh"
#include "dynamic_graph_bridge/ros_parameter.hpp"

#include <dynamic-graph/pool.h>

#include <exception>

// POSIX.1-2001
#include <dlfcn.h>

using namespace std;
using namespace dynamicgraph::sot;
namespace po = boost::program_options;

SotLoaderBasic::SotLoaderBasic() : dynamic_graph_stopped_(true), sotRobotControllerLibrary_(0) {
  nh_ = dynamic_graph_bridge::get_ros_node("SotLoaderBasic");
}

SotLoaderBasic::~SotLoaderBasic()
{
  dynamic_graph_bridge::ros_shutdown("SotLoaderBasic");
}

void SotLoaderBasic::initialize()
{
}

rclcpp::Node::SharedPtr SotLoaderBasic::returnsNodeHandle() {
  return nh_;
}
int SotLoaderBasic::initPublication() {

  // Prepare message to be published
  joint_pub_ = nh_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

  return 0;
}

void SotLoaderBasic::initializeServices() {
  RCLCPP_INFO(rclcpp::get_logger("dynamic_graph_bridge"),
               "Ready to start dynamic graph.");

  using namespace std::placeholders;
  service_start_ = nh_->create_service<std_srvs::srv::Empty>("start_dynamic_graph",
                                                             std::bind(&SotLoaderBasic::start_dg,
                                                                       this,std::placeholders::_1,
                                                                       std::placeholders::_2));

  service_stop_ = nh_->create_service<std_srvs::srv::Empty>("stop_dynamic_graph",
                                                            std::bind(&SotLoaderBasic::stop_dg,
                                                                      this,std::placeholders::_1,
                                                                      std::placeholders::_2));

  dynamicgraph::parameter_server_read_robot_description(nh_);
}

void SotLoaderBasic::setDynamicLibraryName(std::string& afilename) { dynamicLibraryName_ = afilename; }

int SotLoaderBasic::readSotVectorStateParam() {
  std::map<std::string, int> from_state_name_to_state_vector;
  std::map<std::string, std::string> from_parallel_name_to_state_vector_name;

  if (!nh_)    {
      throw std::logic_error("SotLoaderBasic::readSotVectorStateParam() nh_ not initialized");
  }

  // It is necessary to declare parameters first
  // before trying to access them.
  if (not nh_->has_parameter("state_vector_map"))
    nh_->declare_parameter("state_vector_map",std::vector<std::string>{});
  if (not nh_->has_parameter("joint_state_parallel"))
    nh_->declare_parameter("joint_state_parallel",std::vector<std::string>{});

  // Read the state vector of the robot
  // Defines the order in which the actuators are ordered
  try {
    std::string aParameterName("state_vector_map");
    if (!nh_->get_parameter(aParameterName,stateVectorMap_)) {
      logic_error aLogicError(
        "SotLoaderBasic::readSotVectorStateParam : State_vector_map is empty");
      throw aLogicError;
    }
    RCLCPP_INFO(nh_->get_logger(), "state_vector_map parameter size %d",
                stateVectorMap_.size());
  }
  catch (exception &e)
    {
      std::throw_with_nested(std::logic_error(
        "Unable to call nh_->get_parameter"));
    }

  nbOfJoints_ = static_cast<int>(stateVectorMap_.size());
  nbOfParallelJoints_ = 0;

  // Read the parallel joints.
  // Specify the constraint between the joints
  // Currently acts as a mimic or a an inverse mimic joint.

  std::string prefix("joint_state_parallel");
  std::map<std::string,rclcpp::Parameter> joint_state_parallel;
  nh_->get_parameters(prefix,joint_state_parallel);

  // Iterates over the map joint_state_parallel
  for ( std::map<std::string,rclcpp::Parameter>::iterator
          it_map_expression = joint_state_parallel.begin();
        it_map_expression != joint_state_parallel.end();
        ++it_map_expression)
    {
      std::string key = it_map_expression->first;
      std::string map_expression = it_map_expression->second.as_string();
      std::string final_expression;
      double final_coefficient = 1.0;
      // deal with sign
      if (map_expression[0] == '-') {
        final_expression = map_expression.substr(1, map_expression.size() - 1);
        final_coefficient = -1.0;
      } else
        final_expression = map_expression;

      std::cout << key << ": " << final_coefficient << std::endl;
      from_parallel_name_to_state_vector_name[key] = final_expression;
      coefficient_parallel_joints_.push_back(final_coefficient);
    }
  nbOfParallelJoints_ = from_parallel_name_to_state_vector_name.size();


  // Prepare joint_state according to robot description.
  joint_state_.name.resize(static_cast<unsigned long>(nbOfJoints_) + nbOfParallelJoints_);
  joint_state_.position.resize(static_cast<unsigned long>(nbOfJoints_) + nbOfParallelJoints_);

  // Fill in the name of the joints from the state vector.
  // and build local map from state name to state vector
  for (std::size_t i = 0; i < stateVectorMap_.size(); ++i) {
    joint_state_.name[i] = stateVectorMap_[i];

    from_state_name_to_state_vector[joint_state_.name[i]] = static_cast<int>(i);
  }

  // Fill in the name of the joints from the parallel joint vector.
  // and build map from parallel name to state vector
  std::size_t i = 0;
  parallel_joints_to_state_vector_.resize(nbOfParallelJoints_);
  for (std::map<std::string, std::string>::iterator it = from_parallel_name_to_state_vector_name.begin();
       it != from_parallel_name_to_state_vector_name.end(); it++, i++) {
    joint_state_.name[i + static_cast<std::size_t>(nbOfJoints_)] = it->first.c_str();
    parallel_joints_to_state_vector_[i] = from_state_name_to_state_vector[it->second];
  }

  return 0;
}

int SotLoaderBasic::parseOptions(int argc, char* argv[]) {
  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")("input-file", po::value<string>(), "library to load");

  vm_.clear();

  po::store(po::parse_command_line(argc, argv, desc), vm_);
  po::notify(vm_);

  if (vm_.count("help")) {
    cout << desc << "\n";
    return -1;
  }
  if (!vm_.count("input-file")) {
    cout << "No filename specified\n";
    return -1;
  } else
    dynamicLibraryName_ = vm_["input-file"].as<string>();

  return 0;
}

void SotLoaderBasic::loadController() {
  // Load the SotRobotBipedController library.
  sotRobotControllerLibrary_ = dlopen(dynamicLibraryName_.c_str(), RTLD_LAZY | RTLD_GLOBAL);
  if (!sotRobotControllerLibrary_) {
    std::cerr << "Cannot load library: " << dlerror() << '\n';
    return;
  }

  // reset errors
  dlerror();

  // Load the symbols.
  createSotExternalInterface_t* createSot = reinterpret_cast<createSotExternalInterface_t*>(
      reinterpret_cast<long>(dlsym(sotRobotControllerLibrary_, "createSotExternalInterface")));
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    std::cerr << "Cannot load symbol create: " << dlsym_error
              << " from " << dynamicLibraryName_
              << '\n';
    return;
  }

  // Create robot-controller
  sotController_ = createSot();
  cout << "Went out from loadController." << endl;
}

void SotLoaderBasic::CleanUp() {
  dynamicgraph::PoolStorage::destroy();
  // We do not destroy the FactoryStorage singleton because the module will not
  // be reloaded at next initialization (because Python C API cannot safely
  // unload a module...).
  // SignalCaster singleton could probably be destroyed.

  // Load the symbols.
  destroySotExternalInterface_t* destroySot = reinterpret_cast<destroySotExternalInterface_t*>(
      reinterpret_cast<long>(dlsym(sotRobotControllerLibrary_, "destroySotExternalInterface")));
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    std::cerr << "Cannot load symbol destroy: " << dlsym_error << '\n';
    return;
  }

  destroySot(sotController_);
  sotController_ = NULL;

  /// Uncount the number of access to this library.
  try {
    if (sotRobotControllerLibrary_ != 0)
      dlclose(sotRobotControllerLibrary_);
  }
  catch(exception &e) {
    std::throw_with_nested(std::logic_error("SotLoaderBasic::CleanUp() Unable to call dlclose"));
  }
}

void SotLoaderBasic::start_dg(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                              std::shared_ptr<std_srvs::srv::Empty::Response>) {
  dynamic_graph_stopped_ = false;
}

void SotLoaderBasic::stop_dg(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                             std::shared_ptr<std_srvs::srv::Empty::Response>) {
  dynamic_graph_stopped_ = true;
}