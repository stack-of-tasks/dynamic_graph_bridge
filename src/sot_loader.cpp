/*
 * Copyright 2011,
 * Olivier Stasse,
 *
 * CNRS
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */
/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#include "sot_loader.hh"

// POSIX.1-2001
#include <dlfcn.h>

using namespace std;
using namespace dynamicgraph::sot; 
namespace po = boost::program_options;

SotLoader::SotLoader():
  dynamic_graph_stopped_(true),
  sensorsIn_ (),
  controlValues_ (),
  angleEncoder_ (),
  angleControl_ (),
  forces_ (),
  torques_ (),
  baseAtt_ (),
  accelerometer_ (3),
  gyrometer_ (3)
{
  readSotVectorStateParam();
  initPublication();
}

int SotLoader::initPublication()
{
  ros::NodeHandle n;


  // Prepare message to be published
  joint_pub_ = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  return 0;
}

int SotLoader::readSotVectorStateParam()
{
  ros::NodeHandle n;

  if (!ros::param::has("/sot/state_vector_map"))
    {
      std::cerr<< " Read Sot Vector State Param " << std::endl;
      return 1;
    }

  n.getParam("/sot/state_vector_map", stateVectorMap_);
  ROS_ASSERT(stateVectorMap_.getType() == XmlRpc::XmlRpcValue::TypeArray);
  nbOfJoints_ = stateVectorMap_.size();

  // Prepare joint_state according to robot description.
  joint_state_.name.resize(nbOfJoints_);
  joint_state_.position.resize(nbOfJoints_);

  for (int32_t i = 0; i < stateVectorMap_.size(); ++i) 
   {
     joint_state_.name[i]= static_cast<string>(stateVectorMap_[i]);
   }
  angleEncoder_.resize(nbOfJoints_);
  
  return 0;
}


int SotLoader::parseOptions(int argc, char *argv[])
{ 
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("input-file", po::value<string>(), "library to load")
    ;
    
  
  po::store(po::parse_command_line(argc, argv, desc), vm_);
  po::notify(vm_);    
  
  if (vm_.count("help")) {
    cout << desc << "\n";
    return -1;
  }
  if (!vm_.count("input-file")) {
    cout << "No filename specified\n";
    return -1;
  }
  else
    dynamicLibraryName_= vm_["input-file"].as< string >();
  
  Initialization();
  return 0;
}

void SotLoader::Initialization()
{
  
  // Load the SotRobotBipedController library.
  void * SotRobotControllerLibrary = dlopen( dynamicLibraryName_.c_str(),
                                             RTLD_LAZY | RTLD_GLOBAL );
  if (!SotRobotControllerLibrary) {
    std::cerr << "Cannot load library: " << dlerror() << '\n';
    return ;
  }
  
  // reset errors
  dlerror();
  
  // Load the symbols.
  createSotExternalInterface_t * createSot =
    (createSotExternalInterface_t *) dlsym(SotRobotControllerLibrary, 
                          "createSotExternalInterface");
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
    return ;
  }
  
  // Create robot-controller
  sotController_ = createSot();
  cout <<"Went out from Initialization." << endl;
}

void 
SotLoader::fillSensors(map<string,dgs::SensorValues> & sensorsIn)
{
  
  // Update joint values.w
  sensorsIn["joints"].setName("angle");
  for(unsigned int i=0;i<angleControl_.size();i++)
    angleEncoder_[i] = angleControl_[i];
  sensorsIn["joints"].setValues(angleEncoder_);
  
}

void 
SotLoader::readControl(map<string,dgs::ControlValues> &controlValues)
{

  
  // Update joint values.
  angleControl_ = controlValues["joints"].getValues();

  // Check if the size if coherent with the robot description.
  if (angleControl_.size()!=(unsigned int)nbOfJoints_)
    {
      std::cerr << " angleControl_ and nbOfJoints are different !"
                << std::endl;
      exit(-1);
    }

  // Publish the data.
  joint_state_.header.stamp = ros::Time::now();  
  for(int i=0;i<nbOfJoints_;i++)
    {
      joint_state_.position[i] = angleControl_[i];
    }
  
  joint_pub_.publish(joint_state_);  

  
}

void SotLoader::setup()
{
  fillSensors(sensorsIn_);
  sotController_->setupSetSensors(sensorsIn_);
  sotController_->getControl(controlValues_); 
  readControl(controlValues_); 
}

void SotLoader::oneIteration()
{
  fillSensors(sensorsIn_);
  try 
    {
      sotController_->nominalSetSensors(sensorsIn_);
      sotController_->getControl(controlValues_);
    } 
  catch(std::exception &e) { throw e;} 
  
  readControl(controlValues_);
}


bool SotLoader::start_dg(std_srvs::Empty::Request& request, 
                         std_srvs::Empty::Response& response)
{
  dynamic_graph_stopped_=false;    
  return true;
}

bool SotLoader::stop_dg(std_srvs::Empty::Request& request, 
                         std_srvs::Empty::Response& response)
{
  dynamic_graph_stopped_ = true;
  return true;
}

