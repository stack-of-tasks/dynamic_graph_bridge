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

#include <dynamic_graph_bridge/sot_loader.hh>
#include "dynamic_graph_bridge/ros_init.hh"

// POSIX.1-2001
#include <dlfcn.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

boost::condition_variable cond;

using namespace std;
using namespace dynamicgraph::sot; 
namespace po = boost::program_options;



void workThreadLoader(SotLoader *aSotLoader)
{
  while(aSotLoader->isDynamicGraphStopped())
    {
      usleep(5000);
    }

  while(!aSotLoader->isDynamicGraphStopped())
    {
      aSotLoader->oneIteration();
      usleep(5000);
    }
  cond.notify_all();
  ros::waitForShutdown();
}

SotLoader::SotLoader():
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

void SotLoader::startControlLoop()
{
  boost::thread thr(workThreadLoader, this);
}

void SotLoader::initializeRosNode(int argc, char *argv[])
{
  SotLoaderBasic::initializeRosNode(argc, argv);
  angleEncoder_.resize(nbOfJoints_);
  startControlLoop();
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

  //Debug
  std::map<std::string,dgs::ControlValues>::iterator it = controlValues.begin();
  sotDEBUG (30)<<"ControlValues to be broadcasted:"<<std::endl;
  for(;it!=controlValues.end(); it++){
    sotDEBUG (30)<<it->first<<":";
    std::vector<double> ctrlValues_ = it->second.getValues();
    std::vector<double>::iterator it_d = ctrlValues_.begin();
    for(;it_d!=ctrlValues_.end();it_d++) sotDEBUG (30)<<*it_d<<" ";
    sotDEBUG (30)<<std::endl;
  }
  sotDEBUG (30)<<"End ControlValues"<<std::endl;


  // Check if the size if coherent with the robot description.
  if (angleControl_.size()!=(unsigned int)nbOfJoints_)
    {
      std::cerr << " angleControl_"<<angleControl_.size()
		<< " and nbOfJoints"<<(unsigned int)nbOfJoints_
		<< " are different !"
                << std::endl;
      exit(-1);
    }
  // Publish the data.
  joint_state_.header.stamp = ros::Time::now();  
  for(int i=0;i<nbOfJoints_;i++)
    {
      joint_state_.position[i] = angleControl_[i];
    }
  for(unsigned int i=0;i<parallel_joints_to_state_vector_.size();i++)
    {
      joint_state_.position[i+nbOfJoints_] = 
        coefficient_parallel_joints_[i]*angleControl_[parallel_joints_to_state_vector_[i]];
    }

  joint_pub_.publish(joint_state_);

  //Publish robot pose
  //get the robot pose values
  std::vector<double> poseValue_ = controlValues["baseff"].getValues();

  freeFlyerPose_.setOrigin(tf::Vector3(poseValue_[0],
				       poseValue_[1],
				       poseValue_[2]));
  tf::Quaternion poseQ_(poseValue_[4],
			poseValue_[5],
			poseValue_[6],
			poseValue_[3]);
  freeFlyerPose_.setRotation(poseQ_);
  //Publish
  freeFlyerPublisher_.sendTransform(tf::StampedTransform(freeFlyerPose_,
							 ros::Time::now(),
							 "odom",
							 "base_link"));
  
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


