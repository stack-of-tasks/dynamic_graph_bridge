/*
 * Copyright 2011,
 * Olivier Stasse,
 *
 * CNRS
 *
 * This file is part of dynamic_graph_bridge.
 * dynamic_graph_bridge is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * dynamic_graph_bridge is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with dynamic_graph_bridge.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

#include "sot_loader.hh"

boost::condition_variable cond;
boost::mutex mut;

void workThread(SotLoader *aSotLoader)
{
  {
    boost::lock_guard<boost::mutex> lock(mut);
  }
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


int main(int argc, char *argv[])
{
  
  ros::init(argc, argv, "sot_ros_encapsulator");

  SotLoader aSotLoader;
  if (aSotLoader.parseOptions(argc,argv)<0)
    return -1;
  
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("start_dynamic_graph", 
                                                  &SotLoader::start_dg,
                                                  &aSotLoader);
  ROS_INFO("Ready to start dynamic graph.");

  boost::thread thr(workThread,&aSotLoader);

  boost::unique_lock<boost::mutex> lock(mut);
  cond.wait(lock);
  ros::spin();
}
