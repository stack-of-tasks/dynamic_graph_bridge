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

#include "sot_loader.hh"

bool SotLoader::start_dg(std_srvs::Empty::Request& request, 
                         std_srvs::Empty::Response& response)
{
  for(unsigned int i=0;i<500;i++)
    {
      oneIteration();
    }
}


int main(int argc, char *argv[])
{
  SotLoader aSotLoader;
  if (aSotLoader.parseOptions(argc,argv)<0)
    return -1;
  
  ros::init(argc, argv, "start_dynamic_graph");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("start_dynamic_graph", 
                                                  &SotLoader::start_dg,
                                                  &aSotLoader);
  ROS_INFO("Ready to start dynamic graph.");
  ros::spin();
}
