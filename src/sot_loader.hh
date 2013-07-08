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

// System includes
#include <iostream>

// STL includes
#include <map>

// Boost includes
#include <boost/program_options.hpp>

// ROS includes
#include "ros/ros.h"
#include "std_srvs/Empty.h"
// Sot Framework includes 
#include <sot/core/debug.hh>
#include <sot/core/abstract-sot-external-interface.hh>

namespace po = boost::program_options;
namespace dgs = dynamicgraph::sot;

//typedef std::vector<double> SensorValues;
//typedef std::vector<double> ControlValues;

class SotLoader {

protected:

  /// \brief the sot-hrp2 controller
  dgs::AbstractSotExternalInterface * sotController_;

  po::variables_map vm_;
  std::string dynamicLibraryName_;

  /// Map of sensor readings
  std::map <std::string,dgs::SensorValues> sensorsIn_;
  /// Map of control values
  std::map<std::string,dgs::ControlValues> controlValues_;
  /// Angular values read by encoders
  std::vector <double> angleEncoder_;
  /// Angular values sent to motors
  std::vector<double> angleControl_;
  /// Forces read by force sensors
  std::vector<double> forces_;
  /// Torques
  std::vector<double> torques_;
  /// Attitude of the robot computed by extended Kalman filter.
  std::vector<double> baseAtt_;
  /// Accelerations read by Accelerometers
  std::vector <double> accelerometer_;
  /// Angular velocity read by gyrometers
  std::vector <double> gyrometer_;

public:
  SotLoader();
  ~SotLoader() {};

  int parseOptions(int argc, char *argv[]);
  
  void Initialization();
  void oneIteration();

  void fillSensors(std::map<std::string, dgs::SensorValues> & sensorsIn);
  void readControl(std::map<std::string, dgs::ControlValues> & controlValues);
  void setup();
  bool start_dg(std_srvs::Empty::Request& request, 
                std_srvs::Empty::Response& response);
};

