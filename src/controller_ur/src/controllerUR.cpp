/*
 * controllerUR_node.cpp
 *
 *  Created on: Jun 3, 2017
 *      Author: Dominik Belter
 *   Institute: Poznan University of Technology
 */

#include "../include/controllerUR/controllerUR.hpp"

// Math
#include <math.h>

// ROS Logging
#include <ros/ros.h>

// Eigen
#include <Eigen/Dense>

using namespace std;
//using namespace grid_map;

namespace controller_ur {

ControllerUR::ControllerUR(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle)
{
  readParameters();
}

ControllerUR::~ControllerUR()
{
}

bool ControllerUR::readParameters()
{

}

void computeMotion(Eigen::Vector3d tipForce)
{
  ROS_INFO_STREAM("Compute motion ");
}

bool ControllerUR::clean()
{
  return true;
}


} /* namespace */
