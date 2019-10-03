/*
 * controllerUR_node.cpp
 *
 *  Created on: Jun 3, 2017
 *      Author: Dominik Belter
 *   Institute: Poznan University of Technology
 */

#include <ros/ros.h>
#include "../include/controllerUR/controllingUR.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller_ur");
  ros::NodeHandle nodeHandle("~");

  std::string planningGroup;
  nodeHandle.param("/planning_group", planningGroup, std::string("right_arm"));
  controller_ur::ControllingUR controllingUR(nodeHandle, planningGroup);

  // Spin
  ros::AsyncSpinner spinner(8); // Use n threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
