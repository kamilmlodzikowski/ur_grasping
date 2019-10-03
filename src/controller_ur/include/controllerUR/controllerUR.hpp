/*
 * ControllerUR.hpp
 *
 *  Created on: Jun 3, 2017
 *      Author: Dominik Belter
 *   Institute: Poznan University of Technology
 */

#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>


// Boost
#include <boost/thread/recursive_mutex.hpp>

// ROS
#include <ros/ros.h>

namespace controller_ur {

/*!
 * Elevation map stored as grid map handling elevation height, variance, color etc.
 */
class ControllerUR
{
 public:

  /*!
   * Constructor.
   */
  ControllerUR(ros::NodeHandle nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ControllerUR();

  /*!
   * Set the geometry of the elevation map. Clears all the data.
   * @param length the side lengths in x, and y-direction of the elevation map [m].
   * @param resolution the cell size in [m/cell].
   * @param position the 2d position of the elevation map in the elevation map frame [m].
   * @return true if successful.
   */
  void computeMotion(Eigen::Vector3d tipForce);

 private:
  //! ROS nodehandle.
  ros::NodeHandle nodeHandle_;

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Cleans the elevation map data to stay within the specified bounds.
   * @return true if successful.
   */
  bool clean();
};

} /* namespace */
