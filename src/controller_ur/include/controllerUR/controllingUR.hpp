/*
 * controllingUR.hpp
 *
 *  Created on: Jul 5, 2017
 *      Author: Dominik Belter
 *	 Institute: PUT Poznan
 */

#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/WrenchStamped.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <control_msgs/JointTrajectoryControllerState.h> //read state of the arm
#include <control_msgs/FollowJointTrajectoryAction.h> //set goal positions
#include <control_msgs/FollowJointTrajectoryActionResult.h> // result
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

//MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>

// pluginlib
#include <pluginlib/class_loader.h>

// Boost
#include <boost/thread.hpp>

// standard
#include <mutex>
#include <iostream>
#include <thread>
#include <iomanip>
#include <fstream>
#include <iostream>

namespace controller_ur {

/*!
 * The elevation mapping main class. Coordinates the ROS interfaces, the timing,
 * and the data handling between the other classes.
 */
class ControllingUR
{
 public:

    /// ControllerState
    enum ControllerState {
        STATE_INITIALIZATION,
        STATE_MOVE2INIT,
        STATE_NORMAL,
        STATE_MOVING,
    };

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ControllingUR(ros::NodeHandle& nodeHandle, std::string groupName);

  /*!
   * Destructor.
   */
  virtual ~ControllingUR();

  /*!
   * Callback function for new motion order.
   * @param pointCloud the point cloud to be fused with the existing data.
   */
  void motionEventCallback(const geometry_msgs::WrenchStamped& tipForce);

  /// received string order
  void orderArmCallback(const std_msgs::String::ConstPtr& msg);

  /*!
   * Callback function for the update timer. Forces an update of the controller if no new measurements are received for a certain time
   * period.
   * @param timerEvent the timer event.
   */
  void updateTimerCallback(const ros::TimerEvent& timerEvent);

  /// force sensor callback
  void forceSensorCallback(const geometry_msgs::WrenchStamped& tipForce);

  /// controller state result
  void controllerStateCallbackSim(const control_msgs::JointTrajectoryControllerState::ConstPtr& jointState);
  void graspCallback(const geometry_msgs::Pose& pose);
  void controllerStateCallback(const sensor_msgs::JointState::ConstPtr& jointState);
  void controllerResultCallback(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& jointResult);
  void trajectoryCallback(const nav_msgs::PathConstPtr& trajectory);
  /*!
   * Clears all data of the controller and moves the arm to the default position.
   * @param request the ROS service request.
   * @param response the ROS service response.
   * @return true if successful.
   */
  bool resetRobot(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

 private:
  //moveIt
  robot_model_loader::RobotModelLoader robot_model_loader;
  /// kinematic model of the robot
  robot_model::RobotModelPtr kinematic_model;
  /// kinematic state
  moveit::core::RobotStatePtr kinematic_state;
  /// joints
  const robot_state::JointModelGroup* joint_model_group;
  /// maxJointErrorTolerance
  double maxJointErrorTolerance;
  /// ompl planner
  planning_interface::PlannerManagerPtr planner_instance;
  /// planning scene
  planning_scene::PlanningScenePtr planningScene;
  /// planning
  moveit::planning_interface::MoveGroupInterface moveGroup;

  void setReferenceValues(std::vector<double>& refValues, double executionTime);

  std::ofstream logFileForce;
  std::ofstream logFileRobotState;

    std::vector<double> currentJointsState;
    std::vector<double> referenceJointsVals;

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Performs the initialization procedure.
   * @return true if successful.
   */
  bool initialize();

  /// initialize opml library
  void initializeMoveItOMPL(void);

  /*!
   * Reset and start the update timer.
   */
  void resetUpdateTimer();

  /*!
   * Stop the update timer.
   */
  void stopUpdateTimer();

  /// move tool
  void moveTool(const Eigen::Affine3d& _motion);

  /// move tool
  void moveToolOMPL(const Eigen::Affine3d& _motion);

  bool moveToInit(void);

  bool moveToPlace(std::vector<double> placeConf);

  /// get max joint error
  double getMaxJointError(void);

  void saveRobotState(void);

  void processStateCallback(void);

  /// execute trajectory with waypoints
  void planAndExecuteTrajectory(const std::vector<Eigen::Affine3d>& motions);

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! ROS subscribers.
  ros::Subscriber forceSensorSubscriber_;
  ros::Subscriber graspSubscriber_;
  ros::Subscriber controllerStateSubscriber_;
  ros::Subscriber controllerResultSubscriber_;
  ros::Subscriber trajectorySubscriber_;
  // subsribe orders topic
  ros::Subscriber subOrdersArm;

//  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> robotPoseSubscriber_;

  //! ROS service servers.
  ros::ServiceServer resetRobotService_;

  /// ROS publishers
  ros::Publisher robotControllerPublisher;
  /// ROS publishers - arm controller state
  ros::Publisher armControllerStatePub;

  /// robot control messages
  control_msgs::FollowJointTrajectoryActionGoal ctrlMsg;

  /// contact force
  Eigen::Vector3d contactForce;

  std::mutex contactForceMtx;

  //! ROS topics for subscriptions.
  std::string forceSensorTopic_;
  /// ROS topic for controller state
  std::string controllerStateTopic_;
  /// ROS topic for controller result
  std::string controllerResultTopic_;
  /// ROS topic for trajectory
  std::string trajectoryTopic_;
  /// use simulation
  int useSim;

  /// ROS topic for joint goal
  std::string jointGoalTopic_;

  //! Timer for the robot motion update.
  ros::Timer updateTimer_;

  //! Maximum time that the map will not be updated.
  ros::Duration maxNoUpdateDuration_;

  ros::Time timeOfLastUpdate;

  /// controller status
  int controllerStatus;

  ControllerState ctrlState;

  std::string planningGroup;
  std::string armPrefix;

  std::vector<Eigen::Affine3d> trajectory;
  int trajectoryPointNo;
};

} /* namespace */
