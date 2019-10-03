/*
 * controllerUR_node.cpp
 *
 *  Created on: Jun 3, 2017
 *      Author: Dominik Belter
 *   Institute: Poznan University of Technology
 */

#include "../include/controllerUR/controllingUR.hpp"

// Boost
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <string>
#include <math.h>
#include <limits>
#include <thread>

using namespace std;
using namespace ros;
using namespace tf;

double maxContactForce=1;
std::vector<double> initConf = {2.9,-1.9, -1.75,0.0,1.5,0.75};

namespace controller_ur {

ControllingUR::ControllingUR(ros::NodeHandle& nodeHandle, std::string groupName)
    : nodeHandle_(nodeHandle), robot_model_loader("robot_description"), ctrlState(STATE_INITIALIZATION), controllerStatus(3),
      moveGroup(groupName)
{
    ROS_INFO("Controlling UR node started."); 

    readParameters();
    currentJointsState.resize(6);
    std::fill(currentJointsState.begin(), currentJointsState.end(), 0);
    graspSubscriber_= nodeHandle_.subscribe("/grasp_pose",1, &ControllingUR::graspCallback, this);
    forceSensorSubscriber_ = nodeHandle_.subscribe(forceSensorTopic_, 1, &ControllingUR::forceSensorCallback, this);
    if (useSim)
        controllerStateSubscriber_ = nodeHandle_.subscribe(controllerStateTopic_, 1, &ControllingUR::controllerStateCallbackSim, this);
    else
        controllerStateSubscriber_ = nodeHandle_.subscribe(controllerStateTopic_, 1, &ControllingUR::controllerStateCallback, this);
//     std::cout << "controllerStateTopic_ " << controllerStateTopic_ << "\n";

    controllerResultSubscriber_ = nodeHandle_.subscribe(controllerResultTopic_, 1, &ControllingUR::controllerResultCallback, this);
    //orders
    subOrdersArm = nodeHandle_.subscribe("/controller_ur/order", 1, &ControllingUR::orderArmCallback, this);//subscribe orders topic

    std::cout << "trajectoryTopic_ " << trajectoryTopic_ << "\n";
    trajectorySubscriber_ = nodeHandle_.subscribe(trajectoryTopic_, 1, &ControllingUR::trajectoryCallback, this);

    robotControllerPublisher = nodeHandle_.advertise<control_msgs::FollowJointTrajectoryActionGoal>(jointGoalTopic_, 1000); //simulator

    armControllerStatePub = nodeHandle_.advertise<std_msgs::String>("/controller_ur/currentState", 20); //controller state

    timeOfLastUpdate = ros::Time::now();
    updateTimer_ = nodeHandle_.createTimer(maxNoUpdateDuration_, &ControllingUR::updateTimerCallback, this, true, false);

    ctrlMsg.goal.trajectory.joint_names.push_back(armPrefix+"shoulder_pan_joint");
    ctrlMsg.goal.trajectory.joint_names.push_back(armPrefix+"shoulder_lift_joint");
    ctrlMsg.goal.trajectory.joint_names.push_back(armPrefix+"elbow_joint");
    ctrlMsg.goal.trajectory.joint_names.push_back(armPrefix+"wrist_1_joint");
    ctrlMsg.goal.trajectory.joint_names.push_back(armPrefix+"wrist_2_joint");
    ctrlMsg.goal.trajectory.joint_names.push_back(armPrefix+"wrist_3_joint");

    kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    // create logger
    // Getting the starting time
    time_t t = time(0);
    struct tm* now = localtime(&t);
    // Preparing to open stream
    std::string loggingStreamName;
    loggingStreamName = "/home/dominik/catkin_ws/force" + std::to_string(now->tm_year +1900) + "-" + std::to_string(now->tm_mon + 1) + "-" + std::to_string(now->tm_mday) + "_" + std::to_string(now->tm_hour) + ":" + std::to_string(now->tm_min) + ":" + std::to_string(now->tm_sec) + ".log";

    logFileForce.open(loggingStreamName, std::ios_base::app);
    logFileForce << "# timestamp tx ty tz fx fy fz effector_x effector_y effector_z effector_qx effector_qy effector_qz\n";
    logFileForce.setf(ios::fixed);

    loggingStreamName = "/home/dominik/catkin_ws/robotState" + std::to_string(now->tm_year +1900) + "-" + std::to_string(now->tm_mon + 1) + "-" + std::to_string(now->tm_mday) + "_" + std::to_string(now->tm_hour) + ":" + std::to_string(now->tm_min) + ":" + std::to_string(now->tm_sec) + ".log";
    logFileRobotState.open(loggingStreamName, std::ios_base::app);
    logFileRobotState << "# timestamp tx ty tz\n";
    logFileRobotState.setf(ios::fixed);

    initialize();

    Eigen::Affine3d motion(Eigen::Affine3d::Identity());
    motion(0,3)=0.0;
    motion(1,3)=0.15;
    trajectory.push_back(motion);
    motion(0,3)=0.15;
    motion(1,3)=0.0;
    trajectory.push_back(motion);
    motion(0,3)=0.0;
    motion(1,3)=-0.15;
    trajectory.push_back(motion);
    motion(0,3)=-0.15;
    motion(1,3)=0.0;
    trajectory.push_back(motion);
    trajectoryPointNo=0;
}

ControllingUR::~ControllingUR()
{
    logFileForce.close();
    logFileRobotState.close();
//  fusionServiceQueue_.clear();
//  fusionServiceQueue_.disable();
//  nodeHandle_.shutdown();
//  fusionServiceThread_.join();
}

bool ControllingUR::readParameters()
{
  // ControllingUR parameters.
  nodeHandle_.param("/force_sensor_topic", forceSensorTopic_, string(""));
  nodeHandle_.param("/controller_state", controllerStateTopic_, string(""));
  nodeHandle_.param("/joint_result", controllerResultTopic_, string(""));
  nodeHandle_.param("/trajectory_planned", trajectoryTopic_, string("/trajectory_planned"));
  nodeHandle_.param("/joint_goal", jointGoalTopic_, string(""));
  nodeHandle_.param("/maxJointErrorTollerance", maxJointErrorTolerance, 0.05);
  nodeHandle_.param("/useSim", useSim, 1);
  nodeHandle_.param("/planning_group", planningGroup, string("left_arm"));
  nodeHandle_.param("/arm_prefix", armPrefix, string(""));

//  std::cout << "forceSensorTopic_ " << forceSensorTopic_ <<"\n";
//  std::cout << "controllerStateTopic_ " << controllerStateTopic_ <<"\n";
//  std::cout << "jointGoalTopic_ " << jointGoalTopic_ << "\n";

  double minUpdateRate;
  nodeHandle_.param("/min_update_rate", minUpdateRate, 2.0);
  maxNoUpdateDuration_.fromSec(1.0 / minUpdateRate);
  ROS_ASSERT(!maxNoUpdateDuration_.isZero());

  return true;
}

void ControllingUR::setReferenceValues(std::vector<double>& refValues, double executionTime){
    if (useSim){
        while (getMaxJointError()>maxJointErrorTolerance) {// wait until previous motion is finished
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            controllerStatus=3;
        }
        controllerStatus=0;
    }
    else {
        while (controllerStatus!=3) // wait for previous motion to finish
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        controllerStatus=0;
    }

    referenceJointsVals = refValues;
    ctrlMsg.goal.trajectory.points.resize(2);

    ctrlMsg.goal.trajectory.points[0].positions.resize(6);
    ctrlMsg.goal.trajectory.points[0].velocities.resize(6);
    ctrlMsg.goal.trajectory.points[0].accelerations.resize(6);
    for (size_t jointNo=0; jointNo<6;jointNo++){
        ctrlMsg.goal.trajectory.points[0].velocities[jointNo] = 0.0;
        ctrlMsg.goal.trajectory.points[0].positions[jointNo]=currentJointsState[jointNo];
        ctrlMsg.goal.trajectory.points[0].accelerations[jointNo] = 0.0;
    }
    ctrlMsg.goal.trajectory.points[0].time_from_start = ros::Duration(0.0);

    ctrlMsg.goal.trajectory.points[1].positions.resize(6);
    ctrlMsg.goal.trajectory.points[1].velocities.resize(6);
    ctrlMsg.goal.trajectory.points[1].accelerations.resize(6);
    for (size_t jointNo=0; jointNo<6;jointNo++){
        ctrlMsg.goal.trajectory.points[1].positions[jointNo]=refValues[jointNo];
        ctrlMsg.goal.trajectory.points[1].velocities[jointNo] = 0.0;
        ctrlMsg.goal.trajectory.points[1].accelerations[jointNo] = 0.0;
    }
    ctrlMsg.goal.trajectory.points[1].time_from_start = ros::Duration(executionTime);

    robotControllerPublisher.publish(ctrlMsg);
}

/// get max joint error
double ControllingUR::getMaxJointError(void){
    double maxErr = std::numeric_limits<double>::min();
    for (size_t jointNo=0; jointNo<referenceJointsVals.size();jointNo++){
        double err = fabs(referenceJointsVals[jointNo]-currentJointsState[jointNo]);
            if (err>maxErr)
                maxErr = err;
    }
    return maxErr;
}

bool ControllingUR::initialize(){
    ROS_INFO("Controller UR node initializing ... ");
    // wait for controller state feedback
    ros::Duration(1.0).sleep(); // Need this to get the TF caches fill up.

    ///initialize MoveIt
    kinematic_state.reset(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
//    std::vector<std::string> jointModelNames = kinematic_model->getJointModelGroupNames();
//    for (auto nam : jointModelNames)
//        std::cout << nam << "\n";
    joint_model_group = kinematic_model->getJointModelGroup(planningGroup);

    Eigen::VectorXd initConfiguration(6,1); initConfiguration << initConf[0],initConf[1],initConf[2],initConf[3],initConf[4],initConf[5];
    kinematic_state->setJointGroupPositions(joint_model_group,initConfiguration);

    initializeMoveItOMPL();
    resetUpdateTimer();
    ROS_INFO("Initialization done.");
    return true;
}

/// initialize opml library
void ControllingUR::initializeMoveItOMPL(void){
    planningScene.reset(new planning_scene::PlanningScene(kinematic_model));
    //planning_scene::PlanningScene planning_scene(kinematic_model);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planningScene->checkSelfCollision(collision_request, collision_result);
    if (collision_result.collision)
        ROS_INFO("Test 1: Current state is: collision");
    else
        ROS_INFO("Test 1: Current state is: no collision");

    moveGroup.setPlannerId("RRTConnectkConfigDefault");

//    robot_state::RobotState& current_state = planningScene->getCurrentStateNonConst();
//    current_state.setToRandomPositions();
//    collision_result.clear();
//    planningScene->checkSelfCollision(collision_request, collision_result);
//    if (collision_result.collision)
//        ROS_INFO("Test 2: Current state is: collision");
//    else
//        ROS_INFO("Test 2: Current state is: no collision");

//    collision_request.group_name = PLANNING_GROUP;
//    current_state.setToRandomPositions();
//    collision_result.clear();
//    planningScene->checkSelfCollision(collision_request, collision_result);
//    if (collision_result.collision)
//        ROS_INFO("Test 3: Current state is: collision");
//    else
//        ROS_INFO("Test 3: Current state is: no collision");

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    std::string planner_plugin_name;

    nodeHandle_.setParam("planning_plugin", "ompl_interface/OMPLPlanner");
    if (!nodeHandle_.getParam("planning_plugin", planner_plugin_name))
      ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!planner_instance->initialize(kinematic_model, nodeHandle_.getNamespace()))
        ROS_FATAL_STREAM("Could not initialize planner instance");
      ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (std::size_t i = 0; i < classes.size(); ++i)
        ss << classes[i] << " ";
      ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                           << "Available plugins: " << ss.str());
    }
}

void ControllingUR::moveTool(const Eigen::Affine3d& _motion){
    Eigen::Isometry3d motion;
    motion.matrix() = _motion.matrix();

    ctrlState = STATE_MOVING;
    Eigen::VectorXd initConfiguration(6,1);
    for (size_t jointNo=0;jointNo<6;jointNo++)
        initConfiguration(jointNo,0) = currentJointsState[jointNo];
    kinematic_state->setJointGroupPositions(joint_model_group,initConfiguration);
    std::vector<double> jointValues;
    kinematic_state->copyJointGroupPositions(joint_model_group, jointValues);
//    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
//    for(std::size_t i = 0; i < joint_names.size(); ++i) {
//      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
//    }

//    std::vector<std::string> linkNames = kinematic_model->getLinkModelNames();
//    for (auto nam : linkNames)
//        std::cout << nam << "\n";
    const Eigen::Isometry3d &end_effector_state = kinematic_state->getGlobalLinkTransform(armPrefix+"wrist_3_link");

    /* Print end-effector pose. Remember that this is in the model frame */
//    ROS_INFO_STREAM("Curent Translation: " << end_effector_state.translation());
//    ROS_INFO_STREAM("current Rotation: " << end_effector_state.rotation());

    Eigen::Isometry3d newEndEffectorState = end_effector_state * motion;

//    ROS_INFO_STREAM("New Translation: " << newEndEffectorState.translation());
//    ROS_INFO_STREAM("New Rotation: " << newEndEffectorState.rotation());

    bool found_ik = kinematic_state->setFromIK(joint_model_group, newEndEffectorState, armPrefix+"wrist_3_link", 10, 0.1);
    if (found_ik) {
      kinematic_state->copyJointGroupPositions(joint_model_group, jointValues);
//      for(std::size_t i=0; i < joint_names.size(); ++i)
//      {
//        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), jointValues[i]);
//      }
      setReferenceValues(jointValues,1.1);
    }
    else {
      ROS_INFO("Did not find IK solution");
    }
    ctrlState = STATE_NORMAL;
}

void ControllingUR::moveToolOMPL(const Eigen::Affine3d& _motion){
    Eigen::Isometry3d motion;
    motion.matrix() = _motion.matrix();

    while (ctrlState==STATE_INITIALIZATION){
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        ctrlState = STATE_INITIALIZATION;
    }
    ctrlState = STATE_MOVING;
    std::vector<double> jointValues;
    const Eigen::Isometry3d &end_effector_state = kinematic_state->getGlobalLinkTransform(armPrefix+"wrist_3_link");

    Eigen::Isometry3d newEndEffectorState = end_effector_state * motion;

    std::vector<std::string> jointNames = moveGroup.getJoints();
    bool found_ik = kinematic_state->setFromIK(joint_model_group, newEndEffectorState, armPrefix+"wrist_3_link", 100, 0.1);
    if (found_ik) {
        kinematic_state->copyJointGroupPositions(joint_model_group, jointValues);
        for (unsigned int i = 0; i < jointNames.size(); ++i){
//            ROS_INFO_STREAM("\t" << jointNames[i] << " goal position: " << jointValues[i]);
            moveGroup.setJointValueTarget(jointNames[i], jointValues[i]);
        }
        moveGroup.setStartStateToCurrentState();

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveGroup.setPlanningTime(1.0);

        moveGroup.setMaxVelocityScalingFactor(0.1);

        moveit::planning_interface::MoveItErrorCode success = moveGroup.plan(my_plan);

        if ( !success )
            throw std::runtime_error("No plan found1");
//        ROS_INFO_STREAM("Plan found in " << moveGroup.getPlanningTime() << " seconds");

        // Execute the plan
        ros::Time start = ros::Time::now();
        if (useSim){
//            moveGroup.setGoalJointTolerance(0.1);
        }
        moveGroup.execute(my_plan);
        ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
    }
    else {
        ROS_INFO("Did not find IK solution");
    }
    resetUpdateTimer();
    ctrlState = STATE_NORMAL;
}

/// execute trajectory with waypoints
void ControllingUR::planAndExecuteTrajectory(const std::vector<Eigen::Affine3d>& motions){
    ctrlState = STATE_MOVING;
    while (ctrlState==STATE_INITIALIZATION)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
//    std::vector<double> jointValues;
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(armPrefix+"wrist_3_link");

    std::vector<geometry_msgs::Pose> waypoints;
    Eigen::Affine3d effectorPose = end_effector_state;
    effectorPose.matrix().setIdentity();
//    int pointNo = 0;
    for (const auto& motion : motions){
//        std::cout << "efector before:\n" << effectorPose.matrix() << "\n";
        double x = motion(0,3); double y = motion(1,3); double z = motion(2,3);
        Eigen::Affine3d motionCpy(motion);
        motionCpy.matrix()(0,3) = y; motionCpy.matrix()(1,3) = -x; motionCpy.matrix()(2,3) = z;
//        std::cout << "motion:\n" << motion.matrix() << "\n";
        effectorPose = /*effectorPose **/ motionCpy;
//        std::cout << "efector after:\n" << effectorPose.matrix() << "\n";
        geometry_msgs::Pose newPose;
        newPose.position.x = effectorPose(0,3); newPose.position.y = effectorPose(1,3); newPose.position.z = effectorPose(2,3);
        Eigen::Quaterniond quat(effectorPose.rotation());
        newPose.orientation.w = quat.w(); newPose.orientation.x = quat.x(); newPose.orientation.y = quat.y(); newPose.orientation.z = quat.z();
//        std::cout << "new pose " << newPose.position.x << ", " << newPose.position.y << ", " << newPose.position.z << " - " << newPose.orientation.w << ", " << newPose.orientation.x << ", " << newPose.orientation.y << ", " << newPose.orientation.z << "\n";
//        std::cout << "new efector after:\n" << quat.matrix() << "\n";
        waypoints.push_back(newPose);
//        getchar();
//        pointNo++;
//        if (pointNo>5)
//            break;
    }

    moveGroup.setStartStateToCurrentState();
    moveit_msgs::RobotTrajectory trajectory;

    moveGroup.setPoseReferenceFrame(armPrefix+"ee_link");
    double fraction = moveGroup.computeCartesianPath(waypoints,
                                                 0.005,  // eef_step
                                                 0.0,   // jump_threshold
                                                 trajectory);
    std::cout << "fraction " << fraction << "\n";
    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% achieved)",
          fraction * 100.0);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(15.0);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveGroup.setPlanningTime(1.0);

    moveGroup.setMaxVelocityScalingFactor(0.1);
    ros::Duration time(0.0);

    for (auto& point : trajectory.joint_trajectory.points){
        time+=ros::Duration(0.1);
        point.time_from_start = time;
    }

    my_plan.trajectory_ = trajectory;

    // Execute the plan
    ros::Time start = ros::Time::now();
    if (useSim){
        //            moveGroup.setGoalJointTolerance(0.1);
    }
    moveGroup.execute(my_plan);
    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

//    Eigen::Affine3d newEndEffectorState = end_effector_state * motion;

//    std::vector<std::string> jointNames = moveGroup.getJoints();
//    bool found_ik = kinematic_state->setFromIK(joint_model_group, newEndEffectorState, armPrefix+"wrist_3_link", 100, 0.1);
//    if (found_ik) {
//        kinematic_state->copyJointGroupPositions(joint_model_group, jointValues);
//        for (unsigned int i = 0; i < jointNames.size(); ++i){
////            ROS_INFO_STREAM("\t" << jointNames[i] << " goal position: " << jointValues[i]);
//            moveGroup.setJointValueTarget(jointNames[i], jointValues[i]);
//        }
//        moveGroup.setStartStateToCurrentState();

//        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//        moveGroup.setPlanningTime(1.0);

//        moveGroup.setMaxVelocityScalingFactor(0.1);

//        moveit::planning_interface::MoveItErrorCode success = moveGroup.plan(my_plan);

//        if ( !success )
//            throw std::runtime_error("No plan found");
////        ROS_INFO_STREAM("Plan found in " << moveGroup.getPlanningTime() << " seconds");

//        // Execute the plan
//        ros::Time start = ros::Time::now();
//        if (useSim){
////            moveGroup.setGoalJointTolerance(0.1);
//        }
//        moveGroup.execute(my_plan);
//        ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
//    }
//    else {
//        ROS_INFO("Did not find IK solution");
//    }
    ctrlState = STATE_NORMAL;
    resetUpdateTimer();
}

void ControllingUR::motionEventCallback(const geometry_msgs::WrenchStamped& tipForce) {
    ROS_DEBUG("Controlling UR received a motion order");
}

/// received string order
void ControllingUR::orderArmCallback(const std_msgs::String::ConstPtr& msg){
    std::cout << "Order received\n";
    if (msg->data.compare("moveInit")==0){
        moveToInit();
    }
    else if (msg->data.compare("movePlace")==0){
		moveToPlace({3.51, -1.80, -2.02, 0.21, 0.39, 1.13});
	}
    else if (msg->data.compare("moveCameraFront")==0){
		moveToPlace({2.3, -2.3, -1.7, 0.8, 2.4, 0.75});
	}
    else if (msg->data.compare("moveRight")==0){
		moveToPlace({3.99, -1.21, -1.87, 0.16, 0.81, 0.74});
	}
    else if (msg->data.compare("moveLeft")==0){
		moveToPlace({2.35, -2.97, -1.43, 1.17, 2.33, 0.71});
        //moveToPlace({3.51, -1.80, -2.02, 0.21, 0.39, 1.13});
	}
}

void ControllingUR::updateTimerCallback(const ros::TimerEvent&) {
    stopUpdateTimer();
//     ROS_WARN("ControllingUR is updated with time event.");
    timeOfLastUpdate = ros::Time::now();
    Eigen::Affine3d motion(Eigen::Affine3d::Identity());
    motion(1,3)=0.005;
    double contactMagn = contactForce.norm();
//    std::cout << "contact Force " << contactMagn << "\n";
//    if ((ctrlState==STATE_NORMAL)&&contactMagn<maxContactForce){
//        std::cout << "move down motion no " << trajectoryPointNo << " \n";
//        motion = trajectory[trajectoryPointNo];
//        moveToolOMPL(motion);
//        trajectoryPointNo++;
//        if (trajectoryPointNo>=trajectory.size())
//            trajectoryPointNo=0;
//    }
//    else
    if (ctrlState==STATE_MOVE2INIT){
        std_msgs::String msg;
        msg.data = "STATE_MOVE2INIT";
        armControllerStatePub.publish(msg);
        moveToInit();
    }
    else if ((ctrlState==STATE_NORMAL)&&contactMagn>maxContactForce) {
        ctrlState=STATE_MOVE2INIT;
    }
    else{
        //do nothing;
    }

    if (ctrlState==STATE_NORMAL){
        std_msgs::String msg;
        msg.data = "STATE_NORMAL";
        armControllerStatePub.publish(msg);
    }
    else if (ctrlState==STATE_MOVE2INIT){
        std_msgs::String msg;
        msg.data = "STATE_MOVE2INIT";
        armControllerStatePub.publish(msg);
    }
    else if (ctrlState==STATE_MOVING){
        std_msgs::String msg;
        msg.data = "STATE_MOVING";
        armControllerStatePub.publish(msg);
    }
    else if (ctrlState==STATE_INITIALIZATION){
        std_msgs::String msg;
        msg.data = "STATE_INITIALIZATION";
        armControllerStatePub.publish(msg);
    }
    resetUpdateTimer();
}

void ControllingUR::forceSensorCallback(const geometry_msgs::WrenchStamped& tipForce) {
    
//    ROS_WARN("ControllingUR is updated with force event.");
    //std::cout << "force " << tipForce.wrench.force.x << ", " << tipForce.wrench.force.y << ", " << tipForce.wrench.force.z << "\n";
    contactForceMtx.lock();
    contactForce(0) = tipForce.wrench.force.x; contactForce(1) = tipForce.wrench.force.y; contactForce(2) = tipForce.wrench.force.z;
    logFileForce << ros::Time::now() << " " << contactForce(0) << " " << contactForce(1) << " " << contactForce(2) << "\n";
    contactForceMtx.unlock();
    logFileForce.flush();
}

void ControllingUR::controllerStateCallback(const sensor_msgs::JointState::ConstPtr& jointState){
//    ROS_WARN("ControllingUR is updated with controller state event.");
    Eigen::VectorXd configuration(6,1);
    for (size_t jointNo=0; jointNo<jointState->position.size();jointNo++){
//        ROS_INFO("I heard pos[%d]: %f", (int)jointNo, jointState->position[jointNo]);
        configuration(jointNo) = jointState->position[jointNo];
        currentJointsState[jointNo] = jointState->position[jointNo];
    }
    kinematic_state->setJointGroupPositions(joint_model_group,configuration);
    processStateCallback();
}

void ControllingUR::controllerResultCallback(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& jointResult){
//    ROS_WARN("ControllingUR is updated with controller result event.");
//    std::cout << (int)jointResult->status.status << "\n";
    controllerStatus = jointResult->status.status;
}
void ControllingUR::graspCallback(const geometry_msgs::Pose& pose)
{
    int pointNo = 0;
    Eigen::Affine3d prevPose;
    Eigen::Affine3d motion;
    std::vector<Eigen::Affine3d> motions;
    Eigen::Affine3d currPose;
    currPose.matrix().block<3,1>(0,3) << pose.position.x , pose.position.y , pose.position.z;
    Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x , pose.orientation.y, pose.orientation.z);
    currPose.matrix().block<3,3>(0,0) = quat.matrix();
    motions.push_back(currPose);
    planAndExecuteTrajectory(motions);
    ctrlState = STATE_NORMAL;
}
void ControllingUR::trajectoryCallback(const nav_msgs::PathConstPtr& trajectory){
//    ROS_WARN("ControllingUR is updated with controller result event.");
     ctrlState = STATE_MOVING;
//    std::cout << "goal trajectory:\n";
//    int pointNo = 0;
//    Eigen::Affine3d prevPose;
//    Eigen::Affine3d motion;
//    for (const auto& point : trajectory.get()->poses){
//        Eigen::Affine3d currPose;
//        currPose.matrix().block<3,1>(0,3) << point.pose.position.x , point.pose.position.y , point.pose.position.z;
//        Eigen::Quaterniond quat(point.pose.orientation.w, point.pose.orientation.x , point.pose.orientation.y, point.pose.orientation.z);
//        currPose.matrix().block<3,3>(0,0) = quat.matrix();
////        std::cout << "currPose\n" << currPose.matrix() << "\n";
//        if (pointNo>0){
//            motion = prevPose.inverse()*currPose;
////            std::cout << "motion\n" << motion.matrix() << "\n";
//            moveToolOMPL(motion);
//        }
//        prevPose = currPose;
//        pointNo++;
//    }
    int pointNo = 0;
    Eigen::Affine3d prevPose;
    Eigen::Affine3d motion;
    std::vector<Eigen::Affine3d> motions;
    for (const auto& point : trajectory.get()->poses){
        Eigen::Affine3d currPose;
        currPose.matrix().block<3,1>(0,3) << point.pose.position.x , point.pose.position.y , point.pose.position.z;
        Eigen::Quaterniond quat(point.pose.orientation.w, point.pose.orientation.x , point.pose.orientation.y, point.pose.orientation.z);
        currPose.matrix().block<3,3>(0,0) = quat.matrix();
        if (pointNo>0){
            motion = prevPose.inverse()*currPose;
            motions.push_back(motion);
        }
        prevPose = currPose;
        pointNo++;
    }
    planAndExecuteTrajectory(motions);
    ctrlState = STATE_NORMAL;
}

void ControllingUR::saveRobotState(void){
    logFileRobotState << ros::Time::now() << " ";
    for (size_t jointNo=0; jointNo<6;jointNo++){
        logFileRobotState << currentJointsState[jointNo] << " ";
    }
    contactForceMtx.lock();
    logFileRobotState << " " << contactForce(0) << " " << contactForce(1) << " " << contactForce(2) << " ";
    contactForceMtx.unlock();
    const Eigen::Affine3d &effectorState = kinematic_state->getGlobalLinkTransform(armPrefix+"wrist_3_link");
    Eigen::Quaterniond quat(effectorState.rotation());
    logFileRobotState << " " << effectorState(0,3) << " " << effectorState(1,3) << " " << effectorState(2,3) << " " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << "\n";
    logFileRobotState.flush();
}

void ControllingUR::processStateCallback(void){
    saveRobotState();

    double contactMagn = contactForce.norm();
    if (ctrlState==STATE_INITIALIZATION){
        ctrlState=STATE_MOVE2INIT;
    }
    if (contactMagn>maxContactForce){
        //ctrlState = STATE_MOVE2INIT;
        if (ctrlState==STATE_NORMAL){
            moveGroup.stop();
            ROS_INFO("stopEffector\n");
        }
    }
}

void ControllingUR::controllerStateCallbackSim(const control_msgs::JointTrajectoryControllerState::ConstPtr& jointState) {
//     ROS_WARN("ControllingUR is updated with controller state event.");
    Eigen::VectorXd configuration(6,1);
    for (size_t jointNo=0; jointNo<6;jointNo++){
//        ROS_INFO("I heard pos[%d]: %f", (int)jointNo, jointState->actual.positions[jointNo]);
        configuration(jointNo) = jointState->actual.positions[jointNo];
        currentJointsState[jointNo] = jointState->actual.positions[jointNo];
    }
    kinematic_state->setJointGroupPositions(joint_model_group,configuration);
    processStateCallback();
}

bool ControllingUR::resetRobot(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    ROS_INFO("Reset robot.");
    return true;
}

bool ControllingUR::moveToInit(void){
    ctrlState = STATE_MOVE2INIT;
    std::cout << "first run\n";
    ROS_INFO_STREAM("Set init position\n");
            std::vector<std::string> jointNames = moveGroup.getJoints();
            //setReferenceValues(initConf, 5.1);
            //std::this_thread::sleep_for(std::chrono::seconds(10));

            for (unsigned int i = 0; i < jointNames.size(); ++i){
    //            ROS_INFO_STREAM("\t" << jointNames[i] << " goal position: " << jointValues[i]);
                moveGroup.setJointValueTarget(jointNames[i], initConf[i]);
            }
            moveGroup.setStartStateToCurrentState();

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            moveGroup.setPlanningTime(1.0);

            moveGroup.setMaxVelocityScalingFactor(0.05);

            moveit::planning_interface::MoveItErrorCode success = moveGroup.plan(my_plan);

    if ( !success )
        throw std::runtime_error("No plan found2");
    //        ROS_INFO_STREAM("Plan found in " << moveGroup.getPlanningTime() << " seconds");

    // Execute the plan
    ros::Time start = ros::Time::now();
    moveGroup.execute(my_plan);
    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
    ctrlState=STATE_NORMAL;
    return true;
}

bool ControllingUR::moveToPlace(std::vector<double> placeConf){
    ctrlState = STATE_MOVING;
    std::cout << "Moving to given place\n";
    ROS_INFO_STREAM("Set given position\n");
            std::vector<std::string> jointNames = moveGroup.getJoints();
            //setReferenceValues(initConf, 5.1);
            //std::this_thread::sleep_for(std::chrono::seconds(10));

            for (unsigned int i = 0; i < jointNames.size(); ++i){
    //            ROS_INFO_STREAM("\t" << jointNames[i] << " goal position: " << jointValues[i]);
                moveGroup.setJointValueTarget(jointNames[i], placeConf[i]);
            }
            moveGroup.setStartStateToCurrentState();

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            moveGroup.setPlanningTime(1.0);

            moveGroup.setMaxVelocityScalingFactor(0.05);

            moveit::planning_interface::MoveItErrorCode success = moveGroup.plan(my_plan);

    if ( !success )
        throw std::runtime_error("No plan found2");
    //        ROS_INFO_STREAM("Plan found in " << moveGroup.getPlanningTime() << " seconds");

    // Execute the plan
    ros::Time start = ros::Time::now();
    moveGroup.execute(my_plan);
    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
    ctrlState=STATE_NORMAL;
    return true;
}

void ControllingUR::resetUpdateTimer() {
//  ROS_INFO("Reset update timer.");
    updateTimer_.stop();
    Duration periodSinceLastUpdate = ros::Time::now() - timeOfLastUpdate;
    if (periodSinceLastUpdate > maxNoUpdateDuration_) periodSinceLastUpdate.fromSec(0.0);
    updateTimer_.setPeriod(maxNoUpdateDuration_ - periodSinceLastUpdate);
    updateTimer_.start();
//  ROS_INFO("Done reset update timer.");
}

void ControllingUR::stopUpdateTimer(){
    updateTimer_.stop();
}

} /* namespace */

