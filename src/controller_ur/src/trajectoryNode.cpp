// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the PUT hand Driver suite.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE folder in the top
// directory of the source code.
//
// ÂŠ Copyright 2018 Poznan University of Technology, Poznan, Poland
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Dominik Belter
 * \date    2018-05-19
 *
 */
//----------------------------------------------------------------------
// ROS includes.
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>

#include <string>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include<sstream>
#include<vector>
#include <std_msgs/String.h>
#include"std_msgs/Float64.h"
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <thread>
#include <controller_ur/RG2.h>
#include <controller_ur/RG2_Grip.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include "../include/controllerUR/controllingUR.hpp"
/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
using namespace controller_ur;
using namespace std;
int counter_strike=0;
int change = 0;//stores state info: 0-no change, 1-state after trajectory, 2-started moving to initial position, 3-moving to initial position
controller_ur::RG2 tmp;
	  std_msgs::Float64 aaa;
std_msgs::String command;
string ctrlState;

void stateCallback(const std_msgs::String::ConstPtr &msg)
{
    ctrlState=msg->data;
    cout<<" ctrlState " << ctrlState <<endl;
}

int main(int argc, char** argv)
{
    //==========
    // ROS
    //==========

    // Set up ROS.
    ros::init(argc, argv, "trajectory_publisher");
    ros::init(argc, argv, "current_state");
    // Private NH for general params
    ros::NodeHandle n;

    // Tell ROS how fast to run this node. (100 = 100 Hz = 10 ms)
    ros::Rate loop_rate(10);
    //==========
    // Callbacks
    //==========

    ros::Subscriber sub = n.subscribe("/controller_ur/currentState", 1000, stateCallback);

    // Publish current channel positions
    ros::Publisher trajectory_pub = n.advertise<nav_msgs::Path>("/trajectory_planned", 1000);

    //command publisher
    ros::Publisher commandPub = n.advertise<std_msgs::String>("/controller_ur/order", 1000);

    //==========
    // Messaging
    //==========
    nav_msgs::Path path;

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.pose.position.x = 0.0; poseStamped.pose.position.y = 0.0; poseStamped.pose.position.z = 0.0;
    poseStamped.pose.orientation.w = 1.0; poseStamped.pose.orientation.x = 0.0; poseStamped.pose.orientation.y = 0.0; poseStamped.pose.orientation.z = 0.0;
    path.poses.push_back(poseStamped);
    poseStamped.pose.position.x = -0.05;
    poseStamped.pose.position.y = 0.025;
    path.poses.push_back(poseStamped);
    poseStamped.pose.position.x = -0.1;
    poseStamped.pose.position.y = 0.05;
    path.poses.push_back(poseStamped);
    for (int i=0;i<50;i++){
        poseStamped.pose.position.x = -0.1+(double(i)*0.2)/50.0;
        poseStamped.pose.position.y = 0.05+0.1*sin((double(i)/50.0)*6.28);
        path.poses.push_back(poseStamped);
    }
    poseStamped.pose.position.x = -0.0;
    poseStamped.pose.position.y = 0.05;
    path.poses.push_back(poseStamped);

    bool isfirst(true);
    // Main loop.
    while (ros::ok())
    {
        if (change==0 && ctrlState == "STATE_NORMAL"){
            aaa.data=0;
          tmp.request.target_width= aaa;
          ros::service::call("/rg2_gripper/control_width", tmp);
            usleep(10000000);
            cout<<"move init "<<change<<endl;
            std_msgs::String msg;
            std::stringstream ss;
            ss << "moveInit";
            msg.data = ss.str();
            commandPub.publish(msg);
            change = 1;
            usleep(1000000);
        }
        else if (change==1&& ctrlState.compare("STATE_NORMAL")==0){
            aaa.data=160;
          tmp.request.target_width= aaa;
          ros::service::call("/rg2_gripper/control_width", tmp);
            usleep(10000000);
            cout<<"executing trajectory "<<change<<endl;
            trajectory_pub.publish(path);
            isfirst=false;
            cout<<"trajectory executed "<<change<<endl;
            change = 0;
            usleep(1000000);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    cout<<"exited loop "<<change<<endl;
    return 0;
}


