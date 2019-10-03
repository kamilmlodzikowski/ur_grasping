// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the PUT hand Driver suite.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE folder in the top
// directory of the source code.
//
// ĂĹ  Copyright 2018 Poznan University of Technology, Poznan, Poland
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
#include "geometry_msgs/Pose.h"
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
int change = 0;//stores state info: 0-wait for msg, 1-camera front, 2-open gripper, 3-go to object position, 4-grab object, 5-move to Init, 6-open gripper
controller_ur::RG2 tmp;
std_msgs::Float64 aaa;
std_msgs::String command;
string ctrlState;


double PointX, PointY, PointZ;
std::vector<double> cameraConf = {2.3, -2.3, -1.7, 0.8, 2.4, 0.75};

string enableCommand;

void stateCallback(const std_msgs::String::ConstPtr &msg)
{
    ctrlState=msg->data;
    cout<<" ctrlState " << ctrlState <<endl;
}


void graspEnableCallback(const std_msgs::String::ConstPtr &msg)
{

    enableCommand=msg->data;

}


void objectPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
   vector<vector<double>> trasnform_coordinates;//translation from the front of the robot - 10cm and 7cm from up to down
    
    Eigen::Affine3d transCoord;
    Eigen::Affine3d coord;
    Eigen::Affine3d newCoord;
    transCoord.matrix()(0,0) = -1.0;
    transCoord.matrix()(0,1) = 0;
    transCoord.matrix()(0,2) = 0;
    transCoord.matrix()(0,3) = 0;
    transCoord.matrix()(1,0) = 0;
    transCoord.matrix()(1,1) = 0;
    transCoord.matrix()(1,2) = 1.0;
    transCoord.matrix()(1,3) = -0.7;
    transCoord.matrix()(2,0) = 0;
    transCoord.matrix()(2,1) = 1.0;
    transCoord.matrix()(2,2) = 0;
    transCoord.matrix()(2,3) = 0.1;
    transCoord.matrix()(3,0) = 0;
    transCoord.matrix()(3,1) = 0;
    transCoord.matrix()(3,2) = 0;
    transCoord.matrix()(3,3) = 1.0;

  coord.matrix()(0,0)=msg->position.x;
  coord.matrix()(1,0)=msg->position.y;
  if (msg->position.z<0.4)
  {
  coord.matrix()(2,0)=msg->position.z;
  }
  else 
  {
      coord.matrix()(2,0) = 0.4;
      cout<<"wrong z value - error"<<endl;

  }
  coord.matrix()(3,0)=1;
   newCoord = transCoord*coord;

    PointX = newCoord.matrix()(0,0);
    PointY = newCoord.matrix()(1,0);
    PointZ = 0;//newCoord.matrix()(2,0);



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

    ros::Subscriber subGraspEnable = n.subscribe("/controller_ur/enable_grasp", 1000, graspEnableCallback);
    ros::Subscriber subObjectPose = n.subscribe("/controller_ur/objectPose", 1000, objectPoseCallback);



    // Publish current channel positions
    ros::Publisher trajectory_pub = n.advertise<nav_msgs::Path>("/trajectory_planned", 1000);

    //command publisher
    ros::Publisher commandPub = n.advertise<std_msgs::String>("/controller_ur/order", 1000);

    // enable signal publisher
    ros::Publisher enablePub = n.advertise<std_msgs::String>("/controller_ur/enable_grasp", 1000);
    //==========
    // Messaging
    //==========
    nav_msgs::Path path;

    
    

    bool isfirst(true);
    // Main loop.
    while (ros::ok())
    {   

cout<<PointX<<"  " <<PointY<<"   "<<PointZ<<endl;
       if(change==0 && ctrlState == "STATE_NORMAL")
        {  
           if (enableCommand.compare("enable")==0)
            {
                  change=1;
                  enableCommand="disable";
                  std_msgs::String msgEnable;
                  std::stringstream enable;
                  enable << "disable";
                  msgEnable.data = enable.str();    
                  enablePub.publish(msgEnable);        
            }
        }

       else if(change==1 && ctrlState == "STATE_NORMAL")
        {  
            cout<<"moveCameraFront"<<change<<endl;
            std_msgs::String msg;
            std::stringstream ss;
            ss << "moveCameraFront";
            msg.data = ss.str();
            commandPub.publish(msg);
            usleep(1000000);
            change = 2;
            cout<<change;
        }


  
        else if (change==2&& ctrlState.compare("STATE_NORMAL")==0){
            usleep(4000000);
            aaa.data=160;
            cout<<"open after cameraFront";
            tmp.request.target_width= aaa;
            ros::service::call("/rg2_gripper/control_width", tmp);
            usleep(100000);
            change = 3;
            cout<<change;
        }

        else if(change==3 && ctrlState == "STATE_NORMAL")
        {  
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.pose.position.x = 0.0; poseStamped.pose.position.y = 0.0; poseStamped.pose.position.z = 0.0;
    poseStamped.pose.orientation.w = 1.0; poseStamped.pose.orientation.x = 0.0; poseStamped.pose.orientation.y = 0.0; poseStamped.pose.orientation.z = 0.0;
    path.poses.push_back(poseStamped);
    cout<<PointX<<endl;
for (int i=1;i<51;i++)
{
    poseStamped.pose.position.x = i*PointX/50;
    poseStamped.pose.position.y = i*PointY/50;
    poseStamped.pose.position.z = i*PointZ/50;
    path.poses.push_back(poseStamped);
}


            cout<<"executing trajectory "<<change<<endl;
            trajectory_pub.publish(path);
            isfirst=false;
            cout<<"trajectory execution "<<change<<endl;
            usleep(1000000);
            change = 4;
            cout<<change;
        }

        else if(change==4 && ctrlState == "STATE_NORMAL")
        {  
            usleep(5000000);
            cout<<"executed trajectory "<<change<<endl;
            aaa.data=55;
            cout<<"close after bottle";
            tmp.request.target_width= aaa;
            ros::service::call("/rg2_gripper/control_width", tmp);
            usleep(3000000);
            change = 5;
            cout<<change;
        }

            else if(change==5 && ctrlState == "STATE_NORMAL")
        {  
            cout<<"moveInit"<<change<<endl;
            std_msgs::String msg;
            std::stringstream ss;
            ss << "moveInit";
            msg.data = ss.str();
            commandPub.publish(msg);
            usleep(1000000);
            change = 6;
            cout<<change;
        }

        else if(change==6 && ctrlState == "STATE_NORMAL")
        {  
                      
            usleep(4000000);
            aaa.data=160;  
            cout<<"open after init";
            tmp.request.target_width= aaa;
            ros::service::call("/rg2_gripper/control_width", tmp);
            usleep(2000000);
            change = 0;
            cout<<change;
        }



        ros::spinOnce();
        loop_rate.sleep();
    }   

    cout<<"exited loop "<<change<<endl;
    return 0;
}

