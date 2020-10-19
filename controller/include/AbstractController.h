#ifndef ABSTRACT_CONTROLLER_H
#define ABSTRACT_CONTROLLER_H

// #include <iostream>
// #include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <vector>
#include <mutex>

#include <dynamic_reconfigure/server.h>

namespace Navigation
{
    class AbstractController 
    {
        
    public:
        /**
        * is called by client module in order to initialize the required paramters for the controllers
        */
        virtual void initializeController(geometry_msgs::PoseArray &array_msg,geometry_msgs::Pose &robot_pose, geometry_msgs::Pose &last_goal, geometry_msgs::Twist &odom_msg,bool newPlanReceived) =0;


        // /**
        // * is called by client module in order to obtain the command velocity
        // */
        // virtual geometry_msgs::Twist  executeController() = 0;
       
        // /**
        // * return true the goal is reached
        // */
        // virtual bool isGoalReached() = 0;
       
        // /**
        // * reset all states & parameters
        // */
        // virtual void resetControllerParameters() = 0;

        // /**
        // * Debug function to test the controller 
        // * ToDo: remove after debug
        // */
        // virtual controller_msgs::Debug getDebugData() = 0; 

    };
}

#endif
