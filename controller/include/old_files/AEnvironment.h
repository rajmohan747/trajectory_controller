#ifndef AEnvironment_H
#define AEnvironment_H

#include <ros/ros.h>
#include <iostream>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <controller_msgs/Debug.h>
#include <nav_msgs/Path.h>
#include <string>

// Look for a suitable name
namespace Environment 
{
    class AEnvironment 
    {
    public:

     /* Accessors */
    
    virtual geometry_msgs::Pose getRobotPose() = 0;

    virtual geometry_msgs::Pose getGoalPose() = 0;

    virtual geometry_msgs::Twist getOdom() = 0;
    
    virtual bool getGoalStatus() = 0;
    
    virtual bool getDiagnosticsStatus() = 0;

    virtual geometry_msgs::PoseArray getPath() = 0;

    virtual int getParams(std::string) = 0;

    virtual geometry_msgs::PoseArray getGlobalPath() =0;
    /*Set Data*/

    virtual void sendVelocityCmd(geometry_msgs::Twist) = 0;

    virtual void sendStatus(std_msgs::Bool) = 0;
    
    // ToDo: remove after debug
    virtual void sendDebugData(controller_msgs::Debug) = 0;

    // Visualize the smoothed plan in rviz
    virtual void sendSmoothedGlobalPlan(nav_msgs::Path) = 0;

    };
};


#endif
