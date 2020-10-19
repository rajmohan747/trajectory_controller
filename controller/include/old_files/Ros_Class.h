#ifndef ROS_CLASS_H
#define ROS_CLASS_H

#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>

#include <ctime>
#include <ratio>
#include <chrono>
#include <algorithm>

#include <cmath>
#include <math.h>
#include <bits/stdc++.h> 

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>
#include <pthread.h>
#include <mutex>
#include <exception>
//#include "robot_activity/robot_activity.h"

#include "AEnvironment.h"

using namespace Environment;

class RosClass : public AEnvironment
{
public:

    /* Constructor for the Ros class */ 
    RosClass();

     /* Destructor for the Ros class */
    ~RosClass();

    /* Accessors */
    geometry_msgs::Twist getOdom() override;

    geometry_msgs::Pose getRobotPose() override;

    bool getGoalStatus() override;

    bool getDiagnosticsStatus() override;
    
    geometry_msgs::Pose getGoalPose() override;

    geometry_msgs::PoseArray getPath() override;

    int getParams(std::string) override;

    geometry_msgs::PoseArray getGlobalPath() override;

    /*SET DATA*/
    void sendVelocityCmd(geometry_msgs::Twist) override;

    void sendStatus(std_msgs::Bool) override;

    void sendDebugData(controller_msgs::Debug) override;

    void sendSmoothedGlobalPlan(nav_msgs::Path) override;

    
private:
     /**
    * @brief NodeHandle ROS
    */
    ros::NodeHandle nh;

    /**
     * @brief  A callback to obtain the poses provided by the local planner
     * @param path_msg The message returned from a message notifier
     */

    void planCallback(const nav_msgs::PathConstPtr& path_msg);
    
    /**
     * @brief  A callback to obtain the poses provided by the global planner
     * @param path_msg The message returned from a message notifier
     */
    void globalPlanCallback(const nav_msgs::PathConstPtr& globalPath_msg);
    /**
     * @brief  A callback to obtain the Odometry message
     * @param odom_msg The message returned from a message notifier
     */

    void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);

    /**
     * @brief  A callback to obtain the robot's current position
     * @param robot_msg The message returned from a message notifier
     */

    void robotPoseCallback(const geometry_msgs::Pose& robot_msg);
    
    /**
     * @brief  A callback to obtain the final goal to be achieved by the robot
     * @param goal_msg The message returned from a message notifier
     */

    void goalPoseCallback(const move_base_msgs::MoveBaseActionGoal &goal_msg);
    
    /**
     * @brief  A callback to obtain the current robot status from move_base server
     * @param msg The message returned from a message notifier
     */

    void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);


    //void heartbeatDiagnosticsCallback(const robot_activity_msgs::StateConstPtr &state);
    
    /**
     * @brief ROS Subscribers
     */
    ros::Subscriber odomSub;
    ros::Subscriber localPlan;
    ros::Subscriber globalPlan;
    ros::Subscriber robotPose;
    ros::Subscriber goalSub;
    ros::Subscriber statusSub;
    ros::Subscriber heartBeatSub;
    
    /**
     * @brief ROS Publishers
     */

    ros::Publisher cmdVelPub;
    ros::Publisher amrStatusPub;
    ros::Publisher debugPub;
    ros::Publisher smoothedGlobalPlan;

    geometry_msgs::Pose m_robotPose,m_goalPose;

    geometry_msgs::PoseArray m_poseArray;  
    geometry_msgs::PoseArray m_globalArray;

    geometry_msgs::Twist m_odomMsg;

    int m_param;
    int m_pathSize = 0;
    int m_globalSize = 0;
    

    bool m_mbGoalReached;
    bool m_diagnosticsState = true;
    // double m_odomVel__m_s;
    // double m_odomOmega__rad_s; 
};

#endif