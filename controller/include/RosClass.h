#ifndef ROS_CLASS_H
#define ROS_CLASS_H

#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <move_base_msgs/MoveBaseActionGoal.h>

#include <mutex>

class RosClass
{
public:
    /* Constructor for the Ros class */
    RosClass();

    /* Destructor for the Ros class */
    ~RosClass();

    /**
     * @brief  Provides path to the interface class
     * @returns Path from local planner converted wrt globalFrame
     */
    geometry_msgs::PoseArray getPath();

    /**
     * @brief  Provides goal pose to the interface class
     * @returns Goal pose  wrt globalFrame
     */
    geometry_msgs::Pose getGoalPose();

    /**
     * @brief  Provides current robot pose to the interface class
     * @returns Current robot pose wrt globalFrame
     */
    geometry_msgs::Pose getRobotPose();

    /**
     * @brief  Provides odom data to the interface class
     * @returns Odometry data
     */
    geometry_msgs::Twist getOdom();

    /**
     * @brief  Current goal status interface class
     * @returns boolean value indicating the current goal status
     */
    bool getGoalStatus();


    /**
     * @brief  Receives the velocity command from interface class for publishing
     * @param m_cmd The message returned from a message notifier
     */
    void sendVelocityCmd(geometry_msgs::Twist m_cmd);
    
private:
    /**
    * @brief NodeHandle ROS
    */
    ros::NodeHandle nh;

    /**
     * @brief  A callback to obtain the poses provided by the local planner
     * @param path_msg The message returned from a message notifier
     */

    void localPlanCallback(const nav_msgs::PathConstPtr &path_msg);

    /**
     * @brief  A callback to obtain the Odometry message
     * @param odom_msg The message returned from a message notifier
     */

    void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);

    /**
     * @brief  A callback to obtain the robot's current position
     * @param robot_msg The message returned from a message notifier
     */

    void robotPoseCallback(const geometry_msgs::Pose &robot_msg);

    /**
     * @brief  A callback to obtain the final goal to be achieved by the robot
     * @param goal_msg The message returned from a message notifier
     */

    void goalPoseCallback(const move_base_msgs::MoveBaseActionGoal &goal_msg);
    
    /**
     * @brief  A callback to obtain the current goal status from move base
     * @param msg The message returned from a message notifier
     */
    void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);




    /**
     * @brief ROS Subscribers
     */
    ros::Subscriber odomSub;
    ros::Subscriber localPlan;
    ros::Subscriber robotPose;
    ros::Subscriber goalSub;
    ros::Subscriber statusSub;

    /**
     * @brief ROS Publishers
     */

    ros::Publisher cmdVelPub;

    std::string globalFrame = "/map";

    geometry_msgs::Pose m_robotPose, m_goalPose;
    geometry_msgs::PoseArray m_poseArray;
    geometry_msgs::Twist m_odomData;


    int m_pathSize = 0;
    bool m_mbGoalReached;
    // bool m_diagnosticsState = true;
    // double m_odomVel__m_s;
    // double m_odomOmega__rad_s;
};

#endif