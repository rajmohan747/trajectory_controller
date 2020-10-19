#include <ros/console.h>
#include "RosClass.h"

/**
* @brief  Constructor for the RosClass
*/
RosClass::RosClass()
{
    /*Subscribers*/
    localPlan        = nh.subscribe("/move_base/TebLocalPlannerROS/local_plan", 1, &RosClass::localPlanCallback, this);
    odomSub          = nh.subscribe("/odom", 1, &RosClass::odomCallback, this);
    robotPose        = nh.subscribe("/robot_pose", 1, &RosClass::robotPoseCallback, this);
    goalSub          = nh.subscribe("/move_base/goal", 1, &RosClass::goalPoseCallback, this);
    statusSub        = nh.subscribe("/move_base/status",1,&RosClass::statusCallback, this);

    // /*Publishers*/
    cmdVelPub        = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ROS_INFO("RosClass:: Constructor initialized");
}

/**
* @brief  Destructor for the RosClass
*/

RosClass::~RosClass()
{
}

// int RosClass::getParams(std::string param)
// {
//     nh.getParam(param, m_param);
//     return m_param;
// }


 /**
 * @brief Callback for local planner.This function converts
 * the available path to a generalized globalFrame ("map")
 */
void RosClass::localPlanCallback(const nav_msgs::PathConstPtr &path_msg)
{
    geometry_msgs::PoseStamped pathBase, pathWorld;
    tf::TransformListener poseTransform;
    std::mutex local_mtx;
    std::unique_lock<std::mutex> local_lock(local_mtx);

    if (path_msg->poses.size() > 0)
    {
        std::string pathFrame = path_msg->poses[0].header.frame_id;
        m_poseArray.poses.clear();
        m_poseArray.poses.resize(0);

        /*If the incoming local planner is already in globalFrame,just updating it to m_poseArray.
          Otherwise converting the data to globalFrame and updating in m_poseArray*/
        if (globalFrame == pathFrame)
        {
            ROS_INFO("globalFrame and pathFrame are equal");
            for (unsigned i = 0; i < path_msg->poses.size(); i++)
            {
                pathBase.pose.position    = path_msg->poses[i].pose.position;
                pathBase.pose.orientation = path_msg->poses[i].pose.orientation;
                m_poseArray.poses.push_back(pathBase.pose);
            }
        }
        else
        {
            poseTransform.waitForTransform(globalFrame, pathFrame, ros::Time(0), ros::Duration(10.0));
            for (unsigned i = 0; i < path_msg->poses.size(); i++)
            {
                /**
                 * @brief Input path frame  transformation into Map frame
                */
                pathBase.header.frame_id  = pathFrame;
                pathBase.header.stamp     = ros::Time(0);
                pathBase.pose.position    = path_msg->poses[i].pose.position;
                pathBase.pose.orientation = path_msg->poses[i].pose.orientation;
                try
                {
                    poseTransform.transformPose(globalFrame, pathBase, pathWorld);
                }
                catch (tf::TransformException &ex)
                {
                    ROS_ERROR("RosClass:: Received an exception CONTROLLER trying to transform a point : %s", ex.what());
                }
                /**
                 * @brief Storing the transformed set of path points
                */
                m_poseArray.poses.push_back(pathWorld.pose);
            }
        }

        m_pathSize = m_poseArray.poses.size();
    }

    ROS_ERROR_COND(path_msg->poses.size() < 0, "Path size insufficient for computation to process");
}

void RosClass::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    m_odomData = odom_msg->twist.twist;
}

void RosClass::robotPoseCallback(const geometry_msgs::Pose &robot_msg)
{
    m_robotPose  = robot_msg;
}

void RosClass::goalPoseCallback(const move_base_msgs::MoveBaseActionGoal &goal_msg)
{
    ROS_WARN("RosClass:: Received New goal");
    m_goalPose     = goal_msg.goal.target_pose.pose;
}


void RosClass::statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
  int i = msg->status_list.size();
  if(i > 0)
  {
     int n_status = msg->status_list[i-1].status;
     if(n_status == 3)
     {
        m_mbGoalReached = true;
     }
  }
}
/* Getter Functions */
geometry_msgs::PoseArray RosClass::getPath()
{
    return m_poseArray;
}


geometry_msgs::Pose RosClass::getGoalPose()
{
    return m_goalPose;
}

geometry_msgs::Pose RosClass::getRobotPose()
{
    return m_robotPose;
}



geometry_msgs::Twist RosClass::getOdom()
{
    return m_odomData;
}

bool RosClass::getGoalStatus()
{
    return m_mbGoalReached;
}



/* Setter Functions */

// void RosClass::sendVelocityCmd(geometry_msgs::Twist m_cmd)
// {
//     cmdVelPub.publish(m_cmd);
// }

// void RosClass::sendStatus(std_msgs::Bool m_status)
// {
//     amrStatusPub.publish(m_status);
// }

// void RosClass::sendDebugData(controller_msgs::Debug debug)
// {
//     debugPub.publish(debug);
// }

// void RosClass::sendSmoothedGlobalPlan(nav_msgs::Path path)
// {
//     smoothedGlobalPlan.publish(path);
// }
