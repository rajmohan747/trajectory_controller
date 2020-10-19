#include <ros/console.h> 
#include "Ros_Class.h"

/**
* @brief  Constructor for the Computations
*/
RosClass::RosClass()
{
    /*Subscribers*/
    localPlan        = nh.subscribe("/move_base/TebLocalPlannerROS/local_plan", 1, &RosClass::planCallback, this);
    odomSub          = nh.subscribe("/odom", 1, &RosClass::odomCallback, this);
    robotPose        = nh.subscribe("/robot_pose", 1, &RosClass::robotPoseCallback,this);
    goalSub          = nh.subscribe("/move_base/goal",1,&RosClass::goalPoseCallback,this);
    statusSub        = nh.subscribe("/move_base/status",1,&RosClass::statusCallback, this); 
    //heartBeatSub     = nh.subscribe("/heartbeat",1,&RosClass::heartbeatDiagnosticsCallback, this); 
	globalPlan       = nh.subscribe("/move_base/GlobalPlanner/plan", 1, &RosClass::globalPlanCallback, this);

    // /*Publishers*/
    cmdVelPub        = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    amrStatusPub     = nh.advertise<std_msgs::Bool>("/controller_status", 1);  
    debugPub         = nh.advertise<controller_msgs::Debug>("/debug_msgs", 1);  
    smoothedGlobalPlan = nh.advertise<nav_msgs::Path>("/smoothGlobalPlan",1);
}

/**
* @brief  Destructor for the Computations
*/

RosClass::~RosClass()
{

}


int RosClass::getParams(std::string param)
{
    nh.getParam(param, m_param); 
    return m_param;
}

void RosClass::planCallback(const nav_msgs::PathConstPtr& path_msg)
{
    geometry_msgs::PoseStamped path_base_,path_world_;
    tf::TransformListener pose_transform_;
    boost::mutex path_mutex;
    
    if(path_msg->poses.size() > 0) 
    {
        //ROS_INFO("RosClass:: plan size is : %d",path_msg->poses.size());
        std::string path_frame = path_msg->poses[0].header.frame_id;
        pose_transform_.waitForTransform("/map", path_frame, ros::Time(0), ros::Duration(10.0));
        boost::mutex::scoped_lock pathlock(path_mutex); 
        m_poseArray.poses.clear();
        m_poseArray.poses.resize(0);
        for(unsigned i = 0; i < path_msg->poses.size(); i++) 
        {
            
        /**
         * @brief Input path frame  transformation into Map frame
        */
            path_base_.header.frame_id  = path_frame;
            path_base_.header.stamp     = ros::Time(0);
            path_base_.pose.position    = path_msg->poses[i].pose.position;
            path_base_.pose.orientation = path_msg->poses[i].pose.orientation;
            
    
            try
            {
               pose_transform_.transformPose("/map", path_base_, path_world_); 
            }
            catch(tf::TransformException& ex)
            {  
                ROS_ERROR("RosClass:: Received an exception CONTROLLER trying to transform a point : %s", ex.what());
            }
            /**
             * @brief Storing the transformed set of path points
            */
            m_poseArray.poses.push_back(path_world_.pose);   
        }

        m_pathSize = m_poseArray.poses.size();   
        pathlock.unlock();
    }

    ROS_ERROR_COND(path_msg->poses.size() <  0, "Path size insufficient for computation to process");
}

void RosClass::globalPlanCallback(const nav_msgs::PathConstPtr& globalPath_msg)
{
    geometry_msgs::PoseStamped path_base_;
    tf::TransformListener pose_transform_;
    boost::mutex globalPath_mutex;
    
    if(globalPath_msg->poses.size() > 0) 
    {
        //ROS_INFO("RosClass:: plan size is : %d",path_msg->poses.size());
        std::string path_frame = globalPath_msg->poses[0].header.frame_id;
        boost::mutex::scoped_lock globalPathlock(globalPath_mutex); 
        m_globalArray.poses.clear();
        m_globalArray.poses.resize(0);

        for(unsigned i = 0; i < globalPath_msg->poses.size(); i++) 
        { 
            path_base_.header.frame_id  = path_frame;
            path_base_.header.stamp     = ros::Time(0);
            path_base_.pose.position    = globalPath_msg->poses[i].pose.position;
            path_base_.pose.orientation = globalPath_msg->poses[i].pose.orientation;
            m_globalArray.poses.push_back(path_base_.pose);   
        }
        m_globalSize = m_globalArray.poses.size();   
        globalPathlock.unlock();
    }
}


void RosClass::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)  
{
   m_odomMsg.linear.x = odom_msg->twist.twist.linear.x;
   m_odomMsg.angular.z = odom_msg->twist.twist.angular.z;
   //m_odomVel__m_s     = odom_msg->twist.twist.linear.x;
   //m_odomOmega__rad_s = odom_msg->twist.twist.angular.z;
}

void RosClass::robotPoseCallback(const geometry_msgs::Pose &robot_msg)  
{
    m_robotPose  = robot_msg;
} 

void RosClass::goalPoseCallback(const move_base_msgs::MoveBaseActionGoal &goal_msg)
{
    ROS_WARN("RosClass:: Received New goal");
    m_goalPose.position      = goal_msg.goal.target_pose.pose.position;
    m_goalPose.orientation   = goal_msg.goal.target_pose.pose.orientation;

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

/*
void RosClass::heartbeatDiagnosticsCallback(const robot_activity_msgs::StateConstPtr &state)
{
    
    int diagnosticsStatus = state->state;
    
    //Error detected from the diagnostics feedback
    if(diagnosticsStatus == 4)
    {
        m_diagnosticsState = false;
    }
    else if(diagnosticsStatus == 5)
    {
        m_diagnosticsState = true;
    }
    //ROS_INFO("heartbeatDiagnosticsCallback  : %d",diagnosticsStatus);
    // diagnostics_action_state_ = state->state;    
    // if (diagnostics_action_state_ != 5) 
    // {
    //     executed_ = true;
    // }
}
*/



/* GETTERS */
geometry_msgs::PoseArray RosClass::getPath()
{
    return m_poseArray;
}

geometry_msgs::PoseArray RosClass::getGlobalPath()
{
    return m_globalArray;
}
geometry_msgs::Pose RosClass::getGoalPose()
{

    return m_goalPose;
    
}


geometry_msgs::Pose RosClass::getRobotPose()
{
    return m_robotPose;
    
}

bool RosClass::getGoalStatus()
{
    return m_mbGoalReached;
}


bool RosClass::getDiagnosticsStatus()
{
    return m_diagnosticsState;
}

geometry_msgs::Twist RosClass::getOdom()
{
    //geometry_msgs::Twist odom;

    //odom.linear.x = m_odomVel__m_s;
    //odom.angular.z = m_odomOmega__rad_s;
    //return odom;
    return m_odomMsg;
}


/* SETTERS */

void RosClass::sendVelocityCmd(geometry_msgs::Twist m_cmd)
{
    cmdVelPub.publish(m_cmd);
}

void RosClass::sendStatus(std_msgs::Bool m_status)
{
    amrStatusPub.publish(m_status);
}

void RosClass::sendDebugData(controller_msgs::Debug debug)
{
    debugPub.publish(debug);
}

void RosClass::sendSmoothedGlobalPlan(nav_msgs::Path path)
{
    smoothedGlobalPlan.publish(path);
}

