#include "Navigation_Wrapper.h"
#include <math.h>
#include <ros/console.h> 
#include <boost/thread.hpp>
#include <pthread.h>
#include <mutex>
#include <exception>
#include <utility> 
#include "Ros_Class.h"

/**
* @brief  Constructor for the NavigationWrapper
*/
NavigationWrapper::NavigationWrapper( Environment::AEnvironment* envVars):m_getVariables(envVars),m_mbGoalReached(false),m_controllerGoalReached(false)
{
    int m_controllerType = m_getVariables->getParams("/controller/controller_type");

    switch(m_controllerType)
    {
        case 1:
            ROS_WARN_ONCE("Controller 1");
            m_controller_low_level = new TrajectoryController();
            break;
        case 2:
        default:
            ROS_ERROR("Invalid parameter");
            break;
    }
    
}

/**
* @brief  Destructor for the NavigationWrapper
*/
NavigationWrapper::~NavigationWrapper()
{

}

/**
 * @brief reset the parameters
*/
void NavigationWrapper::resetParameters()
{
    m_mbGoalReached         = false;
    m_controllerGoalReached = false;
    m_lastControllerStatus  = true;
    m_pathSize              = 0;
    m_initialPlanReceived   = false;
    m_newPlanReceived       = false;
    m_goalPathCheck         = false;
    m_poseArray.poses.clear();
    m_globalPoseArray.poses.clear();
    m_lastGlobalProcessedPathArray.clear();
    m_lastGlobalProcessedPathArray.resize(0);
    m_processedPath.clear();
    m_processedPath.resize(0);
    m_controller_low_level->resetControllerParameters();
    ROS_WARN_ONCE("NavigationWrapper:: Resetted parameters");
}

/**
 * @brief function for stopping the robot
*/
void NavigationWrapper::stopVelocityCommand()
{
    m_cmd.linear.x      = 0.0;
    m_cmd.angular.z     = 0.0;
    ROS_INFO_ONCE("NavigationWrapper:: Called stop command velocity");
}

/**
 * @brief Used for publishing the messages out of the controller
*/
void NavigationWrapper::publishMsgs()
{
    m_getVariables->sendVelocityCmd(m_cmd);
    if(m_lastControllerStatus != m_controllerGoalReached)
    {
        m_status.data   = m_controllerGoalReached;
        m_getVariables->sendStatus(m_status);
        m_lastControllerStatus = m_controllerGoalReached;
        //ROS_WARN("NavigationWrapper:: m_lastControllerStatus: %d  m_controllerGoalReached: %d",m_lastControllerStatus,m_controllerGoalReached);
    }
}

void NavigationWrapper::publishDebugMsgs()
{
    controller_msgs::Debug debug;
    debug       = m_controller_low_level->getDebugData(); // Add this to interface file 
    m_getVariables->sendDebugData(debug);
}

/**
 * @brief Main controller flow
*/
void NavigationWrapper::controlFlow()
{
    // Check if a new goal position has been given
    bool newGoalReceived = comparePose(m_goalPose,m_getVariables->getGoalPose());
    if(newGoalReceived)
    {
        ROS_WARN("NavigationWrapper:: Reset parameters");
        resetParameters();

        // Set the goalPose member variable, so that controller is only reset after receiving new goal
        m_goalPose = m_getVariables->getGoalPose();
    }

    // Since the new goal received sets the path size to zero, this should be called after the reset.
    // get the local path
    m_poseArray             = m_getVariables->getPath();

    // get the global path
    m_globalPoseArray       = m_getVariables->getGlobalPath();

    // size of the global path in this case (this will be changed after abstration and directly reading the parameters)
    m_pathSize              = m_globalPoseArray.poses.size(); 
       
    // Set the first point of path in member variable current path 
    if(m_pathSize > 0)
    {
        m_currentPath       =  m_globalPoseArray.poses[0];  //global_pose_array
        // Check if the goal and the last point in the set of path points is same or a previous goal is being worked on
        m_goalPathCheck = (m_goalPose.position.x == m_globalPoseArray.poses[m_pathSize-1].position.x) || (m_goalPose.position.y == m_globalPoseArray.poses[m_pathSize-1].position.y) || (m_goalPose.orientation.z == m_globalPoseArray.poses[m_pathSize-1].orientation.z) || (m_goalPose.orientation.w == m_globalPoseArray.poses[m_pathSize-1].orientation.w);
        //ROS_WARN("TEST %d", m_goalPathCheck);
    }

    // Check if for the same goal a new plan is sent.
    m_newPlanReceived       = comparePose(m_currentPath,m_lastPath);

    // Conditions for calculating new smoothed plan
    // a) New plan is received &&
    // b) Goal is still not reached (this makes sure that smoothing is not done unnecessary even after the goal is reached)
    // c) Size of path received is greater than 0. After reset the path is sometimes 0, leading to smoothed path error
    if(m_newPlanReceived && (!m_controllerGoalReached) && m_goalPathCheck && m_pathSize >= 4)
    {
        /*Once this is set,the reset will happen during a new goal receive*/
        m_initialPlanReceived = true;
        bool splineError = false;
        //if(m_pathSize >= 4){
        m_progetProcessedPathcessedPath = mpathProcessing.(m_globalPoseArray,m_odom,m_lastGlobalProcessedPathArray,splineError); 
        //}
        // This is only for visualization in rviz
        if (!splineError)
        {
            geometry_msgs::PoseStamped globalPose;

            smoothedPlan.poses.clear();
            smoothedPlan.poses.resize(0);
            smoothedPlan.header.frame_id = "/map";
            smoothedPlan.header.stamp = ros::Time(0);
            for (unsigned i =0; i< m_processedPath.size();i++)
            {
                globalPose.header.frame_id = "/map";
                globalPose.header.stamp = ros::Time(0);
                globalPose.pose.position =  m_processedPath[i].m_path.position;
                globalPose.pose.orientation =  m_processedPath[i].m_path.orientation;
                smoothedPlan.poses.push_back(globalPose)  ;
            }
           // ROS_ERROR("DISTANCE %f", m_processedPath[m_processedPath.size() - 1].m_distFromInital);
            m_getVariables->sendSmoothedGlobalPlan(smoothedPlan);
        }
    }
    //enable this to check error condition3
    ROS_ERROR_COND(m_pathSize < 4, "Length of path is less than 4; Cannot get processed path");
    
    bool planner_condition  = ((m_pathSize > 0) && !m_controllerGoalReached && m_initialPlanReceived && (m_processedPath.size()>0)); 
    if(planner_condition)
    {

        /* Get the relevant data from Ros class */ 
        m_mbGoalReached = m_getVariables->getGoalStatus();
        m_odom          = m_getVariables->getOdom();
        m_robotPose     = m_getVariables->getRobotPose();
        /* @brief Main part of the code where tracking controller being called            */
        m_controller_low_level->initialize(m_processedPath, m_robotPose,m_goalPose,m_odom,m_newPlanReceived);
        m_cmd                    = m_controller_low_level-> executeController();
        m_controllerGoalReached  = m_controller_low_level->isGoalReached();
        m_lastPath      =  m_currentPath;    
    }
    else
    {    
        stopVelocityCommand();
        ROS_INFO_ONCE("NavigationWrapper:: Reached goal or planner condition violated");
    }
    
    publishMsgs();
    publishDebugMsgs();
}       

/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_controller");
    NavigationWrapper controller(new RosClass());
    ros::Rate rate(20);

    //boost::thread* controller_thread = new boost::thread(boost::bind(&NavigationWrapper::controlFlow, &controller));

    while(ros::ok())
    {
        controller.controlFlow();
        rate.sleep();
        ros::spinOnce();
    }getPath
    return 0;
}