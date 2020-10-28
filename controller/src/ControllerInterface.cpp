#include "ControllerInterface.h"
#include "RosClass.h"

/**
* @brief  Constructor for the LyapunovController
*/
ControllerInterface::ControllerInterface(std::unique_ptr<RosClass> rs):m_rosClass(std::move(rs))
{
    ROS_INFO("ControllerInterface:: Constructor initialized");
    m_controller = new LyapunovController();
}   

/**
* @brief  Destructor for the LyapunovController
*/
ControllerInterface::~ControllerInterface()
{

}

/**
* @brief  Control flow for the  LyapunovController
*/
void ControllerInterface::controlFlow()
{

    bool newGoalReceived = comparePose(m_goalPose,m_rosClass->getGoalPose());
    if(newGoalReceived)
    {
        ROS_ERROR("ControllerInterface:: Reset parameters");
        resetParameters();

        // // Set the goalPose member variable, so that controller is only reset after receiving new goal
        // m_goalPose = m_getVariables->getGoalPose();
    }


    m_poseArray     = m_rosClass->getPath();
    m_odom          = m_rosClass->getOdom();
    m_robotPose     = m_rosClass->getRobotPose();
    m_goalPose      = m_rosClass->getGoalPose();

    m_planSize      = m_poseArray.poses.size();
    

     

    if(m_planSize > 0)
    {
        m_currentPath       = m_poseArray.poses[0];
        m_newPlanReceived   = comparePose(m_currentPath,m_lastPath);
        if(m_newPlanReceived)
        {
            m_initialPlanReceived = true;
        }
        
    }
   

    bool plannerCondition = ((m_planSize > 0) && m_initialPlanReceived && !m_controllerGoalReached);
        
    if (plannerCondition)
    {
        m_controller->initializeController(m_poseArray,m_robotPose, m_goalPose, m_odom,m_newPlanReceived);
        m_lastPath               = m_currentPath;
        m_cmdVel                 = m_controller->sendCommandVelocity();   
        m_controllerGoalReached  = m_controller->isGoalReached();
    }
    else
    {    
        stopVelocityCommand();
        ROS_INFO_ONCE("ControllerInterface:: Reached goal or planner condition violated");
    }
    publishMessages();
}

/**
* @brief  Resets the parameters for the ControllerInterface class
*/
void ControllerInterface::resetParameters()
{
    m_controllerGoalReached = false;
    m_controller->resetControllerParameters();
}

/**
* @brief  Compare two different positions
* @param  two poses as geometry_msgs::Pose poseA and poseB
* @returns True if two poses are different ,else return false
*/
bool ControllerInterface::comparePose(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB)
{
    return ((poseA.position.x != poseB.position.x) || (poseA.position.y != poseB.position.y) || (poseA.orientation.z != poseB.orientation.z) || (poseA.orientation.w != poseB.orientation.w));
}

/**
* @brief  Resets the velocity commands 
*/
void ControllerInterface::stopVelocityCommand()
{
    m_cmdVel.linear.x      = 0.0;
    m_cmdVel.angular.z     = 0.0;
    ROS_INFO_ONCE("ControllerInterface:: Called stop command velocity");
}

/**
* @brief  Sends the messages to the ROS class
*/
void ControllerInterface::publishMessages()
{
    m_rosClass->sendVelocityCmd(m_cmdVel);
}
/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lyapunov_controller");
    std::unique_ptr<RosClass> rosPointer(new RosClass);
    ControllerInterface controller(std::move(rosPointer));
    ros::Rate rate(20);


    while(ros::ok())
    {
        controller.controlFlow();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
