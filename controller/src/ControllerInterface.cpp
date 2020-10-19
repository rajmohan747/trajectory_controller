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
void ControllerInterface::controlFlow()
{
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
   

    bool plannerCondition = ((m_planSize > 0) && m_initialPlanReceived);
        
    if (plannerCondition)
    {
        m_controller->initializeController(m_poseArray,m_robotPose, m_goalPose, m_odom,m_newPlanReceived);
        m_lastPath       = m_currentPath;
    }

}

bool ControllerInterface::comparePose(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB)
{
    return ((poseA.position.x != poseB.position.x) || (poseA.position.y != poseB.position.y) || (poseA.orientation.z != poseB.orientation.z) || (poseA.orientation.w != poseB.orientation.w));
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
