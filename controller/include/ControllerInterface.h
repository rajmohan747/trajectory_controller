#ifndef CONTROLLER_INTERFACE_H
#define CONTROLLER_INTERFACE_H

#include <iostream>
#include "LyapunovController.h"
#include "RosClass.h"

using namespace std;
/**
* @class LyapnovController
* @brief A controller that follows the trajectory provided by any local planner.
*/

class ControllerInterface
{
    public:
       
        /**
         * @brief  Constructor for the LyapnovController
         */
        ControllerInterface(std::unique_ptr<RosClass> rs);

        /**
        * @brief  Destructor for the NavigationWrapper
        */
        ~ControllerInterface();

        void controlFlow();
         
    private:
    


        void stopVelocityCommand();   
        void publishMessages();
        void resetParameters();     

        bool comparePose(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB);

        geometry_msgs::PoseArray m_poseArray;
        geometry_msgs::Pose m_robotPose,m_goalPose;
        geometry_msgs::Pose m_currentPath,m_lastPath;
        geometry_msgs::Twist m_odom,m_cmdVel;

        bool m_newPlanReceived      = false;
        bool m_initialPlanReceived  = false;
        bool m_controllerGoalReached= false;
        
        int m_planSize   =  0;
        std::unique_ptr<RosClass> m_rosClass;
        Navigation::AbstractController *m_controller;  
        LyapunovController *test;

        
};
#endif
