#ifndef NAVIGATION_WRAPPER_H
#define NAVIGATION_WRAPPER_H

#include <iostream>
#include <ros/ros.h>
#include "Trajectory_Controller.h"
#include "Ros_Class.h"
#include "pathProcessing.h"

using namespace std;
  /**
   * @class NavigationWrapper
   * @brief A controller that follows the trajectory provided by a planner.
   */

class NavigationWrapper
{
public:
        /**
     * @brief  Constructor for the NavigationWrapper
     */

    NavigationWrapper(Environment::AEnvironment* enviromentVars);

        /**
        * @brief  Destructor for the NavigationWrapper
        */
    ~NavigationWrapper();


    void controlFlow();

        /**
     * @brief  A function to reset the parameters when required
     */

    void resetParameters();

        /**
     * @brief  A function to publish the stopping commands for the robot
     */

    void stopVelocityCommand();
    
    /**
     * @brief  A function to publish the messages out from the controller
     */

    void publishMsgs();

    /**
     * @brief  A function to publish the debug messages out from the controller
     */
    void publishDebugMsgs();

    //**ToDo make generic to be moved to Math class 
    inline bool comparePose(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB) 
    {
        return ((poseA.position.x != poseB.position.x) || (poseA.position.y != poseB.position.y) || (poseA.orientation.z != poseB.orientation.z) || (poseA.orientation.w != poseB.orientation.w));
    }    

private:
    geometry_msgs::Pose m_robotPose,m_goalPose;
    geometry_msgs::Twist m_odom;
    geometry_msgs::PoseArray m_poseArray, m_globalPoseArray;  
    geometry_msgs::Pose m_currentPath,m_lastPath;
    nav_msgs::Path smoothedPlan;

    /*Objects for publishers*/
    std_msgs::Bool m_status;
    geometry_msgs::Twist m_cmd;
    

    int m_pathSize = 0;

    /*status of goal from move base*/
    bool m_mbGoalReached;

    /*status of goal from controller*/
    bool m_controllerGoalReached;

    /*status of diagnostics node*/
    bool m_diagnosticsStatus = true;

    /*shows whether initial plan is received from the local planner*/
    bool m_initialPlanReceived,m_newPlanReceived;

    /*last status of goal from controller.Used for avoiding publishing result multiple times*/
    bool m_lastControllerStatus = true;

    bool m_goalPathCheck;
    // Pointers to abstract classes
    Navigation::AHighLevelController* m_controller_low_level;
    Environment::AEnvironment* m_getVariables;

	// Object of type of pathProcessing class
    pathProcessing mpathProcessing;

    std::vector<Params::PathData> m_processedPath,m_lastGlobalProcessedPathArray;

};
#endif
