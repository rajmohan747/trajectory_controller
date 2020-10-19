#ifndef AHIGHLEVELCONTROLLER_H
#define AHIGHLEVELCONTROLLER_H

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <controller/controllerConfig.h> // ToDo: remove after debug
#include <controller_msgs/Debug.h> // ToDo: remove after debug
#include "Param.h"

namespace Navigation
{
    class AHighLevelController 
    {
        
    public:
        /**
        * is called by client module in order to initialize the required paramters for the controllers
        */
        virtual void initialize(std::vector<Params::PathData>& ,geometry_msgs::Pose& ,geometry_msgs::Pose& ,geometry_msgs::Twist& ,bool) = 0;
        
        /**
        * is called by client module in order to obtain the command velocity
        */
        virtual geometry_msgs::Twist  executeController() = 0;
       
        /**
        * return true the goal is reached
        */
        virtual bool isGoalReached() = 0;
       
        /**
        * reset all states & parameters
        */
        virtual void resetControllerParameters() = 0;

        /**
        * Debug function to test the controller 
        * ToDo: remove after debug
        */
        virtual controller_msgs::Debug getDebugData() = 0; 

    };
}

#endif