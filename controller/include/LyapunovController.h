#ifndef LYAPUNOV_CONTROLLER_H
#define LYAPUNOV_CONTROLLER_H




#include "AbstractController.h"
#include "Param.h"
#include "Utilities.h"
#include "Transformations.h"

#include <controller/lyapunovConfig.h>

#define MICRO_SECONDS 0.000001 
using namespace std;
using namespace Navigation;
/**
* @class LyapnovController
* @brief A controller that follows the trajectory provided by any local planner.
*/

class LyapunovController : public AbstractController
{
public:
    /**
         * @brief  Constructor for the LyapnovController
         */
    LyapunovController();

    /**
        * @brief  Destructor for the NavigationWrapper
        */
    ~LyapunovController();


    void initializeController(geometry_msgs::PoseArray &array_msg,geometry_msgs::Pose &robot_pose, geometry_msgs::Pose &last_goal, geometry_msgs::Twist &odom_msg,bool newPlanReceived) override;
private:

    void reconfigureCallback(controller::lyapunovConfig &config, uint32_t level);


    void localization();
    
    geometry_msgs::Pose interpolatePathData(const geometry_msgs::Pose& firstPose, const geometry_msgs::Pose& secondPose, const double interpolationRatio);

    double controlLaw();

    void linearVelocityLimit(double &linearVelocity);

    void angularVelocityLimit(double &angularVelocity);

    /**
    * @brief Dynamic reconfigure callback that sets the params obtained from the config files
    * @param config The configurations obtained
    */
    dynamic_reconfigure::Server<controller::lyapunovConfig> server;
    dynamic_reconfigure::Server<controller::lyapunovConfig>::CallbackType f;

    /* Reconfigure parameters */

    controller::lyapunovConfig lastConfig;
    controller::lyapunovConfig defaultConfig;

    bool m_setup = false;
    int m_activeCounter             = 0;
    double m_lastLinearVelocity     = 0.0; 
    double m_lastAngularVelocity    = 0.0;
    double m_crossTrackError        = 0.0;
    double m_orientationError       = 0.0;   
    double m_robotAngle             = 0.0;

    std::string m_globalFrame = "map";
    std::string m_fbFrame     = "errorFrame";



    geometry_msgs::PoseArray m_pathArray;
    geometry_msgs::Pose m_robotPose,m_goalPose;
    geometry_msgs::Pose m_errorRefPoint;
    tf::Transform m_transformErrorFrame;
    tf::TransformBroadcaster m_broadcasterErrorFrame;    
    
    /*Class objects*/
    Params m_Params;
    Transformations m_Transformations;
};
#endif
