#ifndef LYAPUNOV_CONTROLLER_H
#define LYAPUNOV_CONTROLLER_H




#include "AbstractController.h"
#include "Param.h"
#include "Utilities.h"
#include "Transformations.h"

#include <controller/lyapunovConfig.h>

#define MICRO_SECONDS 0.000001 
#define ROBOT_POSE_FREQUENCY 10
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
    void predictRobotPosition();
    void setStates();
    void linearVelocityLimit(double &linearVelocity);
    void angularVelocityLimit(double &angularVelocity);
    void computeDerivatives();
    void localization();
    void predictRobotPosition(geometry_msgs::Twist &odometryMsg, geometry_msgs::Pose &robot_pose);
    void resetControllerParameters()override;


    bool isNewPoseReceived(geometry_msgs::Pose &robot_pose);
    bool isInplaceRotate();
    bool isGoalReached()override;

    double rotationController();
    double controlLaw();
    double computeLinearVelocity();
    double lateralController();
    double longitudinalController();
    double computeCurvature(const geometry_msgs::Pose& sfRefPoint,const geometry_msgs::Pose& sfPreRefPoint);

    geometry_msgs::Pose interpolatePathData(const geometry_msgs::Pose& firstPose, const geometry_msgs::Pose& secondPose, const double interpolationRatio);
    geometry_msgs::Twist sendCommandVelocity() override;

    /**
    * @brief Dynamic reconfigure callback that sets the params obtained from the config files
    * @param config The configurations obtained
    */
    dynamic_reconfigure::Server<controller::lyapunovConfig> server;
    dynamic_reconfigure::Server<controller::lyapunovConfig>::CallbackType f;

    /* Reconfigure parameters */

    controller::lyapunovConfig lastConfig;
    controller::lyapunovConfig defaultConfig;

    bool m_setup              = false;
    bool m_angleLatch         = false; 
    bool m_latchTarget        = false;
    bool m_rotationController = false;
    bool m_goalReached        = false;
    bool m_enableFirst,m_enableSecond,m_enableThird,m_enableFourth;

    int m_activeCounter             = 0;
    int m_counterInplace            = 0;
    int m_activeCounterPred         = 0;
    int m_pathSize                  = 0;
    double m_lastLinearVelocity     = 0.0; 
    double m_lastAngularVelocity    = 0.0;
    double m_crossTrackError        = 0.0;
    double m_orientationError       = 0.0;

    double m_robotAngle             = 0.0;
    double m_inplaceAngle           = 0.0;
    double m_finalGoalAngle         = 0.0;
    double m_inplaceAngleDifference = 0.0;
    
    double m_curvature              = 0.0;
    double m_delta                  = 0.0;

    double m_derivativeS            = 0.0;
    double m_derivativeY            = 0.0;
    double m_derivativeDelta        = 0.0;

    double m_approachControl        = 0.0;
    double m_crossTrackControl      = 0.0;
    double m_orientationControl     = 0.0;
    double m_feedForwardControl     = 0.0;

    double m_linearVel__m_s         = 0.0;
    double m_angularVel__rad_s      = 0.0;
    double m_distanceToGoal__m      = 0.0;
    //double m_decelerationDistance__m= 0.0; 

    double m_sfFrameX               = 0.0;
    double m_sfFrameY               = 0.0;

    std::string m_globalFrame = "map";
    std::string m_fbFrame     = "errorFrame";
    std::string m_sfFrame     = "sf_frame";



    geometry_msgs::PoseArray m_pathArray;
    geometry_msgs::Pose m_robotPose,m_goalPose,m_lastRobotPose,m_inplacePose;
    geometry_msgs::Pose m_errorRefPoint,m_sfRefPoint;

    geometry_msgs::Pose m_robotPoseFB;

    geometry_msgs::Twist m_odom;

    tf::Transform m_transformErrorFrame;
    tf::TransformBroadcaster m_broadcasterErrorFrame;    
    
    /*Class objects*/
    Params m_Params;
    Transformations m_Transformations;
};
#endif
