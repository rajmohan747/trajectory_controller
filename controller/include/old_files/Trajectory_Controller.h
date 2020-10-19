#ifndef TRAJECTORY_CONTROLLER_H 
#define TRAJECTORY_CONTROLLER_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <string>

#include "Transformations.h"
#include "AHighLevelController.h"
#include "Param.h"

#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>


#define MICRO_SECONDS 0.000001 
#define ROBOT_POSE_FREQUENCY 10
#define DECELERATION_DISTANCE 1.8
#define REVERSE_DISTANCE 2.0

using namespace std;
using namespace Navigation;

/**
 * @class Computations
 * @brief Class to compute all the necessary calculations for computing the angular and linear velocities
 */

class TrajectoryController : public AHighLevelController
{
public:
    /**
    * @brief  Constructor for the Computations
    */
    TrajectoryController();

    /**
    * @brief  Destructor for the Computations
    */

    ~TrajectoryController();

    /**
     * @brief Dynamic reconfigure callback that sets the params obtained from the config files
     * @param config The configurations obtained
     */
    dynamic_reconfigure::Server<controller::controllerConfig> server;
    dynamic_reconfigure::Server<controller::controllerConfig>::CallbackType f;
    void reconfigureCallback(controller::controllerConfig& config, uint32_t level);    
    
    /**
    * @brief  initialize the member varaibles obtained
    * @param array_msg the array of poses obtained from the local planner
    * @param robo the pose of the robot in map frame
    * @param last_pose the final goal received by the robot in map frame
    */
    void initialize(std::vector<Params::PathData>& array_msg,geometry_msgs::Pose& robo,geometry_msgs::Pose& last_pose,geometry_msgs::Twist& odom,bool m_newPlanReceived) override;
    
    /* Sends the debug messages to the wrapper node    */
    controller_msgs::Debug getDebugData() override;

    /*sends the velocity commands to the wrapper */
    geometry_msgs::Twist  executeController() override;

    /*  resets the parameters used in the controller  */
    void resetControllerParameters() override;

    /*  checks the status whether robot has reached the goal or not   
    * @return the goal completion task    */
    bool isGoalReached() override;

private:

    /* Predicts the robot position whenever the updated robot pose data is not received.This is based on robot's prev velocity and position    */
    void predictRobotPosition(geometry_msgs::Twist& odometryMsg,geometry_msgs::Pose& robot_pose);

    /* This function localizes the current position of the robot on the trajectory and writes 
       the reference point for error calculation and control calculation to member variables */
    void localisation();

    /*  pre process the variables needed to follow processes in the control law    */
    void calculateErrors();

    // get the transformations in required coordinate frame
    void getAllTransformations();

    // get the angles in Euler
    void getEulerAngles();

    // Check if an inplace turn is required
    void setStates();

    void computePathDirection();
    /*  Get information from path to decide if it requires an inplace turn and angular correction is required while path tracking
     @return if the path satisfies inplace rotation    */
    bool rotateInplaceCheck();

    /* longitudinal controller to obtain the linear velocity    */
    double longitudinalController();

    // Calculate the linear velocity based on curvarure
    double computeLinearVelocity();

    /* sets the linear velocity bounds with consideration of linear acceleration parameter @return the calculated linear velocity   */
    void linearLimit(double &linearVel);

    /*  lateral controller to obtain the angular velocity  */
    double lateralController();

    /* Helper functions to provide the control law  */
    double controlLaw();

        
    /*Pid for generating angular velocities for correction during inplace rotations*/

    void pidAngularVelocity(double &angularVelocity);

    /* Helper functions to get the derivative of equations */
    void getDerivatives();

    /*  perform an inplace rotation with the required angle 
    * @param angle_limit the angle passed for an inplace tolerance    */
    void rotationController();

    /*  sets the angular velocity bounds with consideration of angular acceleration parameter
    * @return the calculated angular velocity    */
    void angularLimit(double &angularVel);
    
    /*Checks whether an updated robot pose is received */
    bool isNewPoseReceived(geometry_msgs::Pose& robot_pose);

    // Interpolates the pathdata with given interpolation ratio
    Params::PathData interpolatePathData(const Params::PathData& input1, const Params::PathData& input2, const double interpolationRatio);

    /* Defining the frames being used */
    std::string m_targetFrame   =  "/sf_frame";
    std::string m_baseFrame     =  "/base_footprint";
    std::string m_globalFrame   =  "/map";
    std::string m_fbFrame       =  "/FB_Frame";
   
    /* A vector for storing all relevant infos for the controller */
    std::vector<Params::PathData> m_processedPathArray;


    /* Reconfigure parameters */

    controller::controllerConfig last_config_;
    controller::controllerConfig default_config_;

    tf::TransformListener poseTransform;


    /**
     * @brief Mutex lock parameters
     */
    
    boost::recursive_mutex configuration_mutex_;

    bool m_rotationController   = false;
    bool m_latchTarget          = false;
    bool m_goalReached          = false;
    bool m_setup;
    bool m_angleLatch           = false;
    bool m_latching;
    bool m_enableFirst,m_enableSecond,m_enableThird,m_enableFourth;
    bool m_useVelocityProfiling,m_useBackwardMotion;
    bool m_reverseMode          = false;
    bool m_isCheckBackward      = false;

    int count = 0;
    double m_lastAngleError = 0.0;
    double m_robotAngle, m_inplaceAngle, m_inplaceAngleDiff, m_finalGoalAngle;
    double m_sfFrameX,m_sfFrameY;
    double m_linearVel__m_s, m_angularVel__rad_s, m_curvature;
    
    /* Control law paramters  */
    double m_derivS, m_derivY, m_derivDelta;
    double m_approachControl; /*factor that controls the robot along curves*/
    double m_cteControl; /*considering the cross track error*/
    double m_orientationControl; /*considers the orientation correction*/
    double m_feedForwardControl; /* correct the curvature and velocity*/
    
    /* Opimization tuning parameters */
    double m_delta;
    double m_distanceToGoal__m;
    double m_lastAngularVel,m_lastLinearVel;

    double m_xyProjLength; 
    double m_crossTrackError,m_orientationError;
    double m_resultant;



    int m_msgSize             = 0;
    int m_counterInplace      = 0;
    int m_activeCounter       = 0;
    int m_activeCounterPred   = 0;
    
    geometry_msgs::PoseArray m_pathArray;            /*Entire sequence of local plan points*/
    geometry_msgs::Pose m_robotPose,m_lastPose,m_sfRefPoint,m_inplacePoint;   /* Robot's current pose ,final goal position and refernce point-origin for S-F frame 
                                                                                            in the global frame*/
    geometry_msgs::Pose m_robotPoseFB;                       /*Pose in the {SF} frame*/ //m_robotPoseSF_
    geometry_msgs::Pose m_errorRefPoint,m_lastRobotPose;
    geometry_msgs::Twist m_odom;


    Transformations m_Transformations;
    Params m_Params;

   
};

#endif
