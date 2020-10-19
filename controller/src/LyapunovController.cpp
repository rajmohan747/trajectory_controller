#include "LyapunovController.h"

/**
* @brief  Constructor for the LyapunovController
*/
LyapunovController::LyapunovController()
{
    ROS_INFO("LyapunovController:: Constructor initialized");
    
    /*  Reconfigure callback  */
    f = boost::bind(&LyapunovController::reconfigureCallback, this, _1, _2);
    server.setCallback(f);
}

/**
* @brief  Destructor for the LyapunovController
*/
LyapunovController::~LyapunovController()
{
}


/**
* @brief  Reconfigurable callback for recofiguring parameters
*/
void LyapunovController::reconfigureCallback(controller::lyapunovConfig &config, uint32_t level)
{
    ROS_INFO_ONCE("LyapunovController:: Reconfiguration call back");
    std::mutex reconfig_mtx;

    std::unique_lock<std::mutex> reconfigLock(reconfig_mtx);

    /*The first time we're called, we just want to make sure we have the original configuration
     * if restore defaults is set on the parameter server, prevent looping    */
    if (config.restore_defaults)
    {
        ROS_WARN_ONCE("LyapunovController:: Re-setting default values");
        config                  = defaultConfig;
        config.restore_defaults = false;
    }

    /*Robot parameters */
    m_Params.robot.m_maxLinearVel__m_s              = config.maxLinearVel__m_s;
    m_Params.robot.m_minLinearVel__m_s              = config.minLinearVel__m_s;
    m_Params.robot.m_maxAngularVel__rad_s           = config.maxAngularVel__rad_s;
    m_Params.robot.m_maxInplaceAngularVel__rad_s    = config.maxInplaceAngularVel__rad_s;
    m_Params.robot.m_linearAccLimit__m_s2           = config.linearAccLimit__m_s2;
    m_Params.robot.m_linearDeccLimit__m_s2          = config.linearDeccLimit__m_s2;
    m_Params.robot.m_angularAccLimit__rad_s2        = config.angularAccLimit__rad_s2;
    m_Params.robot.m_lateralAcceleration__m_s2      = config.lateralAcceleration__m_s2;


    /*Tolerance parameters */
    m_Params.robotTolerance.m_xyGoalTolerance__m    = config.xyGoalTolerance__m;
    m_Params.robotTolerance.m_yawTolerance__rad     = config.yawTolerance__rad;
    m_Params.robotTolerance.m_inplaceTolerance__rad = config.inplaceTolerance__rad;

    /* Control law tuning parameters */
    m_Params.controlLaw.m_k1Gain                    = config.k1Gain;
    m_Params.controlLaw.m_k2Gain                    = config.k2Gain;
    m_Params.controlLaw.m_kDeltaGain                = config.kDeltaGain;
    m_Params.controlLaw.m_gamma                     = config.gamma;
    m_Params.controlLaw.m_thetaA                    = config.thetaA;
    m_Params.controlLaw.m_lookAhead                 = config.lookAhead;
    m_Params.controlLaw.m_inplaceRadius             = config.inplaceRadius;
    m_Params.controlLaw.m_Kp                        = config.Kp;
    m_Params.controlLaw.m_Kd                        = config.Kd;

    /*Enabling/Disabling individual terms of control law.Can be used when tuning the robot first time/in new simulation environment*/

    // m_enableFirst = config.enableFirst;
    // m_enableSecond = config.enableSecond;
    // m_enableThird = config.enableThird;
    // m_enableFourth = config.enableFourth;

    if (!m_setup)
    {
        lastConfig    = config;
        defaultConfig = config;
        m_setup       = true;
        return;
    }
    lastConfig = config;
}


void LyapunovController::initializeController(geometry_msgs::PoseArray &array_msg,geometry_msgs::Pose &robot_pose, geometry_msgs::Pose &goal_pose, geometry_msgs::Twist &odom_msg,bool newPlanReceived)
{
    m_pathArray  = array_msg;
    m_robotPose  = robot_pose;
    m_goalPose   = goal_pose;

    if(newPlanReceived)
    {
        ROS_INFO("initializeController::Reset counter");
        m_activeCounter = 0;
    }
    ROS_INFO("Current path size : %d   counter : %d",m_pathArray.poses.size(),m_activeCounter);

    /*Predict robot position
    predictRobotPosition()
    */
    m_robotAngle    = m_Transformations.QuatToEuler(m_robotPose.orientation.z,m_robotPose.orientation.w);
    localization();
    

    
}


void LyapunovController::localization()
{   

    // /*TO DO: Minimize the lines of codes , make it more compact and avoid unnecessary use of local variables*/

    // double xPathSegment         = m_pathArray.poses[m_activeCounter+1].position.x - m_pathArray.poses[m_activeCounter].position.x;
    // double yPathSegment         = m_pathArray.poses[m_activeCounter+1].position.y - m_pathArray.poses[m_activeCounter].position.y;

    // double xRobotPathSegment    = m_robotPose.position.x - m_pathArray.poses[m_activeCounter].position.x;
    // double yRobotPathSegment    = m_robotPose.position.y - m_pathArray.poses[m_activeCounter].position.y;

    // double pathSegmentLength    = xPathSegment*xPathSegment + yPathSegment*yPathSegment;

    // double projectionFactor;

    // /*Proj of u on v =   v*(u.v / ||v^2||)   */

    // if (pathSegmentLength <= 0)
    // {
    //     projectionFactor = 0.0;
    // }
    // else
    // {
    //     projectionFactor = (xRobotPathSegment * xPathSegment + yRobotPathSegment * yPathSegment) / pathSegmentLength;
    // }


    // /*Ensuring projectionFactor to stay in [0,1]*/
    // if (projectionFactor <= 0)
    // {
    //     projectionFactor = 0.0;
    // }
    // else if (projectionFactor >= 1)
    // {
    //     projectionFactor = 1;
    // }

    // /*Note: A = segment length , B = projection , C = robot-Path segment    B = C - A*/

    // double xProjection            =   xRobotPathSegment - projectionFactor*xPathSegment;
    // double yProjection            =   yRobotPathSegment - projectionFactor*yPathSegment;
    // double xyProjectionLength     =   xProjection*xProjection+yProjection*yProjection;

    int i           = m_activeCounter;
    int pathSize    = m_pathArray.poses.size();

    double xyProjectionLength = 10.0;  /*Initializing to a bigger value,so as by the end of first iteration, the min value will be first element*/
    double projectionFactor;

    for(int i = m_activeCounter; i < pathSize ; i++)
    {
        double xPathSegmentCurrent      = m_pathArray.poses[i + 1].position.x - m_pathArray.poses[i].position.x;
        double yPathSegmentCurrent      = m_pathArray.poses[i + 1].position.y - m_pathArray.poses[i].position.y;

        double xRobotPathSegmentCurrent = m_robotPose.position.x - m_pathArray.poses[i].position.x;
        double yRobotPathSegmentCurrent = m_robotPose.position.y - m_pathArray.poses[i].position.y;

        double pathSegmentLengthCurrent = xPathSegmentCurrent * xPathSegmentCurrent + yPathSegmentCurrent * yPathSegmentCurrent;
        double projectionFactorCurrent;

        /*Proj of u on v =   v*(u.v / ||v^2||)   */

        if (pathSegmentLengthCurrent <= 0)
        {
            projectionFactorCurrent = 0.0;
        }
        else
        {
            projectionFactorCurrent = (xRobotPathSegmentCurrent * xPathSegmentCurrent + yRobotPathSegmentCurrent * yPathSegmentCurrent) / pathSegmentLengthCurrent;
        }

        /*Ensuring projectionFactor to stay in [0,1]*/
        if (projectionFactorCurrent <= 0)
        {
            projectionFactorCurrent = 0.0;
        }
        else if (projectionFactorCurrent >= 1)
        {
            projectionFactorCurrent = 1;
        }
        /*Note: A = segment length , B = projection , C = robot-Path segment    B = C - A*/

        double xProjectionCurrent            =   xRobotPathSegmentCurrent - projectionFactorCurrent*xPathSegmentCurrent;
        double yProjectionCurrent            =   yRobotPathSegmentCurrent - projectionFactorCurrent*yPathSegmentCurrent;
        double xyProjectionLengthCurrent     =   xProjectionCurrent*xProjectionCurrent + yProjectionCurrent*yProjectionCurrent;
        
        /*Finding the closest projection from Robot - Segment ,based on length of projection*/


        
        if  (xyProjectionLengthCurrent  < xyProjectionLength)  //0.01
        {
            xyProjectionLength = xyProjectionLengthCurrent;          
            projectionFactor   = projectionFactorCurrent;
            m_activeCounter    = i;
        }
        
    
    }

    /* Calculate the coordinates of the localised path */

    double xPathSegmentLocalized        =  m_pathArray.poses[m_activeCounter+1].position.x - m_pathArray.poses[m_activeCounter].position.x;
    double yPathSegmentLocalized        =  m_pathArray.poses[m_activeCounter+1].position.y - m_pathArray.poses[m_activeCounter].position.y;
    double xRobotPathSegmentLocalized   = m_robotPose.position.x - m_pathArray.poses[m_activeCounter].position.x;
    double yRobotPathSegmentLocalized   = m_robotPose.position.y - m_pathArray.poses[m_activeCounter].position.y;


    /*To find the direction of the orientation error, a cross product between relevant path segment and robot position is calculated
     Cross product to find the correct sign of cross-track-error*/
    if (xPathSegmentLocalized*yRobotPathSegmentLocalized > yPathSegmentLocalized*xRobotPathSegmentLocalized)
    {
        xyProjectionLength      = -1*sqrt(xyProjectionLength);
    }
    else if(xPathSegmentLocalized*yRobotPathSegmentLocalized < yPathSegmentLocalized*xRobotPathSegmentLocalized)
    {
        xyProjectionLength      = sqrt(xyProjectionLength);
    }
    else
    {
        xyProjectionLength        = 0;
    }
    

    /* Interpolate the data using projectionFactor and define a new frame*/
    m_errorRefPoint      =  interpolatePathData(m_pathArray.poses[m_activeCounter],m_pathArray.poses[m_activeCounter + 1],projectionFactor);


    /* Publish the TF and broadcast the new frame*/
    m_Transformations.sendTransformations(m_globalFrame, m_fbFrame, m_errorRefPoint);


    double errorRefPointOrientation    = m_Transformations.QuatToEuler(m_errorRefPoint.orientation.z,m_errorRefPoint.orientation.w);

    /*Derived the control parameters for the control law: cross track error and orientation error*/
    m_crossTrackError             = xyProjectionLength;
    m_orientationError            = Utilities::shortest_angular_distance(m_robotAngle,errorRefPointOrientation);
    ROS_WARN("Robot angle : %f    cross track : %f      orientation error : %f ",m_robotAngle,m_crossTrackError,m_orientationError);
}

/**
* @brief  Interpolate the two different poses using the concepts of projection and slerp
* @returns Interpolated pose in geometry_msgs::Pose
*/
geometry_msgs::Pose LyapunovController::interpolatePathData(const geometry_msgs::Pose& firstPose, const geometry_msgs::Pose& secondPose, const double interpolationRatio)
{
    geometry_msgs::Pose result;
    geometry_msgs::Quaternion currentOrientation, nextOrientation;
    currentOrientation = firstPose.orientation;
    nextOrientation    = secondPose.orientation;
    
    /* Interpolate orientations using the concept of slerps*/
    result.orientation = m_Transformations.slerp(currentOrientation, nextOrientation, interpolationRatio);

    /*Interpolate the cooridnate values of two points by making use of interpolationRatio */
    result.position.x = firstPose.position.x + interpolationRatio * (secondPose.position.x - firstPose.position.x);
    result.position.y = firstPose.position.y + interpolationRatio * (secondPose.position.y - firstPose.position.y);
    result.position.z = 0;

    return result;
}


double LyapunovController::controlLaw()
{
}

/**
* @brief  Limits the linear velocities
*/
void LyapunovController::linearVelocityLimit(double &linearVelocity)
{
    /**
    * @brief Handling nan and inf linear velocities
    */
    if (!std::isfinite(linearVelocity) || std::isnan(linearVelocity))
    {
        ROS_ERROR("LyapunovController:: v error........!!!");
        linearVelocity = 0.0;
        return ;
    }

    /**
    * @brief Limiting velocity based on v = u +/- a*t
    */
    if (linearVelocity > (m_lastLinearVelocity + (m_Params.robot.m_linearAccLimit__m_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS)))
    {
        linearVelocity = (m_lastLinearVelocity + (m_Params.robot.m_linearAccLimit__m_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS));
    }
    else if (linearVelocity < (m_lastLinearVelocity - (m_Params.robot.m_linearDeccLimit__m_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS))) //m_lastLinearVel
    {
        linearVelocity = (m_lastLinearVelocity - (m_Params.robot.m_linearDeccLimit__m_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS));
    }
    else
    {
        linearVelocity = linearVelocity;
    }

    /**
    * @brief Clipping the linear velocity limit
    */

    if (linearVelocity > m_Params.robot.m_maxLinearVel__m_s)
    {
        linearVelocity = m_Params.robot.m_maxLinearVel__m_s;
    }
    else if (linearVelocity < m_Params.robot.m_minLinearVel__m_s)
    {

        linearVelocity = m_Params.robot.m_minLinearVel__m_s;
    }

    m_lastLinearVelocity = linearVelocity;
}


/**
* @brief  Limits the angular velocities
*/
void LyapunovController::angularVelocityLimit(double &angularVelocity)
{
    /**
    * @brief Handling nan and inf angular velocities
    */
    if (!std::isfinite(angularVelocity) || std::isnan(angularVelocity))
    {
        ROS_ERROR("LyapunovController:: w error........!!!");
        angularVelocity = 0.0;
        return;
    }

    /**
    * @brief ensuring angular velocity complies to w = w_last +/- alpha *delta_time
    */

    /*To Do : In real robot , replace m_lastAngularVel with m_odom.angular.z*/

    if (angularVelocity > (m_lastAngularVelocity + (m_Params.robot.m_angularAccLimit__rad_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS)))
    {
        angularVelocity = (m_lastAngularVelocity + (m_Params.robot.m_angularAccLimit__rad_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS)); //m_lastAngularVel
    }
    else if (angularVelocity < (m_lastAngularVelocity - (m_Params.robot.m_angularAccLimit__rad_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS)))
    {
        angularVelocity = (m_lastAngularVelocity - (m_Params.robot.m_angularAccLimit__rad_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS));
    }
    else
    {
        angularVelocity = angularVelocity;
    }

    /**
    * @brief Clipping the angular velocity limit
    */
    if (angularVelocity > m_Params.robot.m_maxAngularVel__rad_s)
    {
        angularVelocity = m_Params.robot.m_maxAngularVel__rad_s;
    }
    else if (angularVelocity < -m_Params.robot.m_maxAngularVel__rad_s)
    {
        angularVelocity = -m_Params.robot.m_maxAngularVel__rad_s;
    }

    /**
    * @brief To ensure robot doesn't fall down to extreme low angular velocities while correction
    */

    m_lastAngularVelocity = angularVelocity;
}