#include "Trajectory_Controller.h"
#include <math.h>
#include <ros/console.h>
#include <boost/thread.hpp>
#include <pthread.h>
#include <mutex>
#include <exception>
#include <utility>

/*  Constructor for the TrajectoryController*/
TrajectoryController::TrajectoryController() : m_setup(false)
{
    m_Params.robotTime.m_startTime = m_Params.robotTime.m_endTime = MathFunc::micros<uint64_t>();
    m_Params.robotTime.m_robotStartTime = m_Params.robotTime.m_robotEndTime = MathFunc::micros<uint64_t>();

    /*  Reconfigure callback  */
    f = boost::bind(&TrajectoryController::reconfigureCallback, this, _1, _2);
    server.setCallback(f);
}

/*  Destructor for the TrajectoryController */
TrajectoryController::~TrajectoryController()
{
}

/* Reconfigure callback   */
void TrajectoryController::reconfigureCallback(controller::controllerConfig &config, uint32_t level)
{
    ROS_INFO_ONCE("In reconfiguration call back");
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    /*The first time we're called, we just want to make sure we have the original configuration
     * if restore defaults is set on the parameter server, prevent looping    */
    if (config.restore_defaults)
    {
        ROS_WARN_ONCE("TrajectoryController:: Re-setting default values");
        config = default_config_;
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
    m_useVelocityProfiling                          = config.useVelocityProfiling;
    m_useBackwardMotion                             = config.useBackwardMotion;

    /*Tolerance parameters */
    m_Params.robotTolerance.m_xyGoalTolerance__m = config.xyGoalTolerance__m;
    m_Params.robotTolerance.m_yawTolerance__rad = config.yawTolerance__rad;
    m_Params.robotTolerance.m_inplaceTolerance__rad = config.inplaceTolerance__rad;

    /* Control law tuning parameters */
    m_Params.controlLaw.m_k1Gain = config.k1Gain;
    m_Params.controlLaw.m_k2Gain = config.k2Gain;
    m_Params.controlLaw.m_kDeltaGain = config.kDeltaGain;
    m_Params.controlLaw.m_gamma = config.gamma;
    m_Params.controlLaw.m_thetaA = config.thetaA;
    m_Params.controlLaw.m_lookAhead = config.lookAhead;
    m_Params.controlLaw.m_inplaceRadius = config.inplaceRadius;
    m_Params.controlLaw.m_Kp = config.Kp;
    m_Params.controlLaw.m_Kd = config.Kd;

    /*Enabling/Disabling individual terms of control law.Can be used when tuning the robot first time/in new simulation environment*/

    m_enableFirst = config.enableFirst;
    m_enableSecond = config.enableSecond;
    m_enableThird = config.enableThird;
    m_enableFourth = config.enableFourth;

    if (!m_setup)
    {
        last_config_ = config;
        default_config_ = config;
        m_setup = true;
        return;
    }
    last_config_ = config;
}

/*initialize the member varaibles obtained    */
void TrajectoryController::initialize(std::vector<Params::PathData> &array_msg, geometry_msgs::Pose &robot_pose, geometry_msgs::Pose &last_goal, geometry_msgs::Twist &odom_msg, bool newPlanReceived)
{
    /* Resetting the counters for the projections ,once a new plan is received  */
    if ((newPlanReceived) && (!m_rotationController))
    {
        count++;
        m_counterInplace = 0;
        m_activeCounter = 0;
    }

    // else if(newPlanReceived && m_reverseMode)
    // {
    //     m_counterInplace = 0;
    //     m_activeCounter = 0;
    // }
    // if(m_reverseMode)
    // {
    //     m_rotationController = false;
    // }
    /* As long as the robot is in the Rotation Controller,new  plan won't be accepted    */
    if (!m_rotationController) // || m_reverseMode)
    {        
        ROS_ERROR("-------------------- NEW -------------------");
        m_processedPathArray.clear();
        m_processedPathArray.resize(0);

        m_processedPathArray = array_msg;
        m_msgSize            = m_processedPathArray.size();

        m_lastPose           = last_goal;
    }
    
    if(m_useBackwardMotion && !m_isCheckBackward)
    {
        ROS_ERROR("CHECK BACKWARD");
        m_isCheckBackward = true;
        computePathDirection();
    }

    m_odom = odom_msg;

    // predict the robot pose based on current robot pose and odom
    predictRobotPosition(odom_msg, robot_pose);

    // perform localisation on the processed path and get the new control point based on lookahead.
    // Both the feedforward and feedback frames
    localisation();

    // Get the errors required for the control law
    calculateErrors();
}

/*  pre process the parameters required for the controller execution */

void TrajectoryController::calculateErrors()
{
    // First calculate all the transformations for calculating error
    getAllTransformations();

    // Convert the quaternion to Euler angles for the controller
    getEulerAngles();

    // Set the state of the controller
    setStates();

    // Calculation of Xerror, Yerror and orientationError
    m_sfFrameX = fabs(m_robotPoseFB.position.x);
    m_sfFrameY = m_crossTrackError;
}

void TrajectoryController::getAllTransformations()
{
    // Get the transformations
    m_robotPoseFB = m_Transformations.getTransformations(m_globalFrame, m_fbFrame, m_robotPose);
   
}

void TrajectoryController::getEulerAngles()
{
    /*Robot angle,inplace angle and final goal pose's angle w.r.t basefootprint*/
    m_inplaceAngle   = m_Transformations.QuatToEuler(m_inplacePoint.orientation.z, m_inplacePoint.orientation.w);
    m_finalGoalAngle = m_Transformations.QuatToEuler(m_lastPose.orientation.z, m_lastPose.orientation.w);
}

void TrajectoryController::setStates()
{
    /**
     * @brief  It will be used only if rotation controller is performing action
     * @return either m_inplaceTolerance__rad or m_yawTolerance__rad
     */

    m_latching = ((m_distanceToGoal__m) < m_Params.robotTolerance.m_xyGoalTolerance__m) || (m_latchTarget == true);

    m_rotationController = false;

    if (m_latching)
    {
        m_inplaceAngleDiff = MathFunc::shortest_angular_distance<double>(m_robotAngle, m_finalGoalAngle);
        m_rotationController = (fabs(m_inplaceAngleDiff) >= m_Params.robotTolerance.m_yawTolerance__rad);
        m_latchTarget = true;
        ROS_ERROR("m_rotationController:  %d  m_latchTarget :%d  m_inplaceAngleDiff: %f : %f",m_rotationController,m_latchTarget,m_inplaceAngleDiff,m_distanceToGoal__m);
    }
    else
    {
        m_inplaceAngleDiff = MathFunc::shortest_angular_distance<double>(m_robotAngle, m_inplaceAngle);
        m_rotationController = rotateInplaceCheck();
        ROS_WARN("m_rotationController:  %d  m_latchTarget :%d  m_inplaceAngleDiff: %f : %f",m_rotationController,m_latchTarget,m_inplaceAngleDiff,m_distanceToGoal__m);
    }
    if(m_reverseMode &&  m_latchTarget)
    {
        m_reverseMode   = false;
    }
    m_goalReached = m_latchTarget && (!m_rotationController);
}



bool TrajectoryController::rotateInplaceCheck()
{
    /*Ensures that once robot gets within the entry tolerance (angle_ > m_inplaceTolerance__rad) ,till the exit condition  (angle_ < m_yawTolerance__rad)
     is not satisfied,it remains in the state of Rotation controller*/
    bool entryCondition = (fabs(m_inplaceAngleDiff) > m_Params.robotTolerance.m_inplaceTolerance__rad) ? true : false;

    if (entryCondition || m_angleLatch)
    {
        m_angleLatch = true;
        bool exitCondition = (fabs(m_inplaceAngleDiff) < m_Params.robotTolerance.m_yawTolerance__rad) ? true : false;

        if (exitCondition)
        {
            m_angleLatch = false;
        }
        return (!exitCondition);
    }

    return entryCondition;
}

double TrajectoryController::longitudinalController()
{
    ROS_INFO_ONCE("TrajectoryController:: Longitudinal Controller");
    if(m_useVelocityProfiling)
    {
        //m_linearVel__m_s = m_processedPathArray[m_activeCounterPred].m_velLinear;
        
        //ROS_WARN(" ACTIVE COUNTER %d linear vel  %f  distance %f ",m_activeCounter, m_linearVel__m_s, m_processedPathArray[m_processedPathArray.size() - 1].m_distFromInital);

    }
    else
    {
        m_linearVel__m_s = computeLinearVelocity();
    }
    //m_linearVel__m_s = (m_useVelocityProfiling) ? m_processedPathArray[m_activeCounterPred].m_velLinear :  computeLinearVelocity(); 
    linearLimit(m_linearVel__m_s);

    //ROS_WARN("VELOCITY IS after limit %f ",m_linearVel__m_s);
    //ROS_INFO("use backward %d reverse %d", m_useBackwardMotion, m_reverseMode);

    m_linearVel__m_s = (m_reverseMode) ? m_linearVel__m_s*(-1) : m_linearVel__m_s;

    //ROS_WARN("VELOCITY %f ",m_linearVel__m_s);
    return m_linearVel__m_s;
}

double TrajectoryController::computeLinearVelocity()
{
    double velocity, velocityDistance, velocityAcceleration;
    velocityDistance = m_Params.robot.m_maxLinearVel__m_s;

    if (m_distanceToGoal__m <= DECELERATION_DISTANCE)
    {
        double velocityDistanceSquare = std::max(pow(m_odom.linear.x, 2) - 2 * (DECELERATION_DISTANCE - m_distanceToGoal__m) * m_Params.robot.m_linearDeccLimit__m_s2, pow(m_Params.robot.m_minLinearVel__m_s, 2));
        velocityDistance = std::sqrt(velocityDistanceSquare);
        //ROS_ERROR("Deccelerating at :%f  with :   %f", m_distanceToGoal__m, velocityDistance);
    }

    if ((!std::isfinite(m_curvature)) || std::isnan(m_curvature))
    {
        velocityAcceleration = 0;
    }
    else
    {
        velocityAcceleration = std::fabs(sqrt(m_Params.robot.m_lateralAcceleration__m_s2 / fabs(m_curvature)));
        //ROS_INFO("velocityAcceleration : %f  m_minLinearVel__m_s :%f", velocityAcceleration, m_Params.robot.m_minLinearVel__m_s);
    }
    velocity = std::max(std::min(velocityAcceleration, velocityDistance), m_Params.robot.m_minLinearVel__m_s);

    if (m_rotationController)
    {
        velocity = 0.0;
    }
    return velocity;
}

void TrajectoryController::linearLimit(double &linearVel)
{
    //ROS_WARN("IN LINEAR LIMIT INPUT %f",linearVel);
    if (!std::isfinite(linearVel) || std::isnan(linearVel))
    {
        ROS_ERROR("TrajectoryController:: v error........!!!");
        linearVel = 0.0;
        return;
    }
    /* Ensuring linear velocity complies to v= u +/- acc*delta_time  */

    //ROS_INFO("Entry velocity : %f  m_distanceToGoal__m  : %f", linearVel, m_distanceToGoal__m);

    if(m_rotationController && m_reverseMode && count!=1)
    {
        ROS_ERROR("STOP AND TURN count %d",count);
        linearVel = 0.0;
    }
    /*To Do : In real robot , replace m_lastLinearVel with m_odom.linear.x*/
    if(m_rotationController == false || m_reverseMode)
    {

        if (linearVel > (m_lastLinearVel + (m_Params.robot.m_linearAccLimit__m_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS)))
        {
            linearVel = (m_lastLinearVel + (m_Params.robot.m_linearAccLimit__m_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS));
        }
        else if (linearVel < (m_lastLinearVel - (m_Params.robot.m_linearDeccLimit__m_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS))) //m_lastLinearVel
        {
            linearVel = (m_lastLinearVel - (m_Params.robot.m_linearDeccLimit__m_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS));
        }
        else
        {
            linearVel = linearVel;
        }

        /**
             * @brief Clipping the linear velocity limit
            */

        if (linearVel > m_Params.robot.m_maxLinearVel__m_s)
        {
            linearVel = m_Params.robot.m_maxLinearVel__m_s;
        }
        else if (linearVel < m_Params.robot.m_minLinearVel__m_s) 
        {

            linearVel = m_Params.robot.m_minLinearVel__m_s;
        }
    }
    else
    {
        linearVel = 0.0;
    }
    //ROS_WARN("Exit velocity : %f Time  :  %f", linearVel, m_Params.robotTime.m_deltaTime * MICRO_SECONDS);
    m_lastLinearVel = linearVel;
    
}

double TrajectoryController::lateralController()
{
    ROS_INFO_ONCE("TrajectoryController:: Lateral Controller");
    if (m_rotationController && count!=1) // !m_reverseMode)
    {
        ROS_WARN("ROTATION");
        /* Rotation Controller   */
        rotationController();
    }
    else
    {
        ROS_WARN("CONTROL LAW");
        /* Trajectory tracking controller  */
        m_angularVel__rad_s = controlLaw();
    }
    angularLimit(m_angularVel__rad_s);
    // if(m_reverseMode)
    // {
    //     m_angularVel__rad_s = (-1)*m_angularVel__rad_s;
    // }
    return m_angularVel__rad_s;
}

void TrajectoryController::rotationController()
{
    /* Setting linear velocity to 0.0 as its pure inplace rotation*/
    //m_linearVel__m_s    = 0.0;
    pidAngularVelocity(m_angularVel__rad_s);
}

/*Pid for generating angular velocities for correction during inplace rotations*/

void TrajectoryController::pidAngularVelocity(double &angularVelocity)
{
    double deltaError = m_inplaceAngleDiff - m_lastAngleError;
    angularVelocity = m_Params.controlLaw.m_Kp * m_inplaceAngleDiff + m_Params.controlLaw.m_Kd * (deltaError / m_Params.robotTime.m_deltaTime);

    if (fabs(angularVelocity) > fabs(m_Params.robot.m_maxInplaceAngularVel__rad_s))
    {
        angularVelocity = (m_Params.robot.m_maxInplaceAngularVel__rad_s) * MathFunc::sign(angularVelocity);
    }
    m_lastAngleError = m_inplaceAngleDiff;
}

/* control law to find the corresponding angular velocity    */
double TrajectoryController::controlLaw()
{
    if ((!std::isfinite(m_curvature)) || std::isnan(m_curvature))
    {
        ROS_ERROR("TrajectoryController:: Control law fail captured");
        return 0.0;
    }
    getDerivatives();

    double theta_diff = MathFunc::shortest_angular_distance<double>(m_orientationError, m_delta);
    double theta_term = (sin(m_orientationError) - sin(m_delta)) / (theta_diff);

    m_approachControl = m_derivDelta * m_enableFirst;
    m_cteControl = (-1 * m_Params.controlLaw.m_gamma * m_sfFrameY * m_linearVel__m_s * theta_term) * m_enableSecond;
    ROS_WARN("ORIENT:  %f    MDELTA:  %f ", m_orientationError,m_delta);
    m_orientationControl = (-1 * m_Params.controlLaw.m_k2Gain * (m_orientationError - m_delta)) * m_enableThird;
    m_feedForwardControl = m_curvature * m_derivS * m_enableFourth;

    if (theta_diff == 0.0)
    {
        m_cteControl = 0.0;
    }
    m_resultant = (m_approachControl + m_cteControl + m_orientationControl + m_feedForwardControl);
    return m_resultant;
}

/* @brief Control law helper functions     */
void TrajectoryController::getDerivatives()
{
    m_derivS = (m_linearVel__m_s * cos(m_orientationError) + m_Params.controlLaw.m_k1Gain * m_sfFrameX);
    m_derivY = (-1 * m_curvature * m_derivS * m_sfFrameX) + (m_linearVel__m_s * sin(m_orientationError));
    m_derivDelta = (-1 * m_Params.controlLaw.m_thetaA * m_Params.controlLaw.m_kDeltaGain * m_derivY * (1 - (tanh(m_Params.controlLaw.m_kDeltaGain * m_sfFrameY) * tanh(m_Params.controlLaw.m_kDeltaGain * m_sfFrameY))));
    m_delta = (-1 * m_Params.controlLaw.m_thetaA * tanh(m_Params.controlLaw.m_kDeltaGain * m_sfFrameY));
}

/* Bounding the  angular velocities */
void TrajectoryController::angularLimit(double &angularVel)
{
    ROS_INFO("ANGULAR INPUT %f",angularVel);
    if (!std::isfinite(angularVel) || std::isnan(angularVel))
    {
        ROS_ERROR("TrajectoryController:: w error........!!!");
        angularVel = 0.0;
        return;
    }
    

    /**
     * @brief ensuring angular velocity complies to w = w_last +/- alpha *delta_time
    */

    /*To Do : In real robot , replace m_lastAngularVel with m_odom.angular.z*/


    if (angularVel > (m_lastAngularVel + (m_Params.robot.m_angularAccLimit__rad_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS)))
    {
        angularVel = (m_lastAngularVel + (m_Params.robot.m_angularAccLimit__rad_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS)); //m_lastAngularVel
    }
    else if (angularVel < (m_lastAngularVel - (m_Params.robot.m_angularAccLimit__rad_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS)))
    {
        angularVel = (m_lastAngularVel - (m_Params.robot.m_angularAccLimit__rad_s2 * m_Params.robotTime.m_deltaTime * MICRO_SECONDS));
    }
    else
    {
        angularVel = angularVel;
    }

    /**
         * @brief Clipping the angular velocity limit
        */
    if (angularVel > m_Params.robot.m_maxAngularVel__rad_s)
    {
        angularVel = m_Params.robot.m_maxAngularVel__rad_s;
    }
    else if (angularVel < -m_Params.robot.m_maxAngularVel__rad_s)
    {
        angularVel = -m_Params.robot.m_maxAngularVel__rad_s;
    }

    /**
     * @brief To ensure robot doesn't fall down to extreme low angular velocities while correction
    */

    m_lastAngularVel = angularVel;
    ROS_INFO("ANGULAR output %f",angularVel);
    //return angularVel;
}

/* This function localizes the current position of the robot on the trajectory
* and writes the reference point for error calculation and control calculation to member variables */
void TrajectoryController::localisation()
{
    // Initilize the activeCounter from last counter (activeCounter = 0 in first iteration)  m_processedPathArray[i].m_path.position.x
    // Initialization of the search
    double xPathSegment = m_processedPathArray[m_activeCounter + 1].m_path.position.x - m_processedPathArray[m_activeCounter].m_path.position.x;
    double yPathSegment = m_processedPathArray[m_activeCounter + 1].m_path.position.y - m_processedPathArray[m_activeCounter].m_path.position.y;
    double xylengthPathSegment = xPathSegment * xPathSegment + yPathSegment * yPathSegment;

    double xPosPathSegment = m_robotPose.position.x - m_processedPathArray[m_activeCounter].m_path.position.x;
    double yPosPathSegment = m_robotPose.position.y - m_processedPathArray[m_activeCounter].m_path.position.y;

    double xyProjection;
    if (xylengthPathSegment <= 0)
    {
        xyProjection = 0.0;
    }
    else
    {
        xyProjection = (xPosPathSegment * xPathSegment + yPosPathSegment * yPathSegment) / xylengthPathSegment;
    }

    if (xyProjection <= 0)
    {
        xyProjection = 0.0;
    }
    else if (xyProjection >= 1)
    {
        xyProjection = 1;
    }

    double xProjection = xPosPathSegment - xyProjection * xPathSegment;
    double yProjection = yPosPathSegment - xyProjection * yPathSegment;
    double xyProjLength = xProjection * xProjection + yProjection * yProjection; // Taking sqrt since only required for comparison

    // Start the search from next counter and continue till end of the path to find the best match
    // The correct path idx is stored in m_activeCounter
    int i = m_activeCounter + 1;
    int numberofPathIdxSearched = 0;
    while(i < m_msgSize -1 && numberofPathIdxSearched<200) // the 2nd condition limits the search area/idxs
    {
        double xPathSegmentNext = m_processedPathArray[i + 1].m_path.position.x - m_processedPathArray[i].m_path.position.x;
        double yPathSegmentNext = m_processedPathArray[i + 1].m_path.position.y - m_processedPathArray[i].m_path.position.y;
        double xylengthPathSegmentNext = xPathSegmentNext * xPathSegmentNext + yPathSegmentNext * yPathSegmentNext;

        double xPosPathSegmentNext = m_robotPose.position.x - m_processedPathArray[i].m_path.position.x;
        double yPosPathSegmentNext = m_robotPose.position.y - m_processedPathArray[i].m_path.position.y;

        double xyProjectionNext;

        if (xylengthPathSegmentNext <= 0)
        {
            xyProjectionNext = 0.0;
        }
        else
        {
            xyProjectionNext = (xPosPathSegmentNext * xPathSegmentNext + yPosPathSegmentNext * yPathSegmentNext) / xylengthPathSegmentNext;
        }

        if (xyProjectionNext <= 0)
        {
            xyProjectionNext = 0.0;
        }

        else if (xyProjectionNext >= 1)
        {
            xyProjectionNext = 1;
        }

        double xProjectionNext = xPosPathSegmentNext - xyProjectionNext * xPathSegmentNext;
        double yProjectionNext = yPosPathSegmentNext - xyProjectionNext * yPathSegmentNext;
        double xyProjLengthNext = xProjectionNext * xProjectionNext + yProjectionNext * yProjectionNext;

        double orientationAtPathPoint = m_Transformations.QuatToEuler(m_processedPathArray[i].m_path.orientation.z, m_processedPathArray[i].m_path.orientation.w);
        // if(m_reverseMode)
        // {
        //     orientationAtPathPoint = (orientationAtPathPoint < 0.0) ? 3.14 + orientationAtPathPoint : 3.14 - orientationAtPathPoint;
        //     ROS_INFO("orientationAtPathPoint inside %f",orientationAtPathPoint );
        // }

        double absOrientationDifference = std::fabs(MathFunc::shortest_angular_distance<double>(m_robotAngle,orientationAtPathPoint));
        
        
        
        //ROS_WARN("ABS ORIENT %f  xyPRojNext  %f  xyProjLen   %f ", absOrientationDifference, xyProjLengthNext,xyProjLength);
        //add reverse abso
        // Here also check if the localised point is in the same direction by ensuring the angle difference to be small
        if ((xyProjLengthNext < xyProjLength) && (absOrientationDifference < m_Params.robotTolerance.m_inplaceTolerance__rad)) // To not let the active counter change because of floating point compare
        //if  (xyProjLengthNext  < xyProjLength)
        {

            xyProjLength = xyProjLengthNext;
            xyProjection = xyProjectionNext;
            m_activeCounter = i;
            //ROS_ERROR("ACTIVE COUNTER INSIDE %d",m_activeCounter);

        }
        else
        {
            numberofPathIdxSearched++;
        }
        i++;
    }

    // Calculate the coordinates of the localised path
    double xPathSegmentLocalized = m_processedPathArray[m_activeCounter + 1].m_path.position.x - m_processedPathArray[m_activeCounter].m_path.position.x;
    double yPathSegmentLocalized = m_processedPathArray[m_activeCounter + 1].m_path.position.y - m_processedPathArray[m_activeCounter].m_path.position.y;
    double xPosPathSegmentLocalized = m_robotPose.position.x - m_processedPathArray[m_activeCounter].m_path.position.x;
    double yPosPathSegmentLocalized = m_robotPose.position.y - m_processedPathArray[m_activeCounter].m_path.position.y;

    if (std::fabs(yPosPathSegmentLocalized) < 0.001)
    {
        yPosPathSegmentLocalized = 0;
    }
    if (std::fabs(yPathSegmentLocalized) < 0.001)
    {
        yPathSegmentLocalized = 0;
    }

    // To find the direction of the orientation error, a cross product between relevant path segment and robot position is calculated
    // Cross product to find the correct sign of cross-track-error
    if (xPathSegmentLocalized * yPosPathSegmentLocalized > yPathSegmentLocalized * xPosPathSegmentLocalized)
    {
        m_xyProjLength = -1 * sqrt(xyProjLength);
    }
    else if (xPathSegmentLocalized * yPosPathSegmentLocalized < yPathSegmentLocalized * xPosPathSegmentLocalized)
    {
        m_xyProjLength = sqrt(xyProjLength);
    }
    else
    {
        m_xyProjLength = 0;
    }

    // Get the interpolated Pathdata, the interpolated pathdata is stored in this pathdata
    Params::PathData interpolatedPath_errorPoint = interpolatePathData(m_processedPathArray[m_activeCounter], m_processedPathArray[m_activeCounter+1], xyProjection);

    // write the interpolated path data to position to pose and publish TF 
    m_errorRefPoint = interpolatedPath_errorPoint.m_path;

    // Publish the TF
    m_Transformations.sendTransformations(m_globalFrame, m_fbFrame, m_errorRefPoint);

    /*=========================================================================================*/
    /*===== Write the orientation and cross track error to member variable (debug values) =====*/
    /*=========================================================================================*/

    m_crossTrackError = m_xyProjLength;
    double xyProjOrientation = m_Transformations.QuatToEuler(m_errorRefPoint.orientation.z, m_errorRefPoint.orientation.w);
    // if(m_reverseMode)
    // {
    //     xyProjOrientation = (xyProjOrientation < 0.0) ? 3.14 + xyProjOrientation : 3.14 - xyProjOrientation;
    //     ROS_INFO("xyProjOrientation inside %f",xyProjOrientation );
    // }
    ROS_WARN("XY Proj %f  robot %f ",xyProjOrientation , m_robotAngle);
    m_orientationError = MathFunc::shortest_angular_distance(m_robotAngle, xyProjOrientation);
    ROS_INFO("Orientation error %f",m_orientationError );
    
    // Calculate the distance remaining to reach the goal
    double distanceNextPoint = MathFunc::euclideanDistance<double>(m_processedPathArray[m_activeCounter + 1].m_path.position.x, m_processedPathArray[m_activeCounter + 1].m_path.position.y, m_errorRefPoint.position.x, m_errorRefPoint.position.y);
    m_distanceToGoal__m = distanceNextPoint + m_processedPathArray[m_processedPathArray.size() - 1].m_distFromInital - m_processedPathArray[m_activeCounter + 1].m_distFromInital;

    /*=========================================================================================*/
    /*= Calculation of the feedforward frame at a distance of lookahead from projection point =*/
    /*====== Here the Projection point is found out, along with the interpolation ratio =======*/
    /*=========================================================================================*/

    // ToDO: Calculate variable lookahead
    double velocityLookAhead = m_Params.controlLaw.m_lookAhead;

    // For calculation of the control law, define a new coordinate frame at a distance "m_lookAhead" from this error frame
    // Calculate the distance travelled from the active m_errorRefPoint and active counter +1 => this is initial distance (same as distanceNextPoint)
    double distanceFromErrorFrame = 0.0;
    double distanceLast = 0.0;
    m_activeCounterPred = m_activeCounter + 1; //max velLa
    while (distanceFromErrorFrame < (velocityLookAhead - distanceNextPoint) && m_activeCounterPred < m_msgSize - 1)
    {
        distanceFromErrorFrame += MathFunc::euclideanDistance<double>(m_processedPathArray[m_activeCounterPred].m_path.position.x, m_processedPathArray[m_activeCounterPred].m_path.position.y, m_processedPathArray[m_activeCounterPred + 1].m_path.position.x, m_processedPathArray[m_activeCounterPred + 1].m_path.position.y);
        if (distanceFromErrorFrame < (m_Params.controlLaw.m_inplaceRadius - distanceNextPoint))
        {
            m_counterInplace = m_activeCounterPred;
        }
        //if()
        if (distanceFromErrorFrame < (velocityLookAhead - distanceNextPoint))
        {
            distanceLast = distanceFromErrorFrame;
            m_activeCounterPred++;
        }
    }
    double distanceRemaining = velocityLookAhead - distanceLast - distanceNextPoint;

    /*=========================================================================================*/
    /*========= Here the interpolation is done, to find the exact point of lookahead ==========*/
    /*=========================================================================================*/
    double distanceRatioRemaining;
    Params::PathData interpolatedPath;
    // To ensure that the loop was atleast entered once, i.e. the lookahead is atleast larger than distance to next point
    if (distanceRemaining > 0)
    {
        // To ensure that the for loop from above was not exited because end of path
        if (m_activeCounterPred < m_msgSize - 1)
        {
            // Distance that remains to be calculated
            distanceRatioRemaining = 0;
            double distancePathSegment = MathFunc::euclideanDistance<double>(m_processedPathArray[m_activeCounterPred + 1].m_path.position.x, m_processedPathArray[m_activeCounterPred + 1].m_path.position.y, m_processedPathArray[m_activeCounterPred].m_path.position.x, m_processedPathArray[m_activeCounterPred].m_path.position.y);

            if (distancePathSegment > 0)
            {
                distanceRatioRemaining = distanceRemaining / distancePathSegment;
            }

            // Get the interpolated Pathdata, the interpolated pathdata is stored in this pathdata
            interpolatedPath = interpolatePathData(m_processedPathArray[m_activeCounterPred], m_processedPathArray[m_activeCounterPred + 1], distanceRatioRemaining);
        }
        else
        {
            distanceRatioRemaining = 0.0;

            // Get the interpolated Pathdata, the interpolated pathdata is stored in this pathdata
            interpolatedPath = interpolatePathData(m_processedPathArray[m_activeCounterPred], m_processedPathArray[m_activeCounterPred], distanceRatioRemaining);
        }
    }
    // In case loop was never entered. Calculate the interpolation from errorpoint to next pathsegment point
    else
    {
        distanceRatioRemaining = std::fabs(velocityLookAhead / distanceNextPoint);

        // Get the interpolated Pathdata, the interpolated pathdata is stored in this pathdata
        interpolatedPath = interpolatePathData(interpolatedPath_errorPoint, m_processedPathArray[m_activeCounterPred], distanceRatioRemaining);
    }

    // write the interpolated path data to position to pose and publish TF
    m_sfRefPoint = interpolatedPath.m_path;

    // write the curvature from the interpolated data
    m_curvature = interpolatedPath.m_curvatureValue;

    if(m_reverseMode)
    {
        m_curvature =(-1)*m_curvature;
    }
    // write velocity from the interpolated data
    m_linearVel__m_s = interpolatedPath.m_velLinear;
    ROS_INFO("VEL %f",m_linearVel__m_s);



    ROS_ERROR("ACTIVE %d     PRED %d  ROTATE %d   REVERSE %d", m_activeCounter, m_activeCounterPred, m_rotationController, m_reverseMode);
    
    // set the counter for inplace
    m_inplacePoint = m_processedPathArray[m_counterInplace].m_path;

    // Publish the SF frame(same as the FF Frame for PD Control)
    m_Transformations.sendTransformations(m_globalFrame, m_targetFrame, m_sfRefPoint);
}

/*Predicts the robot position whenever the updated robot pose data is not received.This is based on robot's prev velocity and position*/
void TrajectoryController::predictRobotPosition(geometry_msgs::Twist &odometryMsg, geometry_msgs::Pose &robot_pose)
{
    geometry_msgs::Quaternion quat_msg;

    m_Params.robotTime.m_robotStartTime = MathFunc::micros<uint64_t>();
    m_Params.robotTime.m_robotDeltaTime += (m_Params.robotTime.m_robotStartTime - m_Params.robotTime.m_robotEndTime);

    float dt = 0.005; // Step size of the integration in secs, define as macro
    int nSteps = (m_Params.robotTime.m_robotDeltaTime * MICRO_SECONDS) / dt;
    m_robotPose = robot_pose;
    m_robotAngle = m_Transformations.QuatToEuler(m_robotPose.orientation.z, m_robotPose.orientation.w);
    
    bool newRobotDatareceived = isNewPoseReceived(m_robotPose);

    if (newRobotDatareceived || ((m_Params.robotTime.m_robotDeltaTime * MICRO_SECONDS) > (1.0 / ROBOT_POSE_FREQUENCY)))
    {
        m_Params.robotTime.m_robotStartTime = m_Params.robotTime.m_robotEndTime = MathFunc::micros<uint64_t>();
        m_Params.robotTime.m_robotDeltaTime = (m_Params.robotTime.m_robotStartTime - m_Params.robotTime.m_robotEndTime);
    }
    else
    {
        for (int i = 0; i < nSteps; i++)
        {
            m_robotPose.position.x = m_robotPose.position.x + (odometryMsg.linear.x * cos(m_robotAngle) * dt);
            m_robotPose.position.y = m_robotPose.position.y + (odometryMsg.linear.x * sin(m_robotAngle) * dt);
            m_robotAngle = m_robotAngle + odometryMsg.angular.z * dt;
        }
        quat_msg = m_Transformations.EulerToQuat(0.0, 0.0, m_robotAngle);
        m_robotPose.orientation.z = quat_msg.z;
        m_robotPose.orientation.w = quat_msg.w;
    }
    ROS_INFO("Robot  Angle  outside %f",m_robotAngle );
    if(m_reverseMode)
    {
        //convert to 0 to 2pi
        if(m_robotAngle < 0.0)
        {
            m_robotAngle = 6.28 + m_robotAngle;
        }
        else
        {
            m_robotAngle = m_robotAngle;
        }

        if(m_robotAngle > 3.14)
        {
            m_robotAngle = m_robotAngle - 3.14;
        }
        else
        {
            m_robotAngle = m_robotAngle + 3.14;
        }

        if(m_robotAngle < 3.14)
        {
            m_robotAngle = m_robotAngle;
        }
        else
        {
            m_robotAngle = m_robotAngle - 6.28;
        }

        //m_robotAngle = (m_robotAngle < 0.0) ? 3.14 + m_robotAngle : 3.14 - m_robotAngle;
        //m_robotAngle = (-1)*m_robotAngle;
        ROS_ERROR("Robot  Angle  inside %f",m_robotAngle );
    }
    m_Params.robotTime.m_robotEndTime = MathFunc::micros<uint64_t>();
}

/* Checks whether an updated robot pose is received*/
bool TrajectoryController::isNewPoseReceived(geometry_msgs::Pose &robot_pose)
{
    bool changePosition = ((robot_pose.position.x != m_lastRobotPose.position.x) || (robot_pose.position.y != m_lastRobotPose.position.y));
    bool changeOrientation = ((robot_pose.orientation.w != m_lastRobotPose.orientation.w) || (robot_pose.orientation.z != m_lastRobotPose.orientation.z));
    if (changePosition || changeOrientation)
    {
        m_lastRobotPose = robot_pose;
        return true;
    }
    return false;
}

Params::PathData TrajectoryController::interpolatePathData(const Params::PathData& input1, const Params::PathData& input2, const double interpolationRatio)
{
    Params::PathData output;
    geometry_msgs::Quaternion currentOrientation, nextOrientation;
    currentOrientation = input1.m_path.orientation;
    nextOrientation    = input2.m_path.orientation;
    
    // Interpolate orientations and write to output vector
    output.m_path.orientation = m_Transformations.slerp(currentOrientation, nextOrientation, interpolationRatio);

    // Interpolate positions and write to output vector
    output.m_path.position.x = input1.m_path.position.x + interpolationRatio * (input2.m_path.position.x - input1.m_path.position.x);
    output.m_path.position.y = input1.m_path.position.y + interpolationRatio * (input2.m_path.position.y - input1.m_path.position.y);
    output.m_path.position.z = 0;

    output.m_curvatureValue = input1.m_curvatureValue + interpolationRatio * (input2.m_curvatureValue - input1.m_curvatureValue);
    output.m_velLinear = input1.m_velLinear + interpolationRatio * (input2.m_velLinear - input1.m_velLinear);

    return output;
}

void TrajectoryController::resetControllerParameters()
{
    m_goalReached = false;
    m_latchTarget = false;
    m_angleLatch = false;
    m_rotationController = false;
    m_isCheckBackward    = false;
    count               = 0;

    m_Params.robotTime.m_startTime = m_Params.robotTime.m_endTime = MathFunc::micros<uint64_t>();
    ROS_WARN("resetControllerParameters()  Reset");
}

controller_msgs::Debug TrajectoryController::getDebugData()
{
    controller_msgs::Debug debug;
    debug.curvature = m_curvature;
    debug.crossTrackError = m_crossTrackError;
    debug.orientationError = m_orientationError;
    debug.robotVel.linear.x = m_linearVel__m_s;
    debug.robotVel.angular.z = m_angularVel__rad_s;
    debug.controlLaw.first = m_approachControl;
    debug.controlLaw.second = m_cteControl;
    debug.controlLaw.third = m_orientationControl;
    debug.controlLaw.fourth = m_feedForwardControl;
    debug.controlLaw.resultant = m_resultant;
    debug.distanceToGoal = m_distanceToGoal__m;
    return debug;
}

/* Sends the status of the goal reaching to the wrapper */
bool TrajectoryController::isGoalReached()
{
    if (m_goalReached)
    {
        ROS_ERROR("Goal reached");
    }
    return m_goalReached;
}

/* Sends the velocity commands to  the wrapper */
geometry_msgs::Twist TrajectoryController::executeController()
{
    m_Params.robotTime.m_startTime = MathFunc::micros<uint64_t>();
    m_Params.robotTime.m_deltaTime = (m_Params.robotTime.m_startTime - m_Params.robotTime.m_endTime);

    geometry_msgs::Twist commandVelocity;
    commandVelocity.linear.x = longitudinalController();
    commandVelocity.angular.z = lateralController();

    m_Params.robotTime.m_endTime = m_Params.robotTime.m_startTime;
    return commandVelocity;
}

void TrajectoryController::computePathDirection()
{

    geometry_msgs::PoseStamped globalWorld,globalBase;
    geometry_msgs::PoseArray globalArray;
    

    globalArray.poses.clear();
    globalArray.poses.resize(0);
   
   
    if(m_processedPathArray.size() > 0)
    {
        //ROS_INFO("TrajectoryController:: plan size is : %d",path_msg->poses.size());
       
        poseTransform.waitForTransform(m_globalFrame, m_baseFrame, ros::Time(0), ros::Duration(5.0));
        for (unsigned i = 0; i < m_processedPathArray.size(); i++)
        {

            /**
         * @brief Input path frame  transformation into Map frame
        */
            globalWorld.header.frame_id    = m_globalFrame;
            globalWorld.header.stamp       = ros::Time(0);
            globalWorld.pose.position      = m_processedPathArray[i].m_path.position;
            globalWorld.pose.orientation   = m_processedPathArray[i].m_path.orientation;
            //ROS_INFO("x : %f  y : %f ",m_processedPathArray[i].m_path.position.x,m_processedPathArray[i].m_path.position.y);
            try
            {
                poseTransform.transformPose(m_baseFrame, globalWorld, globalBase);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("TrajectoryController:: Received an exception CONTROLLER "
                        "trying to transform a point : %s",
                        ex.what());
            }
            /**
         * @brief Storing the transformed set of path points
        */
            globalArray.poses.push_back(globalBase.pose);
        }

        double distanceToReference = 0.0;
        int referenceCounter = 0;
        for (int i = 0; i < globalArray.poses.size() && (distanceToReference < REVERSE_DISTANCE); i++)
        {
            distanceToReference += MathFunc::euclideanDistance<double>(globalArray.poses[i].position.x, globalArray.poses[i].position.y,globalArray.poses[i + 1].position.x, globalArray.poses[i + 1].position.y);
            referenceCounter++;
        }

        ROS_ERROR("Read value is :%f  AT %d",globalArray.poses[referenceCounter-1].position.x,referenceCounter);
        // if(m_directionSet == false)
        // {  
            if(globalArray.poses[referenceCounter-1].position.x < 0.0)
            {
                ROS_WARN("Direction set as Backward");
                //linearVel = linearVel * (-1);
                // m_robotDirection = -1;
                m_reverseMode    = true;
                // m_setOrientation = 4;   
            }
            else
            {
                ROS_WARN("Direction set as Forward");
                //linearVel = linearVel;
                // m_robotDirection = 1;
                m_reverseMode    = false;
                // m_setOrientation = 1;        
            }
        //    ros::param::set("/move_base/GlobalPlanner/orientation_mode",m_setOrientation);
             
        // }
       
        }
}