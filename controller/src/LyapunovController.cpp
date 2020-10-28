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
    m_Params.robot.m_decelerationDistance__m        = config.m_decelerationDistance__m;


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

    m_enableFirst   = config.enableFirst;
    m_enableSecond  = config.enableSecond;
    m_enableThird   = config.enableThird;
    m_enableFourth  = config.enableFourth;

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
    //m_robotPose  = robot_pose;
    m_goalPose   = goal_pose;
    m_odom       = odom_msg;
    m_pathSize   = m_pathArray.poses.size();
    if(newPlanReceived)
    {
        ROS_INFO("initializeController::Reset counter");
        m_activeCounter = 0;
    }
    //ROS_INFO("Current path size : %d   counter : %d",m_pathArray.poses.size(),m_activeCounter);

    /*Predict robot position*/
    predictRobotPosition(m_odom,robot_pose);

    localization();

    /*Robot angle,inplace angle and final goal pose's angle w.r.t basefootprint*/
    m_inplaceAngle   = m_Transformations.QuatToEuler(m_inplacePose.orientation.z, m_inplacePose.orientation.w);
    m_finalGoalAngle = m_Transformations.QuatToEuler(m_goalPose.orientation.z, m_goalPose.orientation.w);




    m_robotPoseFB    = m_Transformations.getTransformations(m_globalFrame, m_fbFrame, m_robotPose);
    

    setStates();

    // Calculation of Xerror, Yerror and orientationError
    m_sfFrameX = fabs(m_robotPoseFB.position.x);
    m_sfFrameY = m_crossTrackError;
     
}


/**
* @brief  Predicts the robot position based on last positional and velocity informations
*/
// void LyapunovController::predictRobotPosition()
// {
//     m_robotAngle    = m_Transformations.QuatToEuler(m_robotPose.orientation.z,m_robotPose.orientation.w);
// }


/**
* @brief  Sets the different states for executing tracking towards a goal
*/
void LyapunovController::setStates()
{

    bool latchingState   = ((m_distanceToGoal__m) < m_Params.robotTolerance.m_xyGoalTolerance__m) || (m_latchTarget == true);
    m_rotationController = false;
    if (latchingState)
    {
        m_inplaceAngleDifference = Utilities::shortest_angular_distance<double>(m_robotAngle, m_finalGoalAngle);
        m_rotationController     = (fabs(m_inplaceAngleDifference) >= m_Params.robotTolerance.m_yawTolerance__rad);
        m_latchTarget            = true;
        ROS_ERROR("rotationController:  %d  m_latchTarget :%d  m_inplaceAngleDiff: %f : %f",m_rotationController,m_latchTarget,m_inplaceAngleDifference,m_distanceToGoal__m);
    }
    else
    {
        m_inplaceAngleDifference = Utilities::shortest_angular_distance<double>(m_robotAngle, m_inplaceAngle);
        m_rotationController     = isInplaceRotate();
        ROS_WARN("rotationController:  %d  m_latchTarget :%d  m_inplaceAngleDiff: %f : %f",m_rotationController,m_latchTarget,m_inplaceAngleDifference,m_distanceToGoal__m);
    }
    //ROS_WARN("In place angle : %f",m_inplaceAngleDifference);
    
    m_goalReached = m_latchTarget && (!m_rotationController);
}

/**
* @brief  Checks if a inplace rotation is required at the moment 
* @return true if inplace rotation is needed else false
*/
bool LyapunovController::isInplaceRotate()
{
    /*Ensures that once robot gets within the entry tolerance (angle_ > m_inplaceTolerance__rad) ,till the exit condition  (angle_ < m_yawTolerance__rad)
     is not satisfied,it remains in the state of Rotation controller*/
    bool entryCondition = (fabs(m_inplaceAngleDifference) > m_Params.robotTolerance.m_inplaceTolerance__rad) ? true : false;

    if (entryCondition || m_angleLatch)
    {
        m_angleLatch = true;
        bool exitCondition = (fabs(m_inplaceAngleDifference) < m_Params.robotTolerance.m_yawTolerance__rad) ? true : false;

        if (exitCondition)
        {
            m_angleLatch = false;
        }
        return (!exitCondition);
    }

    return entryCondition;    
}


/**
* @brief  Sends the status of the goal reached to the wrapper
* @return true if goal is reached,else false
*/
bool LyapunovController::isGoalReached()
{
    if (m_goalReached)
    {
        ROS_ERROR("Robot reached the goal");
    }
    return m_goalReached;
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

    double xyProjectionLength = 100.0;  /*Initializing to a bigger value,so as by the end of first iteration, the min value will be first element*/
    double projectionFactor = 0.0;

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

    /* Publish the TF and broadcast the error frame  (feeddback frame)*/
    if(m_Transformations.isValidQuaternion(m_sfRefPoint))
    {
        m_Transformations.sendTransformations(m_globalFrame, m_fbFrame, m_errorRefPoint);
    }
    else
    {
        ROS_ERROR("Transfrom error captured : m_fbFrame");
    }

    
    


    double errorRefPointOrientation    = m_Transformations.QuatToEuler(m_errorRefPoint.orientation.z,m_errorRefPoint.orientation.w);

    /*Derived the control parameters for the control law: cross track error and orientation error*/
    m_crossTrackError           = xyProjectionLength;
    m_orientationError          = Utilities::shortest_angular_distance(m_robotAngle,errorRefPointOrientation);
    ROS_WARN("Robot angle : %f    cross track : %f      orientation error : %f errorRefPointOrientation : %f",m_robotAngle,m_crossTrackError,m_orientationError,errorRefPointOrientation);
    ROS_WARN("Orientation : %f    %f",m_errorRefPoint.orientation.z,m_errorRefPoint.orientation.w);
    
    /*Calculate the distance remaining to reach the goal*/
    m_distanceToGoal__m         = Utilities::euclideanDistance(m_robotPose.position.x,m_robotPose.position.y,m_goalPose.position.x,m_goalPose.position.y);

    /*=========================================================================================*/
    /*= Calculation of the feedforward frame at a distance of lookahead from projection point =*/
    /*=========================================================================================*/

    /*For calculation of the control law, define a new coordinate frame at a distance "m_lookAhead" from the error frame*/

    double velocityLookAhead = m_Params.controlLaw.m_lookAhead;
    double distanceFromErrorFrame = 0.0;

    /*@TODO: Recheck initialize value*/
    m_activeCounterPred = m_activeCounter; //max velLa
    while ((distanceFromErrorFrame < velocityLookAhead)  && (m_activeCounterPred < m_pathSize - 1))
    {
        distanceFromErrorFrame += Utilities::euclideanDistance<double>(m_pathArray.poses[m_activeCounterPred].position.x, m_pathArray.poses[m_activeCounterPred].position.y, m_pathArray.poses[m_activeCounterPred + 1].position.x, m_pathArray.poses[m_activeCounterPred + 1].position.y);
        ROS_INFO("distanceFromErrorFrame   : %f",distanceFromErrorFrame);
        if (distanceFromErrorFrame < (m_Params.controlLaw.m_inplaceRadius))
        {
            m_counterInplace = m_activeCounterPred;
        }
        if (distanceFromErrorFrame < velocityLookAhead )
        {
            //distanceLast = distanceFromErrorFrame;
            m_activeCounterPred++;
        }
    }

    /*SF frame pose and In-place pose*/
    /*@TODO: Array position re-check*/

    m_sfRefPoint   = m_pathArray.poses[m_activeCounterPred];
    m_inplacePose  = m_pathArray.poses[m_counterInplace];

    /* Publish the TF and broadcast the new control frame  (SF frame)*/
    if(m_Transformations.isValidQuaternion(m_sfRefPoint))
    {
        m_Transformations.sendTransformations(m_globalFrame, m_sfFrame, m_sfRefPoint);
    }
    else
    {
        ROS_ERROR("Transfrom error captured : m_sfRefPoint");
    }
    
    
    if(m_activeCounterPred > 0)
    {
        m_curvature    = computeCurvature(m_pathArray.poses[m_activeCounterPred],m_pathArray.poses[m_activeCounterPred - 1]);
    }
    else
    {
        m_curvature = 0.0;
    }
    
    

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

double LyapunovController::computeCurvature(const geometry_msgs::Pose &sfRefPoint, const geometry_msgs::Pose &sfPreRefPoint)
{

    double sfRefOrientation = m_Transformations.QuatToEuler(sfRefPoint.orientation.z, sfRefPoint.orientation.w);
    double sfPreRefOrientation = m_Transformations.QuatToEuler(sfPreRefPoint.orientation.z, sfPreRefPoint.orientation.w);
    double deltaTheta = Utilities::shortest_angular_distance<double>(sfRefOrientation, sfPreRefOrientation);
    double deltaS = Utilities::euclideanDistance(sfRefPoint.position.x, sfRefPoint.position.y, sfPreRefPoint.position.x, sfPreRefPoint.position.y);

    if (!std::isfinite(deltaTheta / deltaS))
    {
        ROS_ERROR("The curvature is infinite : %f    %f",deltaTheta,deltaS);
        ROS_ERROR("Orientations : %f  %f  m_activeCounterPred : %d  m_activeCounter :%d  m_pathSize :%d ",sfRefOrientation,sfPreRefOrientation,m_activeCounterPred,m_activeCounter,m_pathSize);
    }
    /*@TODO: Unknown reason for -negative sign*/
    return (-1*deltaTheta / deltaS);
}

/**
* @brief  Lateral controller computation and applies limit on it
* @returns Angular velocity in radians/sec
*/
double LyapunovController::lateralController()
{
    if(m_rotationController)
    {
        m_angularVel__rad_s = rotationController();
    }
    else
    {
        m_angularVel__rad_s = controlLaw();   
    }
    
    angularVelocityLimit(m_angularVel__rad_s);
    return m_angularVel__rad_s;
}

/**
* @brief  Longitudinal controller computation and applies limit on it
* @returns Linear velocity in meters/sec
*/
double LyapunovController::longitudinalController()
{
    if(m_rotationController)
    {
        m_linearVel__m_s = 0.0;
    }
    else
    {
       m_linearVel__m_s = computeLinearVelocity(); 
    }
    
    
    linearVelocityLimit(m_linearVel__m_s);
    //ROS_INFO("Post editiing : %f",m_linearVel__m_s);
    return m_linearVel__m_s;
}

/**
* @brief  Linear velocity computation
* @returns Linear velocity in meters/sec
*/
double LyapunovController::computeLinearVelocity()
{
    double linearVelocity, velocityDistance, velocityAcceleration;

    /*Default in case if m_distanceToGoal > m_decelerationDistance__m*/
    velocityDistance = m_Params.robot.m_maxLinearVel__m_s;

    if (m_distanceToGoal__m <= m_Params.robot.m_decelerationDistance__m)
    {
        double velocityDistanceSquare = std::max(pow(m_odom.linear.x, 2) - 2 * (m_Params.robot.m_decelerationDistance__m - m_distanceToGoal__m) * m_Params.robot.m_linearDeccLimit__m_s2, pow(m_Params.robot.m_minLinearVel__m_s, 2));
        velocityDistance = std::sqrt(velocityDistanceSquare);
        ROS_WARN("computeLinearVelocity::Deccelerating at :%f  with :   %f", m_distanceToGoal__m, velocityDistance);
    }

    if ((!std::isfinite(m_curvature)) || std::isnan(m_curvature))
    {
        velocityAcceleration = 0;
        ROS_ERROR("computeLinearVelocity:: nan m_curvature");
        return 0.0;
    }
    else
    {
        velocityAcceleration = std::fabs(sqrt(m_Params.robot.m_lateralAcceleration__m_s2 / fabs(m_curvature)));
        ROS_WARN("computeLinearVelocity::velocityAcceleration : %f  velocityDistance :%f  m_distanceToGoal__m : %f", velocityAcceleration, velocityDistance,m_distanceToGoal__m);
    }
    linearVelocity = std::max(std::min(velocityAcceleration, velocityDistance), m_Params.robot.m_minLinearVel__m_s);  
    return linearVelocity;  
}

double LyapunovController::rotationController()
{
    ROS_INFO("Rotation controller active....");
    double angularVelocity = m_Params.controlLaw.m_Kp * m_inplaceAngleDifference ;

    if (fabs(angularVelocity) > fabs(m_Params.robot.m_maxInplaceAngularVel__rad_s))
    {
        angularVelocity = (m_Params.robot.m_maxInplaceAngularVel__rad_s) * Utilities::sign(angularVelocity);
    }
    return angularVelocity;
}

double LyapunovController::controlLaw()
{
    if ((!std::isfinite(m_curvature)) || std::isnan(m_curvature))
    {
        ROS_ERROR("LyapunovController:: Control law fail captured");
        return 0.0;
    }
    computeDerivatives();

    double thetaDifference  = Utilities::shortest_angular_distance<double>(m_orientationError, m_delta);
    double sinThetaTerm     = (sin(m_orientationError) - sin(m_delta)) / (thetaDifference);

    if (!std::isfinite(sinThetaTerm) || std::isnan(sinThetaTerm))
    {
        sinThetaTerm = 0.0;
    }
    ROS_INFO("m_orientationError: %f   m_delta: %f",m_orientationError,m_delta);

    /*Defining the four different control parameters as per the control law*/
    m_approachControl       = m_derivativeDelta * m_enableFirst;
    m_crossTrackControl     = (-1 * m_Params.controlLaw.m_gamma * m_sfFrameY * m_linearVel__m_s * sinThetaTerm) * m_enableSecond;
    m_orientationControl    = (-1 * m_Params.controlLaw.m_k2Gain * (m_orientationError - m_delta)) * m_enableThird;
    m_feedForwardControl    = m_curvature * m_derivativeS * m_enableFourth;

    ROS_INFO("App : %f  cte : %f  orie : %f  ffd : %f",m_approachControl,m_crossTrackControl,m_orientationControl,m_feedForwardControl);
    double resultantOmega   = m_approachControl +  m_crossTrackControl + m_orientationControl + m_feedForwardControl;
    return resultantOmega;
}

/**
* @brief  Helper functions for control law 
*/
void LyapunovController::computeDerivatives()
{
    m_derivativeS       = (m_linearVel__m_s * cos(m_orientationError) + m_Params.controlLaw.m_k1Gain * m_sfFrameX);
    m_derivativeY       = (-1 * m_curvature * m_derivativeS * m_sfFrameX) + (m_linearVel__m_s * sin(m_orientationError));

    m_delta             = (-1 * m_Params.controlLaw.m_thetaA * tanh(m_Params.controlLaw.m_kDeltaGain * m_sfFrameY));
    m_derivativeDelta   = (-1 * m_Params.controlLaw.m_thetaA * m_Params.controlLaw.m_kDeltaGain * m_derivativeY * (1 - (tanh(m_Params.controlLaw.m_kDeltaGain * m_sfFrameY) * tanh(m_Params.controlLaw.m_kDeltaGain * m_sfFrameY))));
   
}





/**
* @brief  Limits the linear velocities
*/
void LyapunovController::linearVelocityLimit(double &linearVelocity)
{
    /**
    * @brief Handling nan and inf linear velocities
    */
    //ROS_ERROR("Input : %f ",linearVelocity);
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




/**
* @brief  Predicts the robot position whenever the updated robot pose data is not received.
* This is based on robot's prev velocity and position
*/
void LyapunovController::predictRobotPosition(geometry_msgs::Twist &odometryMsg, geometry_msgs::Pose &robot_pose)
{
    geometry_msgs::Quaternion quat_msg;

    m_Params.robotTime.m_robotStartTime = Utilities::micros<uint64_t>();
    m_Params.robotTime.m_robotDeltaTime += (m_Params.robotTime.m_robotStartTime - m_Params.robotTime.m_robotEndTime);

    float dt = 0.005; // Step size of the integration in secs, define as macro
    int nSteps = (m_Params.robotTime.m_robotDeltaTime * MICRO_SECONDS) / dt;
    m_robotPose = robot_pose;
    m_robotAngle = m_Transformations.QuatToEuler(m_robotPose.orientation.z, m_robotPose.orientation.w);
    
    bool newRobotDatareceived = isNewPoseReceived(m_robotPose);

    if (newRobotDatareceived || ((m_Params.robotTime.m_robotDeltaTime * MICRO_SECONDS) > (1.0 / ROBOT_POSE_FREQUENCY)))
    {
        m_Params.robotTime.m_robotStartTime = m_Params.robotTime.m_robotEndTime = Utilities::micros<uint64_t>();
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
   
    m_Params.robotTime.m_robotEndTime = Utilities::micros<uint64_t>();
}

/**
* @brief  Checks whether an updated robot pose is received
*/
bool LyapunovController::isNewPoseReceived(geometry_msgs::Pose &robot_pose)
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

/**
* @brief  Sends the velocity commands to  the interface class
*/
geometry_msgs::Twist LyapunovController::sendCommandVelocity()
{
    m_Params.robotTime.m_startTime = Utilities::micros<uint64_t>();
    m_Params.robotTime.m_deltaTime = (m_Params.robotTime.m_startTime - m_Params.robotTime.m_endTime);

    geometry_msgs::Twist commandVelocity;
    commandVelocity.linear.x  = longitudinalController();
    commandVelocity.angular.z = lateralController();

    ROS_WARN("Calculated linear : %f Angular : %f",commandVelocity.linear.x,commandVelocity.angular.z);
    m_Params.robotTime.m_endTime = m_Params.robotTime.m_startTime;
    return commandVelocity;
}




/**
* @brief  Resets the parameters for the LyapunovController class
*/
void LyapunovController::resetControllerParameters()
{
    m_goalReached         = false;
    m_latchTarget         = false;
    m_angleLatch          = false;
    m_rotationController  = false;

    m_Params.robotTime.m_startTime = m_Params.robotTime.m_endTime = Utilities::micros<uint64_t>();
    ROS_WARN("resetControllerParameters()  Reset");
}