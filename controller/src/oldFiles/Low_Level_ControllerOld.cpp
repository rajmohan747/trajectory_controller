#include "Low_Level_Controller.h"
#include <math.h>
#include <ros/console.h>
#include <boost/thread.hpp>
#include <pthread.h>
#include <mutex>
#include <exception>
#include <utility> 

/*  Constructor for the LowLevelController*/
LowLevelController::LowLevelController():m_setup_(false)
{ 
    m_angularT1 = m_angularT2 =  MathFunc::micros<uint64_t>();  
    m_linearT1  = m_linearT2  =  MathFunc::micros<uint64_t>();
    m_robotT1   = m_robotT2   =  MathFunc::micros<uint64_t>();

    ROS_INFO_ONCE("LowLevelController::  Constructor initialized : %f %f %f %f",m_k1Gain,m_kDeltaGain,m_gamma,m_thetaA);
    
    /*  Reconfigure callback  */
    f = boost::bind(&LowLevelController::reconfigureCallback, this, _1, _2);
    server.setCallback(f);
}

    
/*  Destructor for the LowLevelController */
LowLevelController::~LowLevelController()
{

}

/* Reconfigure callback   */
void LowLevelController::reconfigureCallback(controller::controllerConfig& config, uint32_t level)
{
    ROS_INFO_ONCE("In reconfiguration call back");
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    /**
     * @brief The first time we're called, we just want to make sure we have the original configuration
     * if restore defaults is set on the parameter server, prevent looping
    */
    if(config.restore_defaults) 
    {
      ROS_WARN_ONCE("LowLevelController:: Re-setting default values");
      config = default_config_;
      config.restore_defaults = false;
    }

    /*Robot parameters */
    m_maxLinearVel__m_s          =  config.max_vel;
    m_minLinearVel__m_s          =  config.min_vel;
    m_maxAngularVel__rad_s       =  config.max_ang;
    m_linearAccLimit__m_s2       =  config.acc_limit;
    m_linearDeccLimit__m_s2      =  config.decc_limit;
    m_angularAccLimit__rad_s2    =  config.ang_acc_limit;   
    m_lateralAcceleration__m_s2  =  config.lateral_acceleration;
    
    /*Tolerance parameters */
    m_xyGoalTolerance__m        =  config.xy_tolerance;
    m_yawTolerance__rad         =  config.angle_tolerance;  
    m_inplaceTolerance__rad     =  config.inPlace_tolerance;  

    /* Control law tuning parameters */
    m_k1Gain             =  config.k1;
    m_k2Gain             =  config.k2;
    m_kDeltaGain         =  config.k_delta;
    m_gamma              =  config.Gamma;
    m_thetaA             =  config.theta_a; 
    m_lookAhead          =  config.laRadius;
    m_inplaceRadius      =  config.inplaceRadius; 

    if(!m_setup_)
    {
      last_config_ = config;
      default_config_ = config;
      m_setup_ = true;
      return;
    }
    last_config_ = config;
}

   
 /*initialize the member varaibles obtained    */
void LowLevelController::initialize(geometry_msgs::PoseArray& array_msg,geometry_msgs::Pose& robot_pose,geometry_msgs::Pose& last_goal,geometry_msgs::Twist& odom_msg,bool newPlanReceived)
{
    /* Resetting the counters for the projections ,once a new plan is received  */
    if((newPlanReceived) && (!m_rotationController))
    {
        m_counterInplace    =  0;
        m_counterSF  = 0;
        m_activeCounter     = 0;
    }

    /* As long as the robot is in the Rotation Controller,new  plan won't be accepted    */
    if(!m_rotationController)
    {
        // set the path array size to 0 and clear it.
        m_pathArray.poses.clear();
        m_pathArray.poses.resize(0);

        m_pathArray    = array_msg;                  
        m_lastPose     = last_goal;
        m_msgSize      =  array_msg.poses.size();  
    }

    // Get the distance of robot from goal
    m_distanceToGoal__m      =  MathFunc::euclideanDistance<double>(m_robotPose.position.x,m_robotPose.position.y ,m_lastPose.position.x, m_lastPose.position.y); 
    

    bool newRobotDatareceived = isNewPoseReceived(robot_pose); 
    m_robotT1            = MathFunc::micros<uint64_t>();
    m_robotDeltaTime    += (m_robotT1 - m_robotT2);

    if(newRobotDatareceived || ((m_robotDeltaTime*MICRO_SECONDS) > (1.0/ROBOT_POSE_FREQUENCY))) 
    {
        m_robotT1         = m_robotT2   = MathFunc::micros<uint64_t>();
        m_robotDeltaTime  = (m_robotT1 - m_robotT2);
        m_robotPose       = robot_pose;
    }
    else
    {
        m_robotPose    = predictRobotPosition(odom_msg);
    }
    
    pathPostProcessing();

    preProcessing();

    velocityProfiling();

    if(m_rotationController)
    {
        m_velProfile = 0.0;
    }
    else
    {
        m_velProfile    = m_processedPathArray[m_activeCounter].m_velLinear;
    }
    m_robotT2         = MathFunc::micros<uint64_t>();
}



/* pre process the parameters required for the controller execution */
void LowLevelController::pathPostProcessing()
{
    // clear the pose array
    m_processedPathArray.clear();
    m_processedPathArray.resize(0);

    // define a local variable pathPoint of structtype pathData
    LowLevelController::pathData pathPoint;
    
    unsigned counter;
    // push back the data to vector
    for(unsigned i = 0; (i < m_msgSize - 1) ; i++)
    {
        pathPoint.m_path.position.x     = m_pathArray.poses[i].position.x;
        pathPoint.m_path.position.y     = m_pathArray.poses[i].position.y;
        pathPoint.m_path.orientation.z  = m_pathArray.poses[i].orientation.z;
        pathPoint.m_path.orientation.w  = m_pathArray.poses[i].orientation.w;
        pathPoint.m_curvatureValue      = m_Transformations.calculateCurvature(m_pathArray.poses[i],m_pathArray.poses[i+1]);
        if (i==0) // distance at first point is zero
        {
            pathPoint.m_distFromInital     += 0; 
            pathPoint.m_velLinear           = 0; // set the initial velocity at path point to odom_vel
        }
        else
        {
            pathPoint.m_distFromInital     += MathFunc::euclideanDistance<double>(m_pathArray.poses[i-1].position.x,m_pathArray.poses[i-1].position.y,m_pathArray.poses[i].position.x,m_pathArray.poses[i].position.y); 
            pathPoint.m_velLinear           = computeLinearVelocity(pathPoint.m_curvatureValue);
        }
        m_processedPathArray.push_back(pathPoint);
        counter = i;
    } 
    counter++;    

    // set the values at last point
    pathPoint.m_path.position.x     = m_pathArray.poses[counter].position.x;
    pathPoint.m_path.position.y     = m_pathArray.poses[counter].position.y;
    pathPoint.m_path.orientation.z  = m_pathArray.poses[counter].orientation.z;
    pathPoint.m_path.orientation.w  = m_pathArray.poses[counter].orientation.w;
    pathPoint.m_curvatureValue      = m_processedPathArray[counter-1].m_curvatureValue; //the curvature can be set to last value
    pathPoint.m_distFromInital     += MathFunc::euclideanDistance<double>(m_pathArray.poses[counter-1].position.x,m_pathArray.poses[counter-1].position.y,m_pathArray.poses[counter].position.x,m_pathArray.poses[counter].position.y); 
    pathPoint.m_velLinear           = computeLinearVelocity(pathPoint.m_curvatureValue); // define here the end velocity if required
    m_processedPathArray.push_back(pathPoint);
}

/*  pre process the parameters required for the controller execution */

void LowLevelController::preProcessing()
{
    m_sfRefPoint        = localisation();
    m_inplacePoint      = m_processedPathArray[m_counterInplace].m_path;
    m_curvature         = m_processedPathArray[m_counterSF-1].m_curvatureValue;

    if((!std::isfinite(m_curvature)) || std::isnan(m_curvature))
    {
        ROS_ERROR("Nan curvature detected");
        m_curvature  =  m_lastValidCurvature;
    }    
    else
    {
        m_lastValidCurvature = m_curvature;
    }
    
    // Publish the SF frame
    m_Transformations.sendTransformations(m_globalFrame, m_targetFrame, m_sfRefPoint);
    
    /*gets all the transformations required*/
    getTransformations();
    
    /* Set all angles*/
    setAngles();
    
    setStates();
}

void LowLevelController::setAngles()
{
        /**
        * @brief Orientation of real,virtual robots w.r.t global frame and the difference between them
        */
        m_thetaM                 = m_Transformations.QuatToEuler(m_robotPose.orientation.z,m_robotPose.orientation.w);
        m_thetaC                 = m_Transformations.QuatToEuler(m_sfRefPoint.orientation.z,m_sfRefPoint.orientation.w);
        m_theta                  = MathFunc::shortest_angular_distance<double>(m_thetaM,m_thetaC);

        /*TODO: Change position*/
        m_sfFrameX               = fabs(m_robotPoseSF_.position.x);
        m_sfFrameY               = m_robotPoseSF_.position.y;//*MathFunc::sign<double>(m_curvature);

        /*Robot angle,inplace angle and final goal pose's angle w.r.t basefootprint*/
        m_robotAngle       = m_Transformations.QuatToEuler(m_robotPoseBase_.orientation.z, m_robotPoseBase_.orientation.w);   
        m_inplaceAngle     = m_Transformations.QuatToEuler(m_inplaceBase_.orientation.z, m_inplaceBase_.orientation.w);
        m_finalGoalAngle   = m_Transformations.QuatToEuler(m_finalGoalBase_.orientation.z, m_finalGoalBase_.orientation.w); 
        
        /**
        * @brief Target angle tolerance to be set for the rotation controller.
        * It will be used only if rotation controller is performing action
        * @return:either m_inplaceTolerance__rad or m_yawTolerance__rad
        */
        m_angleTolerance   = inplaceAngleLimits();
}


double LowLevelController::inplaceAngleLimits()
{
    double angle_limit;
    if(m_latching)
    {
        m_inplaceAngleDiff     = MathFunc::shortest_angular_distance<double>(m_robotAngle, m_finalGoalAngle);  /*Difference in angle at goal */
        angle_limit            = m_yawTolerance__rad;
    }
    else  
    {
        m_inplaceAngleDiff     =  MathFunc::shortest_angular_distance<double>(m_robotAngle,m_inplaceAngle); 
        angle_limit            =  m_inplaceTolerance__rad;
    }
    return angle_limit;
}


void LowLevelController::setStates()
{
    m_rotationController      = false;

    m_latching                = (m_distanceToGoal__m < m_xyGoalTolerance__m) || (m_latchTarget == true); //|| (m_distanceMiss == true);  
    if(m_latching)
    {
        m_rotationController  = (fabs(m_inplaceAngleDiff) >= m_yawTolerance__rad);
        m_latchTarget         = true;
    }
    else
    {
        m_rotationController = rotateInplaceCheck(); 
    }
    m_goalReached           =  m_latching && (!m_rotationController);

}

/*  Will be doing the velocity profiling for the velocity computed in the m_processedPathArray vector*/
void LowLevelController::velocityProfiling()
{
    double deltaDistance,acceleration;

    /*Iteration Vector*/
    vector<double> velocityIteration; 

    // set the elements to initial value
    for (int i=0; i < m_processedPathArray.size(); i++)
    {
        velocityIteration.push_back(m_processedPathArray[i].m_velLinear);
    }

    for(int i=1; i < (m_processedPathArray.size()-1);i++)
    {
        deltaDistance =   m_processedPathArray[i].m_distFromInital - m_processedPathArray[i-1].m_distFromInital;
        acceleration  =  (pow(velocityIteration[i],2) - pow(velocityIteration[i-1],2))/(2*deltaDistance) ;
        if(acceleration > m_linearAccLimit__m_s2)
        {
            acceleration    = m_linearAccLimit__m_s2;
        }
        velocityIteration[i] = std::min(sqrt(pow(m_processedPathArray[i-1].m_velLinear,2) + 2*acceleration*deltaDistance), velocityIteration[i]);
    }

    for(int i=(m_processedPathArray.size()-1); i > 0; i--)
    {
        deltaDistance       = (m_processedPathArray[i].m_distFromInital - m_processedPathArray[i-1].m_distFromInital);
        acceleration        = (pow(velocityIteration[i-1],2) - pow(velocityIteration[i],2))/(2*deltaDistance);
        if(acceleration > m_linearDeccLimit__m_s2)
        {
            acceleration    = m_linearDeccLimit__m_s2;
        }
        velocityIteration[i-1]       = std::min(sqrt(pow(velocityIteration[i],2) + 2*acceleration*deltaDistance),velocityIteration[i-1]);
    }

    for(int i = 0;i < velocityIteration.size(); i++ )
    {
        m_processedPathArray[i].m_velLinear  = velocityIteration[i];
    }
}   


double LowLevelController::longitudinalController()
{
    ROS_INFO_ONCE("LowLevelController:: Longitudinal Controller");
    m_linearVel__m_s    = computeLinearVelocity(m_curvature);
    m_linearVel__m_s    = linearLimit(m_linearVel__m_s);

   // m_linearVel__m_s    = m_velProfile;

    return m_linearVel__m_s;
}

double LowLevelController::computeLinearVelocity(double curvature)
{
    double velocity =  std::abs(sqrt(m_lateralAcceleration__m_s2 / abs(curvature)));  
    if (velocity > m_maxLinearVel__m_s)
    {
        velocity = m_maxLinearVel__m_s;
    }
    return velocity;    
}

double LowLevelController::lateralController()
{
    ROS_INFO_ONCE("LowLevelController:: Lateral Controller");
    if (m_rotationController)
    {
        /**
         * @brief Rotation Controller
        */
        rotationController(m_angleTolerance);
    }
    else
    {
        /**
         * @brief Trajectory tracking controller
        */
         m_angularVel__rad_s = controlLaw();

    }
    m_angularVel__rad_s         = angularLimit(m_angularVel__rad_s);
    return m_angularVel__rad_s;
}

bool LowLevelController::rotateInplaceCheck()
{
    double path_distance;
    bool rotate_     = false;
    double angle_    =  MathFunc::shortest_angular_distance<double>(m_robotAngle,m_inplaceAngle);
    
    if (m_pathArray.poses.size() > INPLACE_PATH_SIZE)
    {   
        rotate_ = true;
        for (int i = 0; ((i < INPLACE_PATH_SIZE) && (rotate_ == true)); i++)
        {
            path_distance =  MathFunc::euclideanDistance<double>(m_pathArray.poses[i].position.x, m_pathArray.poses[i].position.y,m_pathArray.poses[(i+1)].position.x, m_pathArray.poses[(i+1)].position.y);

            rotate_ = (path_distance < INPLACE_PATH_LENGTH) ? true : false;
            //index = i;
        }
       
    }
    
    bool angle_status = (fabs(angle_) > m_inplaceTolerance__rad) ? true : false;
    return angle_status && rotate_;
}

void LowLevelController::rotationController(double angle_limit)
{
    /**
     * @brief Setting linear velocity to 0.0 as its pure inplace rotation
     * */
    m_linearVel__m_s    = 0.0;
    //ROS_INFO("m_inplaceAngleDiff: %f  angle_limit  : %f ",m_inplaceAngleDiff,angle_limit);
    if((m_inplaceAngleDiff) > angle_limit) //&&  ((m_inplaceAngleDiff)  < M_PI))
    {
        /*TO DO: Implement a PID for the angular correction*/
        m_angularVel__rad_s = 0.3;
    }
    else if ((m_inplaceAngleDiff) < -angle_limit) //&&  ((m_inplaceAngleDiff)  > M_PI))
    {
        m_angularVel__rad_s = -0.3;
    }

}


/* @brief Control law helper functions     */
void LowLevelController::getDerivatives()
{
    m_derivS = (m_linearVel__m_s* cos(m_theta) + m_k1Gain*m_sfFrameX);
    m_derivY = (-1*m_curvature*m_derivS*m_sfFrameX) + (m_linearVel__m_s*sin(m_theta));
    m_derivDelta = ( -1*m_thetaA*m_kDeltaGain*m_derivY*(1-(tanh(m_kDeltaGain*m_sfFrameY)*tanh(m_kDeltaGain*m_sfFrameY))));
    m_delta = (-1*m_thetaA*tanh(m_kDeltaGain*m_sfFrameY));
}

/* control law to find the corresponding angular velocity    */
double LowLevelController::controlLaw()
{
    if ((!std::isfinite(m_curvature)) || std::isnan(m_curvature)) 
    {
        ROS_ERROR("LowLevelController:: Control law fail captured");
        return 0.0;
    }
    getDerivatives();

    double theta_diff  =  MathFunc::shortest_angular_distance<double>(m_theta,m_delta);
    double theta_term  = (sin(m_theta)-sin(m_delta))/(theta_diff);


    m_approachControl      = m_derivDelta;
    m_cteControl           = (-1*m_gamma*m_sfFrameY*m_linearVel__m_s*theta_term);
    m_orientationControl   = (-1*m_k2Gain*(m_theta - m_delta));
    m_feedForwardControl   = m_curvature*m_derivS;


    if(m_theta!= m_delta)
    {
        m_resultant = (m_approachControl + m_cteControl + m_orientationControl + m_feedForwardControl);
        return (m_approachControl + m_cteControl + m_orientationControl + m_feedForwardControl);
    }
    else
    {
        m_resultant = (m_approachControl +  m_orientationControl + m_feedForwardControl);
        return (m_approachControl  + m_orientationControl + m_feedForwardControl);
    }
}

double LowLevelController::linearLimit(double linearVel)
{
    m_linearT1          =  MathFunc::micros<uint64_t>();
    m_linearDeltaTime    = m_linearT1 - m_linearT2;

    if (!std::isfinite(linearVel) || std::isnan(linearVel))
    {
       ROS_ERROR("LowLevelController:: v error........!!!");
       return 0.0;
    }
    /**
     * @brief ensuring linear velocity complies to v= u +/- acc*delta_time
    */
    
    if(m_rotationController == false)
    { 
        if (linearVel > (m_lastLinearVel + (m_linearAccLimit__m_s2*m_linearDeltaTime*MICRO_SECONDS)))
        {
            linearVel = (m_lastLinearVel + (m_linearAccLimit__m_s2*m_linearDeltaTime*MICRO_SECONDS));
        }
        else if  (linearVel < (m_lastLinearVel - (m_linearDeccLimit__m_s2*m_linearDeltaTime*MICRO_SECONDS)))
        {
            linearVel = (m_lastLinearVel - (m_linearDeccLimit__m_s2*m_linearDeltaTime*MICRO_SECONDS));
        }
        else
        {
            linearVel = linearVel;
        }

        /**
         * @brief Clipping the linear velocity limit
        */
        
        if(linearVel > m_maxLinearVel__m_s)
        {
            linearVel = m_maxLinearVel__m_s;
        }
        else if(linearVel < m_minLinearVel__m_s) // ra-1&& (m_rotate == false))
        {
            linearVel = m_minLinearVel__m_s;
        }
    }
    else
    {        
        linearVel = 0.0;
    }
    m_lastLinearVel = linearVel;
    m_linearT2      = m_linearT1;
    return linearVel;
}

/* Bounding the  angular velocities */
double LowLevelController::angularLimit(double angularVel)
{
    m_angularT1          =  MathFunc::micros<uint64_t>();
    m_angularDeltaTime    = m_angularT1 - m_angularT2;

    if (!std::isfinite(angularVel) || std::isnan(angularVel))
    {
       ROS_ERROR("LowLevelController:: w error........!!!");
       return 0.0;
    }

    /**
     * @brief ensuring angular velocity complies to w = w_last +/- alpha *delta_time
    */
    if (m_rotationController == false)
    {
        if (angularVel > (m_lastAngularVel + (m_angularAccLimit__rad_s2*m_angularDeltaTime*MICRO_SECONDS)))
        {
        angularVel = (m_lastAngularVel + (m_angularAccLimit__rad_s2*m_angularDeltaTime*MICRO_SECONDS));
        }
        else if  (angularVel < (m_lastAngularVel - (m_angularAccLimit__rad_s2*m_angularDeltaTime*MICRO_SECONDS)))
        {
        angularVel = (m_lastAngularVel - (m_angularAccLimit__rad_s2*m_angularDeltaTime*MICRO_SECONDS));
        }
        else
        {
        angularVel = angularVel;
        }

        /**
         * @brief Clipping the angular velocity limit
        */
        if(angularVel > m_maxAngularVel__rad_s)
        {
        angularVel = m_maxAngularVel__rad_s;
        }
        else if(angularVel < -m_maxAngularVel__rad_s)
        {
        angularVel = -m_maxAngularVel__rad_s;
        }
    }
    else
    {
        angularVel = angularVel;
    }
    
    /**
     * @brief To ensure robot doesn't fall down to extreme low angular velocities while correction
    */

    m_lastAngularVel = angularVel;
    m_angularT2     = m_angularT1;

    return angularVel;
}


/* This function localizes the current position of the robot on the trajectory
* and writes the reference point for error calculation and control calculation to member variables */
geometry_msgs::Pose LowLevelController::localisation()
{
    // Initilize the activeCounter from last counter (activeCounter = 0 in first iteration)
    // Initialization of the search
    double xPathSegment      =  m_pathArray.poses[m_activeCounter+1].position.x - m_pathArray.poses[m_activeCounter].position.x;
    double yPathSegment      =  m_pathArray.poses[m_activeCounter+1].position.y - m_pathArray.poses[m_activeCounter].position.y;
    double xylengthPathSegment  =    xPathSegment*xPathSegment + yPathSegment*yPathSegment;

    double xPosPathSegment = m_robotPose.position.x-m_pathArray.poses[m_activeCounter].position.x;
    double yPosPathSegment = m_robotPose.position.y-m_pathArray.poses[m_activeCounter].position.y;

    double xyProjection;
    if (xylengthPathSegment <= 0)
    {
        xyProjection = 0.0;
    }
    else 
    {
        xyProjection = (xPosPathSegment*xPathSegment + yPosPathSegment*yPathSegment)/xylengthPathSegment;
    }

    if (xyProjection <=0)
    {
        xyProjection = 0.0;
    }
    else if (xyProjection>=1)
    {
        xyProjection = 1;
    }   

    double xProjection      =   xPosPathSegment - xyProjection*xPathSegment;
    double yProjection      =   yPosPathSegment - xyProjection*yPathSegment;
    double xyProjLength     =   xProjection*xProjection+yProjection*yProjection; // Taking sqrt since only required for comparison

    // Start the search from next counter and continue till end of the path to find the best match
    // The correct path idx is stored in m_activeCounter
    int i = m_activeCounter+1; 
    while(i < m_msgSize -1)
    {
        double xPathSegmentNext          =  m_pathArray.poses[i+1].position.x - m_pathArray.poses[i].position.x;
        double yPathSegmentNext          =  m_pathArray.poses[i+1].position.y - m_pathArray.poses[i].position.y;
        double xylengthPathSegmentNext   =  xPathSegmentNext*xPathSegmentNext+ yPathSegmentNext*yPathSegmentNext;

        double xPosPathSegmentNext  = m_robotPose.position.x - m_pathArray.poses[i].position.x;
        double yPosPathSegmentNext  = m_robotPose.position.y - m_pathArray.poses[i].position.y;

        double xyProjectionNext;

        if (xylengthPathSegmentNext<=0)
        {
            xyProjectionNext = 0.0;
        }
        else 
        {
            xyProjectionNext = (xPosPathSegmentNext*xPathSegmentNext + yPosPathSegmentNext*yPathSegmentNext)/xylengthPathSegmentNext;
        }
        

        if (xyProjectionNext <= 0)
        {
            xyProjectionNext = 0.0;
        }
            
        else if (xyProjectionNext>=1)
        {
            xyProjectionNext = 1;
        }

        double xProjectionNext  =   xPosPathSegmentNext - xyProjectionNext*xPathSegmentNext;    
        double yProjectionNext  =   yPosPathSegmentNext - xyProjectionNext*yPathSegmentNext;
        double xyProjLengthNext = xProjectionNext*xProjectionNext+yProjectionNext*yProjectionNext;
        if  ((xyProjLengthNext + 0.01) < xyProjLength) // TO not let the active counter change because of floating point compare
        {
            xyProjLength = xyProjLengthNext;          
            xyProjection = xyProjectionNext;
            m_activeCounter = i;
        }
        i++;
    }
    
    // Calculate the coordinates of the localised path
    double xPathSegmentLocalized =  m_pathArray.poses[m_activeCounter+1].position.x - m_pathArray.poses[m_activeCounter].position.x;
    double yPathSegmentLocalized =  m_pathArray.poses[m_activeCounter+1].position.y - m_pathArray.poses[m_activeCounter].position.y;
    double xPosPathSegmentLocalized  = m_robotPose.position.x - m_pathArray.poses[m_activeCounter].position.x;
    double yPosPathSegmentLocalized  = m_robotPose.position.y - m_pathArray.poses[m_activeCounter].position.y;

    if (std::abs(yPosPathSegmentLocalized)<0.001)
    {
        yPosPathSegmentLocalized = 0;
    }
    if (std::abs(yPathSegmentLocalized)<0.001)
    {
        yPathSegmentLocalized = 0;
    }

    // To find the direction of the orientation error, a cross product between relevant path segment and robot position is calculated
    // Cross product to find the correct sign of cross-track-error
    if (xPathSegmentLocalized*yPosPathSegmentLocalized>yPathSegmentLocalized*xPosPathSegmentLocalized)
    {
        m_xyProjLength      = -1*sqrt(xyProjLength);
    }
    else if (xPathSegmentLocalized*yPosPathSegmentLocalized < yPathSegmentLocalized*xPosPathSegmentLocalized)
    {
        m_xyProjLength      = sqrt(xyProjLength);
    }
    else
    {
        m_xyProjLength = 0;
    }
    
   // Define a new TF frame at that point!
    m_errorRefPoint.position.x = m_pathArray.poses[m_activeCounter].position.x + xyProjection* (m_pathArray.poses[m_activeCounter+1].position.x-m_pathArray.poses[m_activeCounter].position.x);
    m_errorRefPoint.position.y = m_pathArray.poses[m_activeCounter].position.y + xyProjection* (m_pathArray.poses[m_activeCounter+1].position.y-m_pathArray.poses[m_activeCounter].position.y);;
    m_errorRefPoint.position.z = 0;
    
    m_errorRefPoint.orientation.w = m_pathArray.poses[m_activeCounter].orientation.w + xyProjection* (m_pathArray.poses[m_activeCounter+1].orientation.w-m_pathArray.poses[m_activeCounter].orientation.w);
    m_errorRefPoint.orientation.z = m_pathArray.poses[m_activeCounter].orientation.z + xyProjection* (m_pathArray.poses[m_activeCounter+1].orientation.z-m_pathArray.poses[m_activeCounter].orientation.z);

    // Publish the TF
    m_Transformations.sendTransformations(m_globalFrame, "FB_Frame", m_errorRefPoint);

    // Write the orientation and cross track error to member variable
    m_xyProjOrientation   = m_Transformations.QuatToEuler(m_errorRefPoint.orientation.z,m_errorRefPoint.orientation.w);
    m_crossTrackError     = m_xyProjLength;
    m_orientationError    = MathFunc::shortest_angular_distance(m_thetaM,m_xyProjOrientation); 

    /*=========================================================================================*/
    /*= Calculation of the feedforward frame at a distance of lookahead from projection point =*/
    /*=========================================================================================*/

    // ToDO: Calculate variable lookahead
    double velocityLookAhead = m_lookAhead;

    // For calculation of the control law, define a new coordinate frame at a distance "m_lookAhead" from this error frame
    // Calculate the distance travelled from the active m_errorRefPoint and active counter +1 => this is initial distance
    double distanceFromErrorFrame = MathFunc::euclideanDistance<double>(m_errorRefPoint.position.x,m_errorRefPoint.position.y,m_pathArray.poses[(m_activeCounter+1)].position.x,m_pathArray.poses[(m_activeCounter+1)].position.y);
    double distanceLast = 0.0;
    m_activeCounterPred =  m_activeCounter+1;
        
    while(distanceFromErrorFrame < velocityLookAhead && m_activeCounterPred < m_msgSize -1 )
    {
        distanceFromErrorFrame+= MathFunc::euclideanDistance<double>(m_pathArray.poses[m_activeCounterPred].position.x,m_pathArray.poses[m_activeCounterPred].position.y,m_pathArray.poses[(m_activeCounterPred+1)].position.x,m_pathArray.poses[(m_activeCounterPred+1)].position.y);
        m_counterSF             = m_activeCounterPred;
        if(distanceFromErrorFrame < m_inplaceRadius)
        {
            m_counterInplace   = m_activeCounterPred;
        }
        if (distanceFromErrorFrame < velocityLookAhead)
        {
            distanceLast = distanceFromErrorFrame;
        }
        m_activeCounterPred++;
    }

    // Distance that remains to be calculated
    double distanceRemaining = velocityLookAhead- distanceLast;
    double distanceRatioRemaining = 0;

    double distancePathSegment = MathFunc::euclideanDistance<double>(m_pathArray.poses[m_activeCounterPred+1].position.x,m_pathArray.poses[m_activeCounterPred+1].position.y,m_pathArray.poses[(m_activeCounterPred)].position.x,m_pathArray.poses[(m_activeCounterPred)].position.y);

    if(distancePathSegment>0)
    {
        distanceRatioRemaining = distanceRemaining/distancePathSegment;
    }

    // define the controller frame
    m_controlRefPoint.position.x =m_pathArray.poses[m_activeCounterPred].position.x +  distanceRatioRemaining*(m_pathArray.poses[m_activeCounterPred+1].position.x-m_pathArray.poses[m_activeCounterPred].position.x);
    m_controlRefPoint.position.y =m_pathArray.poses[m_activeCounterPred].position.y +  distanceRatioRemaining*(m_pathArray.poses[m_activeCounterPred+1].position.y-m_pathArray.poses[m_activeCounterPred].position.y);
    m_controlRefPoint.position.z = 0.0;

    m_controlRefPoint.orientation.z = m_pathArray.poses[m_activeCounterPred].orientation.z + distanceRatioRemaining*(m_pathArray.poses[m_activeCounterPred+1].orientation.z-m_pathArray.poses[m_activeCounterPred].orientation.z);
    m_controlRefPoint.orientation.w = m_pathArray.poses[m_activeCounterPred].orientation.w + distanceRatioRemaining*(m_pathArray.poses[m_activeCounterPred+1].orientation.w-m_pathArray.poses[m_activeCounterPred].orientation.w);
    
    // Publish the TF
    m_Transformations.sendTransformations(m_globalFrame, "FF_Frame", m_controlRefPoint);
    
 geometry_msgs::Pose referecePoint;
    if(m_distanceToGoal__m < m_lookAhead)
    {
        /*If near to goal,final point in the plan is taken*/
        referecePoint   =  m_pathArray.poses[m_msgSize - 1];
        m_counterSF     =  m_msgSize - 1;
        ROS_INFO("Here 1 : %d",m_msgSize );
    }
    else
    {
    
        geometry_msgs::Pose updatedRefPoint    =  intersectCircle(m_pathArray.poses[m_activeCounterPred - 2],m_pathArray.poses[m_activeCounterPred - 1],m_robotPose,m_lookAhead);
       
        /*Otherwise the controlRef point is taken for SF frame computation*/
        updatedRefPoint.orientation            = m_pathArray.poses[m_activeCounterPred - 1].orientation;
        referecePoint                          = updatedRefPoint;
        ROS_INFO("Here 2 : %d",m_activeCounterPred);

    }


    ROS_WARN("Inside localisation()  : %f  %f  %f  %f",referecePoint.position.x,referecePoint.position.y,referecePoint.orientation.z,referecePoint.orientation.w);
    return referecePoint;
}


/**
 * @brief Find the intersection point for the look ahead radius and path segments.This will be used for the SF pose origin
*/
geometry_msgs::Pose LowLevelController::intersectCircle(geometry_msgs::Pose start_pose, geometry_msgs::Pose end_pose, geometry_msgs::Pose current_pose, double radius)
{
    double slope;
    double offset;
    double A, B, C, D;
    double cx = current_pose.position.x;
    double cy = current_pose.position.y;         
    if (end_pose.position.x == start_pose.position.x)
    {
        offset = end_pose.position.x;
        A = 1;
        B = (-2 * cy);
        C = (pow(cx, 2) + pow(cy, 2) + pow(offset, 2) - pow(radius, 2) - (2 * cx * offset));
    }
    else
    {
        slope = (end_pose.position.y - start_pose.position.y) / (end_pose.position.x - start_pose.position.x);
        offset = start_pose.position.y - (slope * start_pose.position.x);
        A = (1 + pow(slope, 2));
        B = ((2 * slope * offset) - (2 * cx) - (2 * cy * slope));
        C = (pow(offset, 2) - (2 * cy * offset) + pow(cx, 2) + pow(cy, 2) - pow(radius, 2));
    }

    D = (pow(B, 2) - (4 * A * C));


    geometry_msgs::Pose intersectionPoint;
    if (D >= 0)
    {
        double x1, x2, y1, y2;
        // if line if 90degree slope then use x=c equation
        if (end_pose.position.x == start_pose.position.x)
        {
            y1 = ((-B) + sqrt(D)) / (2 * A);
            y2 = ((-B) - sqrt(D)) / (2 * A);
            x1 = offset;
            x2 = offset;
        }
        // else use y = mx+c equation for line
        else
        {
            x1 = ((-B) + sqrt(D)) / (2 * A);
            x2 = ((-B) - sqrt(D)) / (2 * A);
            y1 = (slope * (((-B) + sqrt(D)) / (2 * A))) + offset;
            y2 = (slope * (((-B) - sqrt(D)) / (2 * A))) + offset;
        }
        // calculate distance of points from goal point and decide point shortest to the goal.
        double d1 = sqrt((pow((end_pose.position.y - y1), 2)) + (pow((end_pose.position.x - x1), 2)));
        double d2 = sqrt((pow((end_pose.position.y - y2), 2)) + (pow((end_pose.position.x - x2), 2)));
        if (d2 < d1) //if d2 is shorter select x2,y2 point
        {
            intersectionPoint.position.x = x2;
            intersectionPoint.position.y = y2;
        }
        // if d1 is shorter select x1, y1, point
        else
        {
            intersectionPoint.position.x = x1;
            intersectionPoint.position.y = y1;
        }
    }

    else
    {
        ROS_WARN("no intersection");
    }
    return intersectionPoint;
}


/*Predicts the robot position whenever the updated robot pose data is not received.This is based on robot's prev velocity and position*/
geometry_msgs::Pose LowLevelController::predictRobotPosition(geometry_msgs::Twist& odometryMsg)
{
    geometry_msgs::Quaternion quat_msg;

    float dt = 0.005; // Step size of the integration in secs, define as macro
    int nSteps = (m_robotDeltaTime*MICRO_SECONDS)/dt;
    m_Predict = m_robotPose;
    m_thetaM  = m_Transformations.QuatToEuler(m_robotPose.orientation.z,m_robotPose.orientation.w);
    for (int i=0;i< nSteps; i++)
    {
        m_Predict.position.x = m_Predict.position.x + odometryMsg.linear.x*cos(m_thetaM)*dt;
        m_Predict.position.y = m_Predict.position.y + odometryMsg.linear.x*sin(m_thetaM)*dt;
        m_thetaM = m_thetaM + odometryMsg.angular.z*dt;
    }

    quat_msg         =  m_Transformations.EulerToQuat(0.0,0.0,m_thetaM);
    m_Predict.orientation.z  = quat_msg.z;
    m_Predict.orientation.w  = quat_msg.w;
    return m_Predict;
}

void LowLevelController::getTransformations()
{
    // Get the transformations
    m_robotPoseBase_  = m_Transformations.getTransformations(m_globalFrame,m_baseFrame,m_robotPose);
    m_robotPoseSF_    = m_Transformations.getTransformations(m_globalFrame,m_targetFrame,m_sfRefPoint);
    m_inplaceBase_    = m_Transformations.getTransformations(m_globalFrame,m_baseFrame,m_inplacePoint);
    m_finalGoalBase_  = m_Transformations.getTransformations(m_globalFrame,m_baseFrame,m_lastPose);
}

/* Checks whether an updated robot pose is received*/
bool LowLevelController::isNewPoseReceived(geometry_msgs::Pose& robot_pose)
{
    bool changePosition     = ((robot_pose.position.x != m_lastRobotPose.position.x) || (robot_pose.position.y != m_lastRobotPose.position.y) );
    bool changeOrientation  = ((robot_pose.orientation.w != m_lastRobotPose.orientation.w) || (robot_pose.orientation.z != m_lastRobotPose.orientation.z) );
    if(changePosition || changeOrientation)
    {
        m_lastRobotPose = robot_pose;        
        return true;
    }
    return false;
}

void LowLevelController::resetControllerParameters()
{
    m_goalReached    = false;
    m_latchTarget    = false;
    /*Resetting the timers for linear and angular velocity limit*/
    m_angularT1 = m_angularT2 =  MathFunc::micros<uint64_t>();  
    m_linearT1  = m_linearT2  =  MathFunc::micros<uint64_t>();
    ROS_WARN("resetControllerParameters()  Reset");
}


controller_msgs::Debug LowLevelController::getDebugData()
{
    controller_msgs::Debug debug;
    debug.curvature                  = m_curvature;
    debug.error                      =  m_crossTrackError;
    debug.velProfile                 = m_velProfile;
    debug.robotVel.linear.x          = m_linearVel__m_s;
    debug.robotVel.angular.z         = m_angularVel__rad_s;
    debug.controlLaw.first           = m_approachControl;
    debug.controlLaw.second          = m_cteControl;
    debug.controlLaw.third           = m_orientationControl;
    debug.controlLaw.fourth          = m_feedForwardControl;
    debug.controlLaw.resultant       = m_resultant;
    debug.distanceToGoal             = m_activeCounter;
    return debug;
}

/* Sends the status of the goal reaching to the wrapper */
bool LowLevelController::isGoalReached()
{
    if(m_goalReached)
    {
        ROS_ERROR("Goal reached");
    }
    return m_goalReached;
}

/* Sends the velocity commands to  the wrapper */
geometry_msgs::Twist LowLevelController::executeController()
{
    geometry_msgs::Twist commandVelocity;
    commandVelocity.linear.x  = longitudinalController();
    commandVelocity.angular.z = lateralController();
    return commandVelocity;
}