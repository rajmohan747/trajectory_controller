#ifndef PARAM_H
#define PARAM_H

#include <geometry_msgs/Pose.h>

class Params
{
    public:

        struct Time
        {
            uint64_t m_startTime;
            uint64_t m_endTime;
            uint64_t m_deltaTime;

            uint64_t m_robotStartTime;
            uint64_t m_robotEndTime;
            uint64_t m_robotDeltaTime;


        }robotTime;

        struct RobotTolerance
        {
            double m_xyGoalTolerance__m;
            double m_yawTolerance__rad;
            double m_inplaceTolerance__rad;

        }robotTolerance;

        struct Robot
        {
            double m_maxLinearVel__m_s;
            double m_minLinearVel__m_s;
            double m_maxAngularVel__rad_s;
            double m_maxInplaceAngularVel__rad_s;
            double m_linearAccLimit__m_s2;
            double m_linearDeccLimit__m_s2;
            double m_angularAccLimit__rad_s2;
            double m_lateralAcceleration__m_s2;

        }robot;
        
        struct ControlLaw
        {
            double m_k1Gain;
            double m_k2Gain;
            double m_kDeltaGain;
            double m_gamma;
            double m_thetaA;
            double m_inplaceRadius;
            double m_lookAhead;
            double m_Kp;
            double m_Kd;
            bool m_tuningControlLaw;

        }controlLaw;

       
};
#endif
