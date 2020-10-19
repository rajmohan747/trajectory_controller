#ifndef APATHPROCESSING_H
#define APATHPROCESSING_H

#include "Param.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>

#include <vector>
class APathProcessing
{
    public:
        virtual std::vector<Params::PathData> getProcessedPath(const geometry_msgs::PoseArray& m_pathArray, const geometry_msgs::Twist& odometryMsg, std::vector<Params::PathData>&,bool &splineError, double exitVelocity=0.0) = 0;

};

#endif