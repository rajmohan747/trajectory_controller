#ifndef PATH_PROCESSING_H
#define PATH_PROCESSING_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include "Param.h"
#include "Transformations.h"

#include <APathProcessing.h>
#include "matplotlibcpp.h"
#include "Math_Func.h"
class pathProcessing : public APathProcessing
{
  private:

    /*
    struct SplineSet
      {
        double a;
        double b;
        double c;
        double d;
        double x;
      };
      */

    Params::PathGeneratorParams m_PathParams;
    std::vector<Params::PathData> m_globalProcessedPathArray, m_lastGlobalProcessedPathArray;
    
    Transformations m_Transformations;
    
  public:
    /* Constructor for pathProcessing class*/
    pathProcessing();

    /* Destructor for pathProcessing class*/
    ~pathProcessing();

    /* Returns processed path data*/
    // Vector of Path Points
    std::vector<Params::PathData> getProcessedPath(const geometry_msgs::PoseArray& m_pathArray, const geometry_msgs::Twist& odometryMsg, std::vector<Params::PathData>&,bool &splineError, double exitVelocity=0.0) override;

  private:

    std::vector<double> velocityProfiling(const std::vector<double>& velocity, const std::vector<double>& distances);

    double computeLinearVelocity(double curvature);

    //void evaluateInterpolatedSpline2(const std::vector<SplineSet> &cubicSpline,const std::vector<double> &xlocal,std::vector<double> &xValues,std::vector<double> &yValues, double stepsize= 0.005 );

    //void Quincunx(int n,std::vector<double>,std::vector<double>v,std::vector<double>w,std::vector<double>&q);

    //std::vector<SplineSet> splineSmoother(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &sigma, double lambda);
};

#endif