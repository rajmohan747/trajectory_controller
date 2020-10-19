#include "pathProcessing.h"

namespace plt = matplotlibcpp;
using namespace std;

/* Constructor for path processing */
pathProcessing::pathProcessing()
{
    // Only here to setup the .cpp file
    // Will be removed
    m_PathParams.m_lateralAcceleration__m_s2    = 0.2;
    m_PathParams.m_maxLinearVel__m_s            = 1.0;
    m_PathParams.m_minLinearVel__m_s            = 0.0;
    m_PathParams.m_linearAccLimit__m_s2         = 0.2;
    m_PathParams.m_linearDeccLimit__m_s2        = 0.5;
    m_PathParams.m_linearJerk_m_s3              = 0.05;
}

/* Destructor for path processing */
pathProcessing::~pathProcessing()
{

}

std::vector<Params::PathData> pathProcessing::getProcessedPath(const geometry_msgs::PoseArray& m_pathArray, const geometry_msgs::Twist& odometryMsg,std::vector<Params::PathData>& m_lastGlobalProcessedPathArray, bool &splineComputationError,double exitVelocity)
{
    std::vector<double> xposition,yposition,orientation,distance;
    double distanceInit = 0.0;
    double lastdistanceInit;
    for(unsigned i = 0; i < m_pathArray.poses.size() ; i++)
    {
        if (i==0) // distance at first point is zero
        {
            distanceInit     += 0; 
        }
        else    
        {
            distanceInit     += MathFunc::euclideanDistance<double>(m_pathArray.poses[i-1].position.x,m_pathArray.poses[i-1].position.y,m_pathArray.poses[i].position.x,m_pathArray.poses[i].position.y); 
        }
        if ((distanceInit-lastdistanceInit)<0.001 && i!=0)
        {
            //    ROS_WARN("The distance values are too small, the value of i is: %d, not adding to vector", i);
        }
        else
        {
            xposition.push_back(m_pathArray.poses[i].position.x);
            yposition.push_back(m_pathArray.poses[i].position.y);
            distance.push_back(distanceInit);
            lastdistanceInit = distanceInit;
            orientation.push_back(m_Transformations.QuatToEuler(m_pathArray.poses[i].orientation.z,m_pathArray.poses[i].orientation.w));   // only for debug
        }
    }
    //====================//
    //==Spline Smoother==//
    //===================//

    // Set the standard deviation of points
    std::vector<double> sigma; 
    for(unsigned i = 0; i < xposition.size() ; i++)
    {
        sigma.push_back(0.1);
    }

    // Smoothing factor. 1-> no smoothing. 0 -> Pure smoothing
    double smoothingFactor = 0.8;
    vector<MathFunc::SplineSet> cubicSplineSmoother_dist_xpos = MathFunc::splineSmoother<std::vector<double>>(distance,xposition,sigma,smoothingFactor);
    vector<MathFunc::SplineSet> cubicSplineSmoother_dist_ypos = MathFunc::splineSmoother<std::vector<double>>(distance,yposition,sigma,smoothingFactor);
    std::vector<double> posX_interpolant2 = {}, posX_interpolated2 = {};
    std::vector<double> posY_interpolant2 = {}, posY_interpolated2 = {};
    MathFunc::evaluateInterpolatedSpline2(cubicSplineSmoother_dist_xpos,distance,posX_interpolant2,posX_interpolated2,0.02);
    MathFunc::evaluateInterpolatedSpline2(cubicSplineSmoother_dist_ypos,distance,posY_interpolant2,posY_interpolated2,0.02);

    std::vector<double> a,b,c,d,a1,b1,c1,d1;
    splineComputationError = false;
    for (unsigned i = 0; i<cubicSplineSmoother_dist_xpos.size();i++ )
    {
        // These conditions are required, since in case of collinear points, the fit return nan value
        if (abs(cubicSplineSmoother_dist_xpos[i].c)>1.1 || abs(cubicSplineSmoother_dist_ypos[i].c)>1.1 || std::isnan(posX_interpolated2[i]) || std::isnan(posY_interpolated2[i]))
        {
            splineComputationError = true;
        }
    }
    m_globalProcessedPathArray.clear();
    m_globalProcessedPathArray.resize(0);
	
    if (splineComputationError)
    {
        ROS_ERROR("!! ---SPLINE ERROR: using last solution--- !!");
        m_globalProcessedPathArray = m_lastGlobalProcessedPathArray;
        
        return m_globalProcessedPathArray;
    }
    else
    {      
        // Calculation of orientation angles
        std::vector<double> orientation2; 
        double yawinit;
        for(unsigned i = 0; i < posX_interpolated2.size()-1 ; i++)
        {
            yawinit =  std::atan2((posY_interpolated2[i+1]-posY_interpolated2[i]),(posX_interpolated2[i+1]-posX_interpolated2[i]));
            orientation2.push_back(yawinit);
        }
        double lastorientation= orientation2[orientation2.size()-1];
        orientation2.push_back(lastorientation);

        //Calculate the curvature and velocity
        double deltaTheta, deltaS;
        std::vector<double> curvature, velocity; 
        //curvature.push_back(0); // Condition from spline
        for(unsigned i = 0; i < orientation2.size()-1 ; i++)
        {
            deltaTheta = MathFunc::shortest_angular_distance<double>(orientation2[i],orientation2[i+1]);
            deltaS = posX_interpolant2[i+1]-posX_interpolant2[i];
            if (!std::isfinite(deltaTheta/deltaS))
            {
                ROS_ERROR("The curvature is infinite");
            }
            curvature.push_back(deltaTheta/deltaS);
            if (i==0)
            {
                velocity.push_back(odometryMsg.linear.x);
            }
            else
            {
                velocity.push_back(computeLinearVelocity(deltaTheta/deltaS));
            }
        }
        // curvature at last point is same as curvature at second last point
        double lastcurvature= curvature[curvature.size()-1]; 
        curvature.push_back(lastcurvature);

        // Exit Velocity as optional parameter
        velocity.push_back(exitVelocity);

        // Limit the velocity based on accelerations
        std::vector<double> velocityProfilled,velocitySmoothed,distancePoints;
        velocityProfilled = velocityProfiling(velocity,posX_interpolant2);
        
        tf2::Quaternion quat;
        Params::PathData  mPathPoint;
        for (unsigned i=0; i< posX_interpolant2.size();i++)
        {
            mPathPoint.m_path.position.x = posX_interpolated2[i];

            mPathPoint.m_path.position.y = posY_interpolated2[i];
            quat.setRPY(0,0,orientation2[i]);
            mPathPoint.m_path.orientation.x = quat.x();
            mPathPoint.m_path.orientation.y = quat.y();
            mPathPoint.m_path.orientation.z = quat.z();
            mPathPoint.m_path.orientation.w = quat.w();
            
            mPathPoint.orientation = orientation2[i];

            mPathPoint.m_distFromInital = posX_interpolant2[i];

            mPathPoint.m_curvatureValue = curvature[i];

            mPathPoint.m_velLinear = velocityProfilled[i];

            m_globalProcessedPathArray.push_back(mPathPoint);
        }
        m_lastGlobalProcessedPathArray = m_globalProcessedPathArray;
    }

    // This is optional (Calculates the orientation as a function of path length)
    // vector<SplineSet> cubicSplineSmoother_dist_orient = splineSmoother(distance,orientation,sigma,smoothingFactor);
    // std::vector<double> orientation_interpolant2 = {}, orientation_interpolated2 = {};
    // evaluateInterpolatedSpline2(cubicSplineSmoother_dist_orient,distance,orientation_interpolant2,orientation_interpolated2); 
    
    
    //This is optional.
    /* Additional smoothing of processed velocity profile
    vector<SplineSet> cubicSplineSmoother_dist_velocity = splineSmoother(posX_interpolant2,velocityProfilled,sigma,0.9);
    std::vector<double> velocity_interpolant2 = {}, velocity_interpolated2 = {};
    evaluateInterpolatedSpline2(cubicSplineSmoother_dist_velocity,posX_interpolant2,velocity_interpolant2,velocity_interpolated2);  
    */
    /*
    plt::figure(1);
    plt::subplot(2,2,1);
    plt::named_plot("s-x",a,"r");
    plt::subplot(2,2,2);
    plt::named_plot("s-x",b,"r");

    
    plt::subplot(2,2,3);
    plt::named_plot("s-x",c,"r");
    plt::subplot(2,2,4);
    plt::named_plot("s-x",d,"r");
    plt::xlabel("y position");
    
    plt::legend();
    plt::show();

    /*
    plt::subplot(2,2,1);
    plt::named_plot("s-x",distance,xposition,"bx");
    plt::named_plot("s-x",posY_interpolant2,posX_interpolated2,"ro");
    plt::xlabel("x position");
    plt::ylabel("y position");
    plt::subplot(2,2,2);
   // plt::named_plot("s-yaw",distance,orientation,"b");
   plt::named_plot("s-x",distance,yposition,"bx");  
   plt::named_plot("s-y",posY_interpolant2,posY_interpolated2,"ro");
    plt::xlabel("distance");
    plt::ylabel("orientation");
    
    plt::subplot(2,2,3);
    plt::named_plot("s-x",posX_interpolated2,"bx");
    plt::xlabel("x position");
    plt::subplot(2,2,4);
    plt::named_plot("s-x",posY_interpolated2,"bx");
  //  plt::named_plot("s-velocity1",posX_interpolant2,velocity_interpolated2,"-.g");
    plt::xlabel("y position");
    
    plt::legend();
    plt::show();
    plt::close();
    */
   return m_globalProcessedPathArray;
}

std::vector<double> pathProcessing::velocityProfiling(const std::vector<double> &velocityInput, const std::vector<double>& distances)
{
    /*Iteration Vector*/
    vector<double> velocityIteration, distanceOriginal, accelerationFinal; 
    double deltaDistance,acceleration;

    for (int i=0; i < distances.size(); i++)
    {
        velocityIteration.push_back(velocityInput[i]);
        distanceOriginal.push_back(distances[i]);
        accelerationFinal.push_back(0);
    }
    for(int i=1; i < (distances.size()-1);i++)
    {
        deltaDistance =   distances[i] - distances[i-1];
        acceleration  =  (pow(velocityIteration[i],2) - pow(velocityIteration[i-1],2))/(2*deltaDistance) ;
        if(acceleration > m_PathParams.m_linearAccLimit__m_s2)
        {
            acceleration    = m_PathParams.m_linearAccLimit__m_s2;
        }
        if (acceleration >0)
        {
            accelerationFinal[i] = acceleration;
        }
        velocityIteration[i] = std::min(sqrt(pow(velocityIteration[i-1],2) + 2*acceleration*deltaDistance), velocityIteration[i]);
    }
    
    for(int i=(distances.size()-1); i > 0; i--)
    {
        deltaDistance       = (distances[i] - distances[i-1]);
        acceleration        = (pow(velocityIteration[i-1],2) - pow(velocityIteration[i],2))/(2*deltaDistance);
        if(acceleration > m_PathParams.m_linearDeccLimit__m_s2)
        {
            acceleration    = m_PathParams.m_linearDeccLimit__m_s2;
        }
        if (acceleration >0)
        {
            accelerationFinal[i-1] = -acceleration;
        }
        velocityIteration[i-1]       = std::min(sqrt(pow(velocityIteration[i],2) + 2*acceleration*deltaDistance),velocityIteration[i-1]);
    }

    return velocityIteration;

    /*
    // Perform Smoothing based on superfilter (For negative acceleraions)
    std:vector<double> velocityIterationOrg,accelOrg;
    for (unsigned i = 0; i<velocityIteration.size();i++)
    {
        velocityIterationOrg.push_back(velocityIteration[i]);
        accelOrg.push_back(accelerationFinal[i]);
    }

    
    double deltaAcceleration, deltaDistance, distanceS1, distanceS2;
    unsigned j =0;  
    bool exitFlag = false;
    while (j<100)
    {
        for (unsigned i = 1; i < velocityIteration.size()-1;i++)    
        {
            exitFlag = true;
            deltaAcceleration =  accelerationFinal[i-1] - accelerationFinal[i];
            //deltaDistance = distances[i-1]-distances[i]; 
            if (deltaAcceleration > std::min((m_PathParams.m_linearJerk_m_s3)/velocityIteration[i-1],0.02))
            {   
                exitFlag = false;
                distanceS1 = distances[i] - distances[i-1];
                distanceS2 = distances[i+1] - distances[i];
                accelerationFinal[i-1] = accelerationFinal[i-1] - (accelerationFinal[i-1] - accelerationFinal[i])/3;
                velocityIteration[i] = sqrt(std::max((pow(velocityIteration[i-1],2)+ 2*accelerationFinal[i-1]*distanceS1),0.0));
                accelerationFinal[i] = (pow(velocityIteration[i],2) - pow(velocityIteration[i-1],2)) /(2*distanceS2);
            }
        } 
        j++;       
    }
    */
    
}  

double pathProcessing::computeLinearVelocity(double curvature)
{
    double velocity =  std::fabs(sqrt(m_PathParams.m_lateralAcceleration__m_s2 / fabs(curvature)));  
    if (velocity > m_PathParams.m_maxLinearVel__m_s)
    {
        velocity = m_PathParams.m_maxLinearVel__m_s;
    }
    return velocity;    
}

/*
void pathProcessing::evaluateInterpolatedSpline2(const std::vector<pathProcessing::SplineSet> &cubicSpline,const std::vector<double> &xlocal,std::vector<double> &xValues,std::vector<double> &yValues, double stepsize )
{
    bool endReached = false;
    double startdistance = 0;
    double interpolatedValue;
    double deltaX;
    int i =0; 
    while(!endReached)
    {   
        if (startdistance>xlocal[i+1])
        {
            i++;
        }
        xValues.push_back(startdistance);
        deltaX = (startdistance-xlocal[i]) ;
        interpolatedValue = cubicSpline[i].d + cubicSpline[i].c*deltaX + cubicSpline[i].b*pow(deltaX,2)+ cubicSpline[i].a*pow(deltaX,3);
        yValues.push_back(interpolatedValue);
        
        startdistance =startdistance + stepsize;
        if(startdistance> xlocal[xlocal.size()-1])
        {
            endReached =true;
        }
    }
}


std::vector<pathProcessing::SplineSet> pathProcessing::splineSmoother(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &sigma, double lambda)
{
    int n = x.size()-1;
    std::vector<double> h(n), r(n), f(n), p(n), q(n), u(n), v(n), w(n);
    int i,j;

    // set the smoothing factor based on user input
    double mu = 2*(1-lambda)/(3*lambda);

    h[0]= x[1]-x[0]; // difference between x values
    r[0] = 3/h[0];

    // dummy
    f[0] = 0; // Not needed
    p[0] = 0; // Not needed
    q[0] = 0; // Condition that b0 =0, linear at start
    for (i=1; i<n; i++)
    {    
        h[i]= x[i+1]-x[i];
        r[i]= 3/h[i];
        f[i]= -1*(r[i-1]+r[i]);
        p[i]= 2*(x[i+1]-x[i-1]);
        q[i]= 3*(y[i+1]-y[i])/h[i] -3*(y[i]-y[i-1])/h[i-1];
    }
    
    // This is done, since the linear element i.e. b at first point is always zero
    u[0]=0;
    v[0]=0;
    w[0]=0;

    for (i=1;i<n;i++)
    {
        u[i] = pow(r[i-1],2)*sigma[i-1] + pow(f[i],2)*sigma[i] + pow(r[i],2)*sigma[i+1];
        u[i] = mu*u[i] + p[i];
        v[i] = f[i]*r[i]*sigma[i] + r[i]*f[i+1]*sigma[i+1];
        v[i] = mu*v[i] + h[i];
        w[i] = mu*r[i]*r[i+1]*sigma[i+1];
    }
    

    // Do the factorisation
    Quincunx(x.size(),u,v,w,q);

    std::vector<pathProcessing::SplineSet> cubicSplineSmoother(n);
    cubicSplineSmoother[0].d = y[0] - mu*r[0]*q[1]*sigma[0];
    cubicSplineSmoother[1].d = y[1] - mu*(f[1]*q[1]+ r[1]*q[2])*sigma[1];
    cubicSplineSmoother[0].a = q[1]/(3*h[0]);
    cubicSplineSmoother[0].b = 0;
    cubicSplineSmoother[0].c = (cubicSplineSmoother[1].d - cubicSplineSmoother[0].d)/h[0] - q[1]*h[0]/3;
    r[0] = 0;
    cubicSplineSmoother[0].x = 0;
   
    for(j=1; j<n;j++)
    {
        cubicSplineSmoother[j].a =  (q[j+1]-q[j])/(3*h[j]);
        cubicSplineSmoother[j].b =  q[j];
        cubicSplineSmoother[j].c =  (q[j]+q[j-1])*h[j-1] + cubicSplineSmoother[j-1].c ;
        cubicSplineSmoother[j].d =  r[j-1]*q[j-1] + f[j]*q[j] + r[j]*q[j+1] ;
        cubicSplineSmoother[j].d =  y[j] - mu*cubicSplineSmoother[j].d*sigma[j];
        cubicSplineSmoother[j].x =  0;
    }
    return cubicSplineSmoother;
}

void pathProcessing::Quincunx(int n,std::vector<double> u,std::vector<double>v,std::vector<double>w,std::vector<double>&q)
{
    int j;
    for (j=1;j< n-1;j++)
    {
        if (j==1)
        {
            u[j]=  u[j] - 0 -0 ; // u[j-2]*pow(w[j-2],2)= 0 , u[j-1]*pow(v[j-1],2) = 0
            v[j] = (v[j]-u[j-1]*v[j-1]*w[j-1])/u[j];
            w[j] = w[j]/u[j];
        }
        else
        {
             u[j] =  u[j] - u[j-2]*pow(w[j-2],2) -u[j-1]*pow(v[j-1],2) ;
             v[j] = (v[j]-u[j-1]*v[j-1]*w[j-1])/u[j];
             w[j] = w[j]/u[j];
        }
    }

    // Forward loop
    for(j=1;j<n-1;j++)
    {
        if (j==1)
        {
            q[j] = q[j] - v[j-1]*q[j-1] - 0; // w[j-2]*q[j-2] =0;
        }
        else
        {
            q[j] = q[j] - v[j-1]*q[j-1] - w[j-2]*q[j-2];
        }
    }

    for(j=1;j<n-1;j++)
    {
        q[j] = q[j]/u[j];
    }
    
    q.push_back(0); // nth element
    q.push_back(0); // n+1 th element
    // Backward loop
    for(j=n-1;j>0;j--)
    {
        q[j]= q[j] - v[j]*q[j+1] -w[j]*q[j+2];
    }
}

*/