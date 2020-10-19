#ifndef MATH_FUNC_H
#define MATH_FUNC_H

#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <algorithm>

#include <cmath>
#include <math.h>
#include <bits/stdc++.h> 

  /**
   * @Namespace math computation
   */

namespace MathFunc
{
     struct SplineSet
      {
        double a;
        double b;
        double c;
        double d;
        double x;
      };
    /**
     * @brief Given 2 angles, this returns the shortest angular difference.  The inputs and ouputs radians.
     * @param from angle from which to normalize
     * @param to till which angle to normalize
     * @result output would always be -pi <= result <= pi.  Adding the result to "from" will always get you an equivelent angle to "to"
    */

    template <typename T>
    T shortest_angular_distance(T from, T to)
    {
        T angle = to-from;
        T result = fmod(angle + M_PI, 2.0*M_PI);
        if(result <= 0.0) return result + M_PI;
        return result - M_PI;
    }

    /**
    * @brief  find the euclidean distance between two points
    * @param p_1 xpos for Point 1
    * @param p_2 ypos for Point 1
    * @param p_3 xpos for Point 2
    * @param p_4 ypos for Point 2
    * @re1turn the calculated distance
    */
    template<typename T>
    T euclideanDistance(T point1_x, T point1_y, T point2_x, T point2_y)
    {
        T x_diff, y_diff, distance;
        x_diff = (point1_x - point2_x);
        y_diff = (point1_y - point2_y);
        distance = sqrt(x_diff * x_diff + y_diff * y_diff);
        return distance;
    }

        /**
    * @brief checks if a point in path lies within the circle with LaRadius
    * @param center_x the center xposition of the circle
    * @param center_y the center yposition of the circle
    * @param point_x xpos of point to be tested
    * @param point_y ypos of point to be tested
    * @param radius the radius whose circle's intersection in the path to be found
    * @return whether the point lies within or outside the circle
    */

    template<typename T>
    bool isInsideCircle(T center_x, T center_y, T point_x, T point_y,T radius)
    {
        if ((point_x - center_x) * (point_x - center_x) + (point_y - center_y) * (point_y - center_y) <= radius * radius) 
            return true; 
        else
            return false;
    }

        /**
    * @brief find the area of the traiangle with given side length
    */ 
    template <typename T>
    T findTriArea(T s1, T s2, T s3)
    {
        if (s1 < 0 || s2 < 0 || s3 < 0 ||  (s1 + s2 <= s3) || (s1 + s3 <= s2) || (s2 + s3 <= s1) )
        {  
            std::cerr << "Not a valid triangle" << std::endl;  
            return 0;
        }  
        T s = (s1 + s2 + s3) / 2;  
        return sqrt(s * (s - s1) *  (s - s2) * (s - s3));   
    }

        /**
    * @brief find the sign of the input
    * @param data data whose sign is to check
    * @return the multiplicative for sign
    */
    template<typename T>
    int sign(T data)
    {
        int result = 0;
        if(data > 0) {
            result = 1;
        } else if(data < 0) {
            result = -1;
        } else {
            result = 0;
        }
        return result;
    }

    /**
     * @brief Returns time in microseconds
    */
    template <typename T>
    T micros()
    {
        T us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        return us;
    } 

    template <typename T>
    void Quincunx(int n,T u,T v, T w, T &q)
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

    template <typename T>
    std::vector<SplineSet> splineSmoother(const T &x, const T &y, const T &sigma, double lambda)
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
        Quincunx<std::vector<double>>(x.size(),u,v,w,q);

        std::vector<SplineSet> cubicSplineSmoother(n);
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

    template <typename T>
    void evaluateInterpolatedSpline2(const std::vector<SplineSet> &cubicSpline,const T &xlocal,T &xValues,T &yValues, double stepsize = 0.005 )
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

};

#endif
