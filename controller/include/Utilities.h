#ifndef UTILITIES_H
#define UTILITIES_H

#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <algorithm>

#include <cmath>
#include <math.h>
#include <bits/stdc++.h> 


namespace Utilities
{
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


    /* @brief find the sign of the input
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
};

#endif