#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class Transformations
{
    public:
    /**
    * @brief  obtain the transformation between two frames currently from map frame to Serret-Frenet frame 
    * @param source_frame frame from which to be transformed
    * @param target_frame frame to which to be transformed
    * @param inputPose the pose to be transformed
    * @return the resultant pose that has been transformed
    */
    geometry_msgs::Pose getTransformations(const std::string& source_frame,const std::string& target_frame,const geometry_msgs::Pose& inputPose);

    /**
    * @brief  obtain the transformation between two frames currently from map frame to Serret-Frenet frame 
    * @param source_frame frame from which to be transformed
    * @param target_frame frame to which to be transformed
    * @param inputPose the pose to be transformed
    * @return the resultant pose that has been transformed
    */

    void sendTransformations(const std::string& parent_frame,const std::string& target_frame,const geometry_msgs::Pose& inputPose);

    /**
    * @brief converts Quaternion to Euler angles
    * @param z the Quaternion orientation z
    * @param w the Quaternion orientation w
    * @return the euler angle
    */

   // ToDo: need to be removed from here
    double QuatToEuler(double z,double w);
    
   // ToDo: need to be removed from here
    geometry_msgs::Quaternion EulerToQuat (double const roll, double const pitch, double const yaw);


    /**
    * @brief  obtain the quaternion interpolated output between two quaterions
    * @param v0 first quaternion
    * @param v1 second quaternion
    * @param t interpolation parameter
    * @return the resultant quaternion output
    */
    geometry_msgs::Quaternion slerp(geometry_msgs::Quaternion v0, geometry_msgs::Quaternion v1, double t);

    
    private:
    // Message types
    geometry_msgs::PoseStamped poseSource, poseTarget ;
    geometry_msgs::Pose poseTransformed;

    //Listener
    tf::TransformListener m_listener;

    // Broadcaster
    tf::TransformBroadcaster m_broadcaster;
    tf::Transform m_transform;
};

#endif