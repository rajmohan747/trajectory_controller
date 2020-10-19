#include "Transformations.h"

geometry_msgs::Pose Transformations::getTransformations(const std::string& source_frame,const std::string& target_frame,const geometry_msgs::Pose& inputPose)
{
    poseSource.header.frame_id     = source_frame;
    poseSource.header.stamp        = ros::Time(0);
    poseSource.pose.position       = inputPose.position;
    poseSource.pose.orientation    = inputPose.orientation;
    std::string targetFrame        = target_frame;
    std::string sourceFrame        = source_frame;
    
    m_listener.waitForTransform(targetFrame, sourceFrame, ros::Time(0), ros::Duration(1.0));
    try
    {
        m_listener.transformPose(targetFrame, poseSource, poseTarget); 
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("Computations:: Received an exception trying to transform a point : %s  %s  %s ", ex.what(),target_frame.c_str(),source_frame.c_str());
    }

    poseTransformed.position     = poseTarget.pose.position;
    poseTransformed.orientation  = poseTarget.pose.orientation;
    return poseTransformed;
}


void Transformations::sendTransformations(const std::string& parent_frame,const std::string& target_frame,const geometry_msgs::Pose& inputPose)
{
    std::string targetFrame           = target_frame;
    std::string parentFrame           = parent_frame;
    m_transform.setOrigin(tf::Vector3(inputPose.position.x,inputPose.position.y, 0.0) );
    m_transform.setRotation(tf::Quaternion(0.0, 0.0, inputPose.orientation.z, inputPose.orientation.w));          
    m_broadcaster.sendTransform(tf::StampedTransform(m_transform, ros::Time(0),parentFrame, targetFrame)); 
}

// ToDo: need to be removed from here
double Transformations::QuatToEuler(double z,double w)
{
    double roll, pitch, yaw;
    tf::Quaternion q(0,0,z,w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

geometry_msgs::Quaternion Transformations::EulerToQuat (double const roll, double const pitch, double const yaw)
{
  /* Abbreviations for the various angular functions */ 
    geometry_msgs::Quaternion quatValue;
    double const cy = ::cos(yaw * 0.5);
    double const sy = ::sin(yaw * 0.5);
    double const cr = ::cos(roll * 0.5);
    double const sr = ::sin(roll * 0.5);
    double const cp = ::cos(pitch * 0.5);
    double const sp = ::sin(pitch * 0.5);
    
    quatValue.w = cy * cr * cp + sy * sr * sp;
    quatValue.x = cy * sr * cp - sy * cr * sp;
    quatValue.y = cy * cr * sp + sy * sr * cp;
    quatValue.z = sy * cr * cp - cy * sr * sp;
    return quatValue;
}



geometry_msgs::Quaternion Transformations::slerp(geometry_msgs::Quaternion v0, geometry_msgs::Quaternion v1, double t) 
{
    geometry_msgs::Quaternion result;
    tf2::Quaternion startAngle,endAngle,resultAngle;
    tf2::convert(v0 , startAngle);    
    tf2::convert(v1 , endAngle);    
    startAngle.normalize();
    endAngle.normalize();


    double dot =  startAngle[0]*endAngle[0]  + startAngle[1]*endAngle[1]  + startAngle[2]*endAngle[2] + startAngle[3]*endAngle[3];//dot_product(v0, v1);

    // If the dot product is negative, slerp won't take
    // the shorter path. Note that v1 and -v1 are equivalent when
    // the negation is applied to all four components. Fix by 
    // reversing one quaternion.
    if (dot < 0.0) 
    {
        endAngle[0] = -endAngle[0];
        endAngle[1] = -endAngle[1];
        endAngle[2] = -endAngle[2];
        endAngle[3] = -endAngle[3];
        dot = -dot;
    }

    const double DOT_THRESHOLD = 0.9995;
    
    if (dot > DOT_THRESHOLD) 
    {
        // If the inputs are too close for comfort, linearly interpolate
        // and normalize the result.

        resultAngle[0] = startAngle[0] + t*(endAngle[0] - startAngle[0]);
        resultAngle[1] = startAngle[1] + t*(endAngle[1] - startAngle[1]);
        resultAngle[2] = startAngle[2] + t*(endAngle[2] - startAngle[2]);
        resultAngle[3] = startAngle[3] + t*(endAngle[3] - startAngle[3]);
        resultAngle.normalize();

        result.x  = resultAngle[0];
        result.y  = resultAngle[1];
        result.z  = resultAngle[2];
        result.w  = resultAngle[3];

        return result;
    }

    // Since dot is in range [0, DOT_THRESHOLD], acos is safe
    double theta_0 = acos(dot);        // theta_0 = angle between input vectors
    double theta = theta_0*t;          // theta = angle between v0 and result
    double sin_theta = sin(theta);     // compute this value only once
    double sin_theta_0 = sin(theta_0); // compute this value only once

    double s0 = cos(theta) - dot * sin_theta / sin_theta_0;  // == sin(theta_0 - theta) / sin(theta_0)
    double s1 = sin_theta / sin_theta_0;

      
    result.x  = s0*startAngle[0]  +  s1*endAngle[0];
    result.y  = s0*startAngle[1]  +  s1*endAngle[1];
    result.z  = s0*startAngle[2]  +  s1*endAngle[2];
    result.w  = s0*startAngle[3]  +  s1*endAngle[3];
    

    return result;
    

}