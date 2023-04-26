#ifndef ARTICULATED_DRIVE_H
#define ARTICULATED_DRIVE_H

#include "drives/differential_drive.h"
#include <ros/time.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace kinematics

{

struct articulatedWheelSpeed
{
    DifferentialWheelSpeed Front, Rear;
};

enum coordinate
{
    Front,
    Rear,
    JointFront,
    JointRear
};

class ArticulatedDrive
{
    public:
    ArticulatedDrive();
    ArticulatedDrive(double axesLength, double wheelDiameter, coordinate Base);
    ~ArticulatedDrive();

    articulatedWheelSpeed inverseKinematics(geometry_msgs::Twist cmdVelMsg);
    geometry_msgs::Pose2D forwardKinematics(articulatedWheelSpeed WheelSpeed, ros::Time Timestamp);
    geometry_msgs::Pose2D getActualPose(coordinate Frame);
    geometry_msgs::Twist getSpeed();

    void setParam(double AxesLength, double WheelDiameter, coordinate Base);
    private:
    
    tf2_ros::Buffer TFBuffer_;
    tf2_ros::TransformListener* pTF_Listener_;
    kinematics::coordinate Base_;
    kinematics::differentialDrive frontDrive_, rearDrive_;
    double frontlength_, rearlength_;
};
}
#endif