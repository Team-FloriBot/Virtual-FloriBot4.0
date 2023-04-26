#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <ros/time.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>

namespace kinematics
{
    struct DifferentialWheelSpeed
    {
        double leftWheel;
        double rightWheel;
    };

    class differentialDrive
    {
        public:
        differentialDrive(double axesLength, double wheelDiameter);
        differentialDrive();
        ~differentialDrive();

        DifferentialWheelSpeed inverseKinematics(geometry_msgs::Twist cmdVelMsg);
        geometry_msgs::Pose2D forwardKinematics(DifferentialWheelSpeed WheelSpeed, ros::Time Timestamp);
        geometry_msgs::Pose2D getActualPose();
        geometry_msgs::Twist getSpeed();

        void reset();
        void setParam(double axesLength, double wheelDiameter);

        private:

        geometry_msgs::Pose2D Pose_;
        DifferentialWheelSpeed WheelSpeed_;
        geometry_msgs::Twist Speed_;
        ros::Time TimeStamp_;
        double axesLength_, wheelDiameter_, wheelCircumference_, wheelRadius_;

    };
} 

#endif