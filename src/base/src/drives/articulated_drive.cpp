#include "drives/articulated_drive.h"
#include <ros/ros.h>


kinematics::ArticulatedDrive::ArticulatedDrive()
{
    pTF_Listener_=new tf2_ros::TransformListener(TFBuffer_);
}


kinematics::ArticulatedDrive::ArticulatedDrive(double axesLength, double wheelDiameter, coordinate Base):
        frontDrive_(axesLength, wheelDiameter), rearDrive_(axesLength, wheelDiameter), Base_(Base)
        {
            pTF_Listener_=new tf2_ros::TransformListener(TFBuffer_);
        }


kinematics::ArticulatedDrive::~ArticulatedDrive() 
{
    delete pTF_Listener_;
}

kinematics::articulatedWheelSpeed kinematics::ArticulatedDrive::inverseKinematics(geometry_msgs::Twist cmdVelMsg)
{
    articulatedWheelSpeed retVal;
    geometry_msgs::Twist FrontMsg, RearMsg;
    geometry_msgs::TransformStamped Axes2Joint, Joint2Joint, Joint2Axes;
    tf2::Vector3 SpeedAxesFront, SpeedJointFront, SpeedJointRear, SpeedAxesRear;
    tf2::Vector3 OmegaFront, OmegaRear;
    tf2::Vector3 TranslationFront, TranslationRear;
    tf2::Quaternion Rotation;


    //Calculate Speeds for the two differential drives regarding the base frame
    try
    {
        switch (Base_)
        {
        //Front Speed is given
        case coordinate::Front:
            //Get latest Transforms

            Axes2Joint=TFBuffer_.lookupTransform("jointFront", "axesFront", ros::Time(0));
            Joint2Joint=TFBuffer_.lookupTransform("jointRear", "jointFront", ros::Time(0));
            Joint2Axes=TFBuffer_.lookupTransform("axesRear", "jointRear", ros::Time(0));

            //Get Data from the Messages, since the tf2::fromMsg-Function returns Linking-Errors with workaround
            SpeedAxesFront.setValue(cmdVelMsg.linear.x,cmdVelMsg.linear.y,cmdVelMsg.linear.z);
            OmegaFront.setValue(cmdVelMsg.angular.x,cmdVelMsg.angular.y,cmdVelMsg.angular.z);
            TranslationFront.setValue(-Axes2Joint.transform.translation.x, -Axes2Joint.transform.translation.y, -Axes2Joint.transform.translation.z);
            TranslationRear.setValue(-Joint2Axes.transform.translation.x, -Joint2Axes.transform.translation.y, -Joint2Axes.transform.translation.z);
            Rotation.setValue(Joint2Joint.transform.rotation.x,Joint2Joint.transform.rotation.y, Joint2Joint.transform.rotation.z, Joint2Joint.transform.rotation.w);           
            //tf2::fromMsg(cmdVelMsg.linear, SpeedAxesFront);
            //tf2::fromMsg(cmdVelMsg.angular, OmegaFront);
            //tf2::fromMsg(Axes2Joint.transform.translation,TranslationFront);
            //tf2::fromMsg(Joint2Axes.transform.translation,TranslationRear);
            //tf2::fromMsg(Joint2Joint.transform.rotation, Rotation);


            if (abs((double)Rotation.getAngle()>M_PI/2))
            {
                retVal.Front.leftWheel=0;
                retVal.Front.rightWheel=0;
                retVal.Rear.leftWheel=0;
                retVal.Rear.rightWheel=0;
                return retVal;
            }


            //Calculate Speed Joint Front
            SpeedJointFront=SpeedAxesFront+OmegaFront.cross(TranslationFront);

            //Transform Speed in JointRear, because they have to move with the same Speed
            SpeedJointRear=SpeedJointFront.rotate(Rotation.getAxis(), Rotation.getAngle());

            //Calculate needed Speed and Omega for AxesRear, assuming that Z and Y for the Translation from the Joint to the Axes are zero
            OmegaRear.setValue(0,0,-SpeedJointRear.y()/TranslationRear.x());
            SpeedAxesRear.setValue(SpeedJointRear.x(),0,0);

            break;

        //RearSpeed is given
        case coordinate::Rear:

            //Get latest Transforms
            Axes2Joint=TFBuffer_.lookupTransform("jointRear", "axesRear", ros::Time(0));
            Joint2Joint=TFBuffer_.lookupTransform("jointFront", "jointRear", ros::Time(0));
            Joint2Axes=TFBuffer_.lookupTransform("axesFront", "jointFront", ros::Time(0));

            //Get Data from the Messages, since the tf2::fromMsg-Function returns Linking-Errors with workaround
            SpeedAxesRear.setValue(cmdVelMsg.linear.x,cmdVelMsg.linear.y,cmdVelMsg.linear.z);
            OmegaRear.setValue(cmdVelMsg.angular.x,cmdVelMsg.angular.y,cmdVelMsg.angular.z);
            TranslationRear.setValue(-Axes2Joint.transform.translation.x, -Axes2Joint.transform.translation.y, -Axes2Joint.transform.translation.z);
            TranslationFront.setValue(-Joint2Axes.transform.translation.x, -Joint2Axes.transform.translation.y, -Joint2Axes.transform.translation.z);
            Rotation.setValue(Joint2Joint.transform.rotation.x,Joint2Joint.transform.rotation.y, Joint2Joint.transform.rotation.z, Joint2Joint.transform.rotation.w);
            
            if (abs(Rotation.getAngle()>M_PI/2))
            {
                retVal.Front.leftWheel=0;
                retVal.Front.rightWheel=0;
                retVal.Rear.leftWheel=0;
                retVal.Rear.rightWheel=0;
                return retVal;
            }
            
            //tf2::fromMsg(cmdVelMsg.linear, SpeedAxesRear);
            //tf2::fromMsg(cmdVelMsg.angular, OmegaRear);
            //tf2::fromMsg(Axes2Joint.transform.translation,TranslationRear);
            //tf2::fromMsg(Joint2Axes.transform.translation,TranslationFront);
            //tf2::fromMsg(Joint2Joint.transform.rotation, Rotation);

            //Calculate Speed Joint Front
            SpeedJointRear=SpeedAxesRear+OmegaRear.cross(TranslationRear);

            //Transform Speed in JointRear, because they have to move with the same Speed
            SpeedJointFront=SpeedJointRear.rotate(Rotation.getAxis(), Rotation.getAngle());

            //Calculate needed Speed and Omega for AxesRear, assuming that Z and Y for the Translation from the Joint to the Axes are zero
            OmegaFront.setValue(0,0,-SpeedJointFront.y()/TranslationFront.x());
            SpeedAxesFront.setValue(SpeedJointFront.x(),0,0);
            break;

        //Do not calculate when any other Frame is given
        default:
            throw new std::runtime_error("Only Front and Rear Frames are allowed for inverse kinematics");
        }
    }
    catch(tf2::TransformException &e)
    {
        ROS_ERROR("tf not connected! Can not calculate Transform");
        retVal.Front.leftWheel=0;
        retVal.Front.rightWheel=0;
        retVal.Rear.leftWheel=0;
        retVal.Rear.rightWheel=0;
        return retVal;
    }

    //Set Messages for further Calculation, since the toMsg returns LinkerErrors here with workaround
    FrontMsg.linear.x=SpeedAxesFront.x();
    FrontMsg.linear.y=SpeedAxesFront.y();
    FrontMsg.linear.z=SpeedAxesFront.z();

    FrontMsg.angular.x=OmegaFront.x();
    FrontMsg.angular.y=OmegaFront.y();
    FrontMsg.angular.z=OmegaFront.z();

    RearMsg.linear.x=SpeedAxesRear.x();
    RearMsg.linear.y=SpeedAxesRear.y();
    RearMsg.linear.z=SpeedAxesRear.z();

    RearMsg.angular.x=OmegaRear.x();
    RearMsg.angular.y=OmegaRear.y();
    RearMsg.angular.z=OmegaRear.z();
    //FrontMsg.linear=tf2::toMsg<tf2::Vector3, geometry_msgs::Vector3>(SpeedAxesFront);
    //FrontMsg.angular=tf2::toMsg<tf2::Vector3, geometry_msgs::Vector3>(OmegaFront);

    retVal.Front=frontDrive_.inverseKinematics(FrontMsg);
    retVal.Rear=rearDrive_.inverseKinematics(RearMsg);

    return retVal;
}

geometry_msgs::Pose2D kinematics::ArticulatedDrive::forwardKinematics(articulatedWheelSpeed WheelSpeed, ros::Time Timestamp)
{
    geometry_msgs::Pose2D FrontPose=frontDrive_.forwardKinematics(WheelSpeed.Front, Timestamp);
    geometry_msgs::Pose2D RearPose=rearDrive_.forwardKinematics(WheelSpeed.Rear, Timestamp);

    switch (Base_)
    {
        case coordinate::Front:
            return FrontPose;
            break;

        case coordinate::Rear:
            return RearPose;
            break;

        default:
            throw new std::runtime_error("Can not calculate forward kinematics for given Frame");
    }
}

void kinematics::ArticulatedDrive::setParam(double AxesLength, double WheelDiameter, coordinate Base)
{
    frontDrive_.setParam(AxesLength, WheelDiameter);
    rearDrive_.setParam(AxesLength, WheelDiameter);
    Base_=Base;
}


geometry_msgs::Pose2D kinematics::ArticulatedDrive::getActualPose(coordinate Frame)
{
    switch (Base_)
    {
        case coordinate::Front:
            return frontDrive_.getActualPose();
            break;
        case coordinate::Rear:
            return rearDrive_.getActualPose();
            break;
        default:
            throw new std::runtime_error("Can not get Pose for given Frame");
    }
}

geometry_msgs::Twist kinematics::ArticulatedDrive::getSpeed()
{
    switch (Base_)
    {
        case coordinate::Front:
            return frontDrive_.getSpeed();
            break;

        case coordinate::Rear:
            return rearDrive_.getSpeed();
            break;

        default:
            throw new std::runtime_error("Can not get Speed for given Frame");
    }
}
