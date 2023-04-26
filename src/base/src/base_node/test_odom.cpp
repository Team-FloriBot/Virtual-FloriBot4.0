#include <ros/ros.h>
#include <ros/time.h>
#include <base/Wheels.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TestOdom");
    ros::NodeHandle nh;
    ros::Publisher pub=  nh.advertise<base::Wheels>("engine/actualSpeed", 1);
    base::Wheels msg;
    msg.frontLeft=0.9;
    msg.frontRight=1.1;
    msg.rearLeft=1;
    msg.rearRight=1;

    while (ros::ok())
    {
        msg.header.stamp=ros::Time::now();
        pub.publish(msg);
        ros::spinOnce();
    }

    return 0;
}