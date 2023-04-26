#include <ros/ros.h>
#include <ros/time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <base/Angle.h>
#include <sensor_msgs/JointState.h>
#include <base_node/base_publisher.h>

//void AngleCallback(const base::Angle::ConstPtr& msg);
void AngleCallback(const sensor_msgs::JointState msg);
tf2_ros::TransformBroadcaster* TF;
geometry_msgs::TransformStamped TFAngleMsg;
// Wheels Message erstellen
base::Wheels Wheels_actual;
// Publisher fuer Actual Speed erstellen
ros::Publisher ActualSpeed;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "angle2tf");
    ros::NodeHandle Nh;
    TF = new tf2_ros::TransformBroadcaster();
    //ros::Subscriber AngleSubs=Nh.subscribe("/sensors/bodyAngle", 1, &AngleCallback);
    ros::Subscriber AngleSubs=Nh.subscribe("/floribot/joint_states", 1, &AngleCallback);

    // Publisher starten
    ActualSpeed=Nh.advertise<base::Wheels>("engine/actualSpeed", 1);

    ros::spin();

    return 0;
}

//void AngleCallback(const base::Angle::ConstPtr &msg)
void AngleCallback(const sensor_msgs::JointState msg)
{  
    if(msg.velocity.size() > 0){
        tf2::Quaternion q; 
        TFAngleMsg.header.seq=msg.header.seq;
        TFAngleMsg.child_frame_id="jointRear";
        TFAngleMsg.header.frame_id="jointFront";
        TFAngleMsg.header.stamp=ros::Time::now();

        //q.setRPY(0,0,msg->angle);
        q.setRPY(0,0,msg.position[0]);
        TFAngleMsg.transform.translation.x=0;
        TFAngleMsg.transform.translation.y=0;
        TFAngleMsg.transform.translation.z=0;

        TFAngleMsg.transform.rotation.x=q.x();
        TFAngleMsg.transform.rotation.y=q.y();
        TFAngleMsg.transform.rotation.z=q.z();
        TFAngleMsg.transform.rotation.w=q.w();

        TF->sendTransform(TFAngleMsg);

        // Zusammenbauen der Actual Speed Message
        // Header
        Wheels_actual.header.seq=msg.header.seq;
        Wheels_actual.header.stamp=ros::Time::now();

        //Body
        Wheels_actual.frontLeft=msg.velocity[1];
        Wheels_actual.frontRight=msg.velocity[2];
        Wheels_actual.rearLeft=msg.velocity[3];
        Wheels_actual.rearRight=msg.velocity[4];

        ActualSpeed.publish(Wheels_actual);
    }

}
