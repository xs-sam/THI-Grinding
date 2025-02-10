#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <kuka_tcp/kukaPoint.h>
static tf::StampedTransform  tf1Tool2Base;
static tf::StampedTransform  tf2Tool2Base;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"toolpose");
    ros::NodeHandle n;
    ros::Publisher pubPos=n.advertise<kuka_tcp::kukaPoint>("/kuka/toolVector/pose",1);
    tf::TransformListener listener;

    ros::Rate r(10);
    while (ros::ok())
    {
        double a,b,c;
        kuka_tcp::kukaPoint msg;
        try
        {
            listener.waitForTransform("base_link","tool2",  ros::Time(0), ros::Duration(10));
            listener.lookupTransform("base_link","tool2", ros::Time(0),tf2Tool2Base); //接触点到基坐标的变换
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            return -1;
        }
        try
        {
            listener.waitForTransform("base_link","tool1",  ros::Time(0), ros::Duration(10));
            listener.lookupTransform("base_link","tool1", ros::Time(0),tf1Tool2Base); //圆盘圆心到基坐标的变换
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            return -1;
        }

        tf::Vector3 T1= tf1Tool2Base.getOrigin();
        tf::Vector3 T2= tf2Tool2Base.getOrigin();

        tf::Vector3 v = T1-T2;
        v = v.normalized();
        tf::Vector3 X(1,0,0);
        tf::Vector3 Y(0,1,0);
        tf::Vector3 Z(0,0,1);
        tf::Vector3 vx(0,v.y(),v.z());
        tf::Vector3 vy(v.x(),0,v.z());
        tf::Vector3 vz(v.x(),v.y(),0);
        vx = vx.normalized();
        vy = vy.normalized();
        vz = vz.normalized();
        a = std::acos(Y.dot(vz));
        b = std::acos(X.dot(vy));
        c = std::acos(Z.dot(vx));

        msg.x=static_cast<float>(v.x());
        msg.y=static_cast<float>(v.y());
        msg.z=static_cast<float>(v.z());
        msg.a=static_cast<float>(a*180/M_PI);
        msg.b=static_cast<float>(b*180/M_PI);
        msg.c=static_cast<float>(c*180/M_PI);

        pubPos.publish(msg);
        r.sleep();
        ros::spinOnce();
    }
}
