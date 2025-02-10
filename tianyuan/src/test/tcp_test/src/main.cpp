#include <ros/ros.h>
#include <kuka_tcp/kukaTrack.h>
int main(int argc,char** argv)
{
    ros::init(argc,argv,"tcp_test");
    ros::NodeHandle n;
    ros::ServiceClient ser = n.serviceClient<kuka_tcp::kukaTrack>("/kuka/track");

    kuka_tcp::kukaTrack srv;
    srv.request.mod = 2;
    srv.request.speed = 30;
    kuka_tcp::kukaPoint p;
    for (int i =0;i<20;i++)
    {
        p.x = i+1;
        p.y = -90;
        p.z = 90;
        p.a = 0;
        p.b = 0;
        p.c = 90;
        srv.request.track.push_back(p);
    }

    if(ser.call(srv))
    {
        ROS_INFO("OK");
    }

    ros::spin();
}
