#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <kuka_tcp/kukaPoint.h>
#include <tf/tf.h>

static ros::Publisher pubKukaConversionPose;

void kukaPoseBC(const kuka_tcp::kukaPoint::ConstPtr &msg)
{
  tf2::Quaternion q;
  double link2base[6];
  geometry_msgs::PoseStamped msgPose;

  link2base[0]=static_cast<double>(msg->x/1000);
  link2base[1]=static_cast<double>(msg->y/1000);
  link2base[2]=static_cast<double>(msg->z/1000);
  link2base[3]=static_cast<double>(msg->c)*M_PI/180;
  link2base[4]=static_cast<double>(msg->b)*M_PI/180;
  link2base[5]=static_cast<double>(msg->a)*M_PI/180;

  q.setRPY(link2base[3],link2base[4],link2base[5]);

  msgPose.header.stamp=ros::Time::now();
  msgPose.header.frame_id="link6";
  msgPose.pose.position.x=link2base[0];
  msgPose.pose.position.y=link2base[1];
  msgPose.pose.position.z=link2base[2];
  msgPose.pose.orientation.w=q.w();
  msgPose.pose.orientation.x=q.x();
  msgPose.pose.orientation.y=q.y();
  msgPose.pose.orientation.z=q.z();

  pubKukaConversionPose.publish(msgPose);
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"kuka_pose_conversion");
  ros::NodeHandle n;
  pubKukaConversionPose=n.advertise<geometry_msgs::PoseStamped>("/link6/pose",1);
  ros::Subscriber subKukaPose= n.subscribe("/kuka/link6/pose",1,kukaPoseBC);
#if 0 //是否为给值调试方式
  ros::Rate r(10);
  int mn=0,nn=0,qn=0;
  while (ros::ok())
  {
    tf2::Quaternion q;
    q.setRPY(1.57,0,3.14);
    geometry_msgs::PoseStamped msgPose;
    msgPose.header.stamp=ros::Time::now();
    msgPose.header.frame_id="link6";
    msgPose.pose.position.x=nn++;
    msgPose.pose.position.y=mn++;
    msgPose.pose.position.z=qn++;
    msgPose.pose.orientation.w=q.w();
    msgPose.pose.orientation.x=q.x();
    msgPose.pose.orientation.y=q.y();
    msgPose.pose.orientation.z=q.z();

    pubKukaConversionPose.publish(msgPose);

    r.sleep();
    ros::spinOnce();
  }
#else
  ros::spin();
  return 0;
#endif

}
