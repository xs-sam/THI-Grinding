#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


static geometry_msgs::PoseStamped LINK2BASE_TF_MSG;
void poseBC(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  LINK2BASE_TF_MSG=*msg;
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"kuka_create_tf");
  ros::NodeHandle n;
  ros::Rate rate(10);

  tf::Transform tfLink2Base;
  tf::Transform tf1Tool2Link;
  tf::Transform tf2Tool2Link;

  tf::TransformBroadcaster broadcaster;
  ros::Subscriber subPose = n.subscribe("/link6/pose",1,poseBC);

  double Link2Base_x,Link2Base_y,Link2Base_z,Link2Base_a,Link2Base_b,Link2Base_c;
  double Tool2Link_x1,Tool2Link_y1,Tool2Link_z1,Tool2Link_a1,Tool2Link_b1,Tool2Link_c1;
  double Tool2Link_x2,Tool2Link_y2,Tool2Link_z2,Tool2Link_a2,Tool2Link_b2,Tool2Link_c2;


  n.getParam("link2base/x",Link2Base_x);
  n.getParam("link2base/y",Link2Base_y);
  n.getParam("link2base/z",Link2Base_z);
  n.getParam("link2base/a",Link2Base_a);
  n.getParam("link2base/b",Link2Base_b);
  n.getParam("link2base/c",Link2Base_c);


  n.getParam("tool2link/x1",Tool2Link_x1);
  n.getParam("tool2link/y1",Tool2Link_y1);
  n.getParam("tool2link/z1",Tool2Link_z1);
  n.getParam("tool2link/a1",Tool2Link_a1);
  n.getParam("tool2link/b1",Tool2Link_b1);
  n.getParam("tool2link/c1",Tool2Link_c1);

  n.getParam("tool2link/x2",Tool2Link_x2);
  n.getParam("tool2link/y2",Tool2Link_y2);
  n.getParam("tool2link/z2",Tool2Link_z2);
  n.getParam("tool2link/a2",Tool2Link_a2);
  n.getParam("tool2link/b2",Tool2Link_b2);
  n.getParam("tool2link/c2",Tool2Link_c2);



  ROS_INFO("\033[1;32m The tf relationship between camera_link and camera_color_optical_frame is established \033[0m");

  //初始化tfLink2Base
  tf::Quaternion Q_TF_MSG;
  Q_TF_MSG.setRPY(Link2Base_a*M_PI/180,Link2Base_b*M_PI/180,Link2Base_c*M_PI/180);
  LINK2BASE_TF_MSG.header.frame_id="link6";
  LINK2BASE_TF_MSG.header.stamp=ros::Time::now();
  LINK2BASE_TF_MSG.pose.position.x=Link2Base_x;
  LINK2BASE_TF_MSG.pose.position.y=Link2Base_y;
  LINK2BASE_TF_MSG.pose.position.z=Link2Base_z;
  LINK2BASE_TF_MSG.pose.orientation.x=Q_TF_MSG.x();
  LINK2BASE_TF_MSG.pose.orientation.y=Q_TF_MSG.y();
  LINK2BASE_TF_MSG.pose.orientation.z=Q_TF_MSG.z();
  LINK2BASE_TF_MSG.pose.orientation.w=Q_TF_MSG.w();



  //圆心点到link的tf
  tf::Quaternion Q_TF_TOOL1_2LINK;
  Q_TF_TOOL1_2LINK.setRPY(Tool2Link_a1*M_PI/180,Tool2Link_b1*M_PI/180,Tool2Link_c1*M_PI/180);
  tf::Vector3 Tool1_2LinkTranslation(Tool2Link_x1, Tool2Link_y1, Tool2Link_z1);
  tf::Quaternion Tool1_2LinkRrotation(Q_TF_TOOL1_2LINK.x(), Q_TF_TOOL1_2LINK.y(), Q_TF_TOOL1_2LINK.z(), Q_TF_TOOL1_2LINK.w());
  tf1Tool2Link.setOrigin(Tool1_2LinkTranslation);
  tf1Tool2Link.setRotation(Tool1_2LinkRrotation);

  //砂轮接触点到link的tf
  tf::Quaternion Q_TF_TOOL2_2LINK;
  Q_TF_TOOL2_2LINK.setRPY(Tool2Link_a2*M_PI/180,Tool2Link_b2*M_PI/180,Tool2Link_c2*M_PI/180);
  tf::Vector3 Tool2_2LinkTranslation(Tool2Link_x2, Tool2Link_y2, Tool2Link_z2);
  tf::Quaternion Tool2_2LinkRrotation(Q_TF_TOOL2_2LINK.x(), Q_TF_TOOL2_2LINK.y(), Q_TF_TOOL2_2LINK.z(), Q_TF_TOOL2_2LINK.w());
  tf2Tool2Link.setOrigin(Tool2_2LinkTranslation);
  tf2Tool2Link.setRotation(Tool2_2LinkRrotation);


  while (ros::ok())
  {
    tf::Quaternion qTfRotation;
    tf::quaternionMsgToTF(LINK2BASE_TF_MSG.pose.orientation,qTfRotation);
    tfLink2Base.setRotation(qTfRotation);
    tfLink2Base.setOrigin(tf::Vector3(LINK2BASE_TF_MSG.pose.position.x,LINK2BASE_TF_MSG.pose.position.y,LINK2BASE_TF_MSG.pose.position.z));

    broadcaster.sendTransform(tf::StampedTransform(tfLink2Base, ros::Time::now(), "base_link", "link6"));
    broadcaster.sendTransform(tf::StampedTransform(tf1Tool2Link, ros::Time::now(), "link6", "tool1"));
    broadcaster.sendTransform(tf::StampedTransform(tf2Tool2Link, ros::Time::now(), "link6", "tool2"));

    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
