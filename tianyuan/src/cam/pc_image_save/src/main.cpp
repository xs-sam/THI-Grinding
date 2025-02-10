#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pc_image_save/savePcAndImage.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <surfacehd100.h>


typedef pcl::PointXYZ PointT;


int pointCloudWrite(std::string fileName, std::string fileType,  pcl::PointCloud<PointT> cloud)
{
  if(fileType=="ply")
  {
    std::ofstream outFile(fileName+".ply");
    if (!outFile.is_open())
    {
      std::cerr << "Error opening "+fileName+" for writing." << std::endl;
      return -1;
    }
    outFile << "ply\n";
    outFile << "format ascii 1.0\n";
    outFile << "element vertex "<<cloud.size()<<"\n";
    outFile << "property float x\n";
    outFile << "property float y\n";
    outFile << "property float z\n";
    outFile << "end_header\n";

    for(unsigned int i=0;i<cloud.size();++i)
    {
      outFile <<cloud.points[i].x<<" "<<cloud.points[i].y<<" "<<cloud.points[i].z<<"\n";
    }
    outFile.close();
    return static_cast<int>(cloud.size());
  }
  else
  {
    return pcl::io::savePCDFileASCII(fileName+".pcd", cloud);
  }

}

bool getPointCloud( pcl::PointCloud<PointT>::Ptr &cloud)
{
  SurfaceHD100 cam;
  CAMERA_CODE code;
  if(CAMERA_SUCCESS != (code = cam.CameraInit()))
  {
    if(code == CAMERA_CONNECT)
      ROS_ERROR("Surface HD100 3D Camera init is fail ( connection fail )!");
    if(code == CAMERA_INFO)
      ROS_ERROR("Surface HD100 3D Camera init is fail ( get camera info )!");
    return false;
  }

  if(CAMERA_SUCCESS != (code = cam.StartDepth()))
  {
    if(code == CAMERA_DEPTH_INFO)
      ROS_ERROR("Surface HD100 3D Camera StartDepth is fail ( get camera depth info )!");
    if(code == CAMERA_DEPTH_START)
      ROS_ERROR("Surface HD100 3D Camera StartDepth is fail ( camera depth stream start )!");
    if(code == CAMERA_DEPTH_INTRINSIC)
      ROS_ERROR("Surface HD100 3D Camera StartDepth is fail ( get camera intrinsic )!");
    return false;
  }

  cam.SetCameraParam();

  if(CAMERA_SUCCESS != (code = cam.GetPointCloud(cloud)))
  {
    if(code == CAMERA_DEPTH_GETFRAME)
      ROS_ERROR("Surface HD100 3D Camera GetPointCloud is fail ( get depth frame fail )!");
    if(code == CAMERA_DEPTH_SCALE)
      ROS_ERROR("Surface HD100 3D Camera GetPointCloud is fail ( get depth scale )!");
    return false;
  }

  if(CAMERA_SUCCESS != (code = cam.StopCamera()))
  {

    if(code == CAMERA_STOP)
      ROS_ERROR("Surface HD100 3D Camera Stop is fail !");
    return false;
  }

  return true;
}

bool savePointCloud(pc_image_save::savePcAndImageRequest &req,pc_image_save::savePcAndImageResponse &res)
{
  if(!req.fileBaseName.empty())
  {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    if (!getPointCloud(cloud))
    {
      res.ok=false;
      return true;
    }
    if(pointCloudWrite(req.fileBaseName,req.PointCloudFileType,*cloud)<0)
    {
      res.ok=false;
      return true;
    }
    res.ok=true;
    return true;
  }
  else
  {
    ROS_WARN("Point Cloud File Name is Empty");
    res.ok=false;
    return true;
  }
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"point_cloud_save");
  ros::NodeHandle n;
  ros::ServiceServer ser;
  ser=n.advertiseService("/Thi/savePointCloudAndImage",savePointCloud);

  ros::spin();
  return 0;
}
