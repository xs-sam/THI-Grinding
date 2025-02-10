#include <fstream>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <pc_merge/mergePc.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/transforms.h>


static std::string UPPOSEFILE;
static std::string UPPCBASE;
static std::string UPSAVE;
static int UPPCNUM;

static std::string DOWNRPOSEFILE;
static std::string DOWNRPCBASE;
static std::string DOWNRSAVE;
static int DOWNRPCNUM;

static std::string DOWNLPOSEFILE;
static std::string DOWNLPCBASE;
static std::string DOWNLSAVE;
static int DOWNLPCNUM;

static std::string SAVE;
static Eigen::Matrix4f Rtcam2Tcp;
static float Tcp2Link[6];

Eigen::Matrix4f getRT(float x,float y,float z,float a,float b,float c)
{
  Eigen::Vector3f translation(x, y, z);

  float radiansX = a * static_cast<float>(M_PI)/ 180.0f;
  float radiansY = b * static_cast<float>(M_PI) / 180.0f;
  float radiansZ = c * static_cast<float>(M_PI) / 180.0f;

  Eigen::Matrix3f rotationX;
  rotationX << 1, 0, 0,
      0, cos(radiansX), -sin(radiansX),
      0, sin(radiansX), cos(radiansX);

  Eigen::Matrix3f rotationY;
  rotationY << cos(radiansY), 0, sin(radiansY),
      0, 1, 0,
      -sin(radiansY), 0, cos(radiansY);

  Eigen::Matrix3f rotationZ;
  rotationZ << cos(radiansZ), -sin(radiansZ), 0,
      sin(radiansZ), cos(radiansZ), 0,
      0, 0, 1;
  Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity();
  transformationMatrix.topLeftCorner(3, 3) = rotationZ * rotationY * rotationX;
  transformationMatrix.topRightCorner(3, 1) = translation;
  return transformationMatrix;
}

int getBasePc(std::string fileName,Eigen::Vector3f TPhoto,Eigen::Vector3f RPhoto,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

  if (pcl::io::loadPLYFile<pcl::PointXYZ>(fileName, *cloud) == -1)
  {
    ROS_ERROR("Don't open %s",fileName.c_str());
    return -1;
  }

  //  Eigen::Matrix4f Rtcam2Tcp = Eigen::Matrix4f::Identity();
  //  Rtcam2Tcp(0, 0) =   1.000f;  Rtcam2Tcp(0,1) =  0.002f; Rtcam2Tcp(0, 2) = 0.009f; Rtcam2Tcp(0, 3) = 360.375f;
  //  Rtcam2Tcp(1, 0) =  -0.009f;  Rtcam2Tcp(1,1) =  0.013f; Rtcam2Tcp(1, 2) = 1.000f;  Rtcam2Tcp(1, 3) = 220.821f;
  //  Rtcam2Tcp(2, 0) =   0.002f;  Rtcam2Tcp(2,1) = -1.000f; Rtcam2Tcp(2, 2) = 0.013f;  Rtcam2Tcp(2, 3) = -652.186f;
  //  Rtcam2Tcp(3, 0) =   0.000f;  Rtcam2Tcp(3,1) =  0.000f; Rtcam2Tcp(3, 2) = 0.000f;  Rtcam2Tcp(3, 3) = 1.000f;

  Eigen::Matrix4f RtLink2Base = getRT(TPhoto(0),TPhoto(1),TPhoto(2),RPhoto(0),RPhoto(1),RPhoto(2));

//  Eigen::Matrix4f RtTcp2Link = getRT(643.63f,-30.38f,387.04f,0,90,0);
  Eigen::Matrix4f RtTcp2Link = getRT(Tcp2Link[0],Tcp2Link[1],Tcp2Link[2],Tcp2Link[3],Tcp2Link[4],Tcp2Link[5]);

  Eigen::Matrix4f RtTcp2Base = RtLink2Base*RtTcp2Link;

  Eigen::Matrix4f RtCam2Base = RtTcp2Base * Rtcam2Tcp;

  pcl::transformPointCloud(*cloud, *cloud, RtCam2Base);

  return  0;
}

int readPhotoPoseTXT(std::string poseFile,std::vector<Eigen::Vector3f> &Ts,std::vector<Eigen::Vector3f> &Rs)
{

  std::ifstream file(poseFile);

  if (!file.is_open())
  {
    ROS_ERROR("Don't open %s",poseFile.c_str());
    return -1;
  }

  std::string line;
  while (getline(file, line))
  {
    std::stringstream ss(line);
    float num1, num2, num3, num4, num5, num6;
    ss >> num1 >> num2 >> num3 >> num4 >> num5 >> num6;

    Eigen::Vector3f t(num1,num2 ,num3);
    Eigen::Vector3f r(num4,num5 ,num6);

    Ts.push_back(t);
    Rs.push_back(r);
  }

  file.close();
  ROS_INFO("\033[1;32m Read PoseFile Successfully form %s \033[0m",poseFile.c_str());
  return 0;
}

int savePly(std::string fileName,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  std::ofstream outFile(fileName);
  if (!outFile.is_open())
  {
    ROS_ERROR("Don't open %s for writing.",fileName.c_str());
    return -1;
  }
  outFile << "ply\n";
  outFile << "format ascii 1.0\n";
  outFile << "element vertex "<<cloud->size()<<"\n";
  outFile << "property float x\n";
  outFile << "property float y\n";
  outFile << "property float z\n";
  outFile << "end_header\n";

  for(unsigned int i=0;i<cloud->size();++i)
  {
    outFile <<cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].z<<"\n";
  }
  outFile.close();
  return 0;
}


bool mergePointCloud(pc_merge::mergePcRequest &req,pc_merge::mergePcResponse &res)
{
  if(req.workpiece == 0)//上半件
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAll(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<Eigen::Vector3f> Ts,Rs;

    if(readPhotoPoseTXT(UPPCBASE+UPPOSEFILE,Ts,Rs)<0)
    {
      res.ok=false;
      return true;
    }

    int pcNum=(req.pcnum==-1)?UPPCNUM:req.pcnum;

    for (int i = 0;i<pcNum;i++)
    {
      ROS_INFO("\033[1;35m This is %d PointCloud File for UP. \033[0m",i+1);
      cloud->clear();
      std::string f = UPPCBASE+"PointCloud/"+std::to_string(i)+".ply";
      if(0>getBasePc(f,Ts[static_cast<unsigned long>(i)],Rs[static_cast<unsigned long>(i)],cloud))
      {
        res.ok=false;
        return true;
      }
      for (unsigned long j = 0;j<cloud->size();j++)
      {
        cloudAll->push_back(cloud->points[j]);
      }
    }

    std::string savweName=(req.saveName.empty())?UPSAVE:req.saveName;
    SAVE = UPPCBASE+savweName;
    if(savePly(SAVE,cloudAll)<0)
    {
      res.ok=false;
      return true;
    }

  }
  else if(req.workpiece == 1) //下右半件
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAll(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<Eigen::Vector3f> Ts,Rs;

    if(readPhotoPoseTXT(DOWNRPCBASE+DOWNRPOSEFILE,Ts,Rs)<0)
    {
      res.ok=false;
      return true;
    }

    int pcNum=(req.pcnum==-1)?DOWNRPCNUM:req.pcnum;
    for (int i = 0;i<pcNum;i++)
    {
      ROS_INFO("\033[1;38;5;208m This is %d PointCloud File for Down Right. \033[0m",i+1);
      cloud->clear();
      std::string f = DOWNRPCBASE+"PointCloud/"+std::to_string(i)+".ply";
      if(0>getBasePc(f,Ts[static_cast<unsigned long>(i)],Rs[static_cast<unsigned long>(i)],cloud))
      {
        res.ok=false;
        return true;
      }
      for (unsigned long j = 0;j<cloud->size();j++)
      {
        cloudAll->push_back(cloud->points[j]);
      }
    }

    std::string savweName=(req.saveName.empty())?DOWNRSAVE:req.saveName;
    SAVE = DOWNRPCBASE+savweName;
    if(savePly(SAVE,cloudAll)<0)
    {
      res.ok=false;
      return true;
    }

  }
  else if(req.workpiece == 2) //下左半件
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAll(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<Eigen::Vector3f> Ts,Rs;

    if(readPhotoPoseTXT(DOWNLPCBASE+DOWNLPOSEFILE,Ts,Rs)<0)
    {
      res.ok=false;
      return true;
    }

    int pcNum=(req.pcnum==-1)?DOWNLPCNUM:req.pcnum;
    for (int i = 0;i<pcNum;i++)
    {
      ROS_INFO("\033[1;38;5;208m This is %d PointCloud File for Down Left. \033[0m",i+1);
      cloud->clear();
      std::string f = DOWNLPCBASE+"PointCloud/"+std::to_string(i)+".ply";
      if(0>getBasePc(f,Ts[static_cast<unsigned long>(i)],Rs[static_cast<unsigned long>(i)],cloud))
      {
        res.ok=false;
        return true;
      }
      for (unsigned long j = 0;j<cloud->size();j++)
      {
        cloudAll->push_back(cloud->points[j]);
      }
    }

    std::string savweName=(req.saveName.empty())?DOWNLSAVE:req.saveName;
    SAVE = DOWNLPCBASE+savweName;
    if(savePly(SAVE,cloudAll)<0)
    {
      res.ok=false;
      return true;
    }

  }
  else
  {
    ROS_WARN("Work Piece is error!!!");
    res.ok = false;
    return true;
  }
  ROS_INFO("\033[1;32m PointCloud Merge Successfully! (Located %s) \033[0m",SAVE.c_str());
  res.ok=true;
  return true;
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"pc_merge");
  ros::NodeHandle n;


  n.getParam("upPc/base",UPPCBASE);
  n.getParam("upPc/pose",UPPOSEFILE);
  n.getParam("upPc/save",UPSAVE);
  n.getParam("upPc/pcNum",UPPCNUM);

  n.getParam("downRightPc/base",DOWNRPCBASE);
  n.getParam("downRightPc/pose",DOWNRPOSEFILE);
  n.getParam("downRightPc/save",DOWNRSAVE);
  n.getParam("downRightPc/pcNum",DOWNRPCNUM);

  n.getParam("downLeftPc/base",DOWNLPCBASE);
  n.getParam("downLeftPc/pose",DOWNLPOSEFILE);
  n.getParam("downLeftPc/save",DOWNLSAVE);
  n.getParam("downLeftPc/pcNum",DOWNLPCNUM);

  n.getParam("tcp2link/x",Tcp2Link[0]);
  n.getParam("tcp2link/y",Tcp2Link[1]);
  n.getParam("tcp2link/z",Tcp2Link[2]);
  n.getParam("tcp2link/a",Tcp2Link[3]);
  n.getParam("tcp2link/b",Tcp2Link[4]);
  n.getParam("tcp2link/c",Tcp2Link[5]);


  Rtcam2Tcp = Eigen::Matrix4f::Identity();

  for (int i=0;i<3;i++)
  {
    for (int j=0;j<4;j++)
    {
      float data;
      n.getParam("cam2tcp/R"+std::to_string(i)+std::to_string(j),data);
      Rtcam2Tcp(i,j)=data;
    }
  }
  Rtcam2Tcp(3,0)=0.000f;
  Rtcam2Tcp(3,1)=0.000f;
  Rtcam2Tcp(3,2)=0.000f;
  Rtcam2Tcp(3,3)=1.000f;


  ros::ServiceServer ser;
  ser=n.advertiseService("/Thi/mergePointCloud",mergePointCloud);

  ros::spin();
  return 0;

}
