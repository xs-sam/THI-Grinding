#include <ros/ros.h>
#include <thi_vision/visionTracks.h>
#include <thivision.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <locale>

static tf::StampedTransform  tf1Tool2Tool2;
static tf::StampedTransform  tf2Tool2Link;
static tf::StampedTransform  tf1Tool2Link;

static double FIRSTZ1,FIRSTZ2,LASTZ1,LASTZ2;
static double DownFIRSTZ[3],DownLASTZ[3];
static float groundZ,baffleX;//定义了地面的z向高度值和挡板的x向值(基于base_link)

bool floatIsEqual(float a, float b)
{
    return (std::fabs(a-b)<1e-10f)?true:false;
}

void calculateRotationQuaternion(const tf2::Vector3& a_x, const tf2::Vector3& a_y, const tf2::Vector3& a_z,
                                 const tf2::Vector3& b_x, const tf2::Vector3& b_y, const tf2::Vector3& b_z,
                                 tf2::Quaternion& quaternion)
{
    // 创建 a 坐标系的旋转矩阵
    tf2::Matrix3x3 a_matrix;
    a_matrix.setValue(a_x.x(), a_y.x(), a_z.x(),
                      a_x.y(), a_y.y(), a_z.y(),
                      a_x.z(), a_y.z(), a_z.z());

    // 创建 b 坐标系的旋转矩阵
    tf2::Matrix3x3 b_matrix;
    b_matrix.setValue(b_x.x(), b_y.x(), b_z.x(),
                      b_x.y(), b_y.y(), b_z.y(),
                      b_x.z(), b_y.z(), b_z.z());

    // 计算从 a 坐标系到 b 坐标系的旋转矩阵
    tf2::Matrix3x3 rotation_matrix_final = b_matrix * a_matrix.inverse();

    // 从旋转矩阵获取旋转四元数
    rotation_matrix_final.getRotation(quaternion);
}

tf::Vector3 getAngle2Vector(double a,double b,double c)
{
    tf::Transform  Link2Base;
    tf::Quaternion q;
    q.setRPY(c*M_PI/180,b*M_PI/180,a*M_PI/180);
    Link2Base.setOrigin(tf::Vector3(0,0,0));
    Link2Base.setRotation(q);

    tf::Transform  Tool12Base,Tool22Base;
    Tool12Base = Link2Base*tf1Tool2Link;
    Tool22Base = Link2Base*tf2Tool2Link;

    return (Tool12Base.getOrigin()-Tool22Base.getOrigin()).normalized();
}
tf::Quaternion getVectorAngle3D(tf::Vector3 VTool, tf::Vector3 VPlane, tf::Vector3 VAix)
{
    tf::Quaternion QTool2Plane,QY2Aix,R;
    VTool.normalize();
    VPlane.normalize();
    VAix.normalize();
    tf::Vector3 VY(0,1,0);
    if(VAix.x()<0)
        VAix=-VAix;

    tf::Vector3 Y1 = VTool.cross(VY);
    tf::Vector3 Y2 = VPlane.cross(VAix);

    tf::Vector3 Z1 = VTool.cross(Y1);
    tf::Vector3 Z2 = VPlane.cross(Y2);

    tf2::Vector3 x1(VTool.x(),VTool.y(),VTool.z());
    tf2::Vector3 x2(VPlane.x(),VPlane.y(),VPlane.z());

    tf2::Vector3 y1(Y1.x(),Y1.y(),Y1.z());
    tf2::Vector3 y2(Y2.x(),Y2.y(),Y2.z());

    tf2::Vector3 z1(Z1.x(),Z1.y(),Z1.z());
    tf2::Vector3 z2(Z2.x(),Z2.y(),Z2.z());

    tf2::Quaternion ret;

    calculateRotationQuaternion(x1,y1,z1,x2,y2,z2,ret);

    R.setX(ret.x());
    R.setY(ret.y());
    R.setZ(ret.z());
    R.setW(ret.w());

    return R;
}

void showInfo(thi_vision::visionTracksRequest &req)
{
    //设置为中文输出
    std::locale::global(std::locale(""));
    std::cout.imbue(std::locale());
    //

    ROS_INFO("\033[1;32m Start visual calculation \033[0m");

    std::string workpieceY = "\033[1;33m 工件类型: \033[0m";
    std::string workpieceB ;

    if(req.workpiece == 0)
    {
        workpieceB = "\033[1;34m 上半件 \033[0m";
    }
    else if(req.workpiece == 1)
    {
       workpieceB = "\033[1;34m 下右半件 \033[0m";
    }
    else if(req.workpiece == 2)
    {
       workpieceB = "\033[1;34m 下左半件 \033[0m";
    }
    else
    {
       workpieceB = "\033[1;34m 不存在 \033[0m";
    }

    ROS_INFO("%s%s", workpieceY.c_str(), workpieceB.c_str());

    std::string pointCloudFileY = "\033[1;33m 点云路径: \033[0m";
    std::string pointCloudFileB="\033[1;34m"+req.pointCloudFile+" \033[0m";
    ROS_INFO("%s%s", pointCloudFileY.c_str(), pointCloudFileB.c_str());

    std::string fileTypeY = "\033[1;33m 点云类型: \033[0m";
    std::string fileTypeB="\033[1;34m"+req.fileType+" \033[0m";
    ROS_INFO("%s%s", fileTypeY.c_str(), fileTypeB.c_str());

    std::string lengthY = "\033[1;33m 打磨长度: \033[0m";
    std::string lengthB="\033[1;34m"+std::to_string(req.length)+" mm"+ "\033[0m";
    ROS_INFO("%s%s", lengthY.c_str(), lengthB.c_str());

    std::string grooveWidthY = "\033[1;33m 槽宽长度: \033[0m";
    std::string grooveWidthB="\033[1;34m"+std::to_string(req.grooveWidth)+" mm"+" \033[0m";
    ROS_INFO("%s%s", grooveWidthY.c_str(), grooveWidthB.c_str());

    std::string widthY = "\033[1;33m 道宽长度: \033[0m";
    std::string widthB="\033[1;34m"+std::to_string(req.width)+" mm"+" \033[0m";
    ROS_INFO("%s%s", widthY.c_str(), widthB.c_str());

    std::string pressY = "\033[1;33m 下压长度: \033[0m";
    std::string pressB="\033[1;34m"+std::to_string(req.press)+" mm"+" \033[0m";
    ROS_INFO("%s%s", pressY.c_str(), pressB.c_str());

    std::string avoidRightY = "\033[1;33m 右侧避障距离: \033[0m";
    std::string avoidRightB="\033[1;34m"+std::to_string(req.avoidRight)+" mm"+" \033[0m";
    ROS_INFO("%s%s", avoidRightY.c_str(), avoidRightB.c_str());

    std::string avoidLeftY = "\033[1;33m 左侧避障距离: \033[0m";
    std::string avoidLeftB="\033[1;34m"+std::to_string(req.avoidLeft)+" mm"+" \033[0m";
    ROS_INFO("%s%s", avoidLeftY.c_str(), avoidLeftB.c_str());

    std::string angleRightY = "\033[1;33m 右侧避障角度(a,b,c): \033[0m";
    std::string angleRightB="\033[1;34m"+std::to_string(req.angleRight.a)+"° "+std::to_string(req.angleRight.b)+"° "+std::to_string(req.angleRight.c)+"° "+" \033[0m";
    ROS_INFO("%s%s", angleRightY.c_str(), angleRightB.c_str());

    std::string angleLeftY = "\033[1;33m 左侧避障角度(a,b,c): \033[0m";
    std::string angleLeftB="\033[1;34m"+std::to_string(req.angleLeft.a)+"° "+std::to_string(req.angleLeft.b)+"° "+std::to_string(req.angleLeft.c)+"° "+" \033[0m";
    ROS_INFO("%s%s", angleLeftY.c_str(), angleLeftB.c_str());



}
bool visionTracksBC(thi_vision::visionTracksRequest &req,thi_vision::visionTracksResponse &res)
{
    showInfo(req);
    // 获取程序开始的时间
    auto start = std::chrono::high_resolution_clock::now();
    if(req.workpiece == 0) //上半件
    {
        ThiVision v;
        tf::Vector3 VAix;
        PointT p1,p2;
        std::vector<PointT> points1,points2;
        std::vector<PointT> vector1,vector2;
        std::vector<pcl::PointCloud<PointT>::Ptr> wordAreaPoints;
        std::vector<std::vector<PointT>>listOutPoints,listVectors;
        pcl::PointIndices::Ptr  lineInliers(new pcl::PointIndices);
        pcl::PointIndices::Ptr  planeInliers(new pcl::PointIndices);
        pcl::PointIndices::Ptr  cylinderInliers(new pcl::PointIndices);
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        std::vector<pcl::ModelCoefficients::Ptr> PlaneCylinderLineList; //圆柱与承压台的交线(2条)
        pcl::ModelCoefficients::Ptr sectionPlane(new pcl::ModelCoefficients);
        std::vector<PointT> trackPoints,trackPoints1,trackVector1,trackPoints2,trackVector2;
        pcl::ModelCoefficients::Ptr lineCoefficients(new pcl::ModelCoefficients);
        pcl::ModelCoefficients::Ptr planeCoefficients(new pcl::ModelCoefficients);
        pcl::ModelCoefficients::Ptr cylinderCoefficients(new pcl::ModelCoefficients);
        pcl::ModelCoefficients::Ptr sectionLineCoefficients(new pcl::ModelCoefficients); //截面直线(基础线)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProjection(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmentation(new pcl::PointCloud<pcl::PointXYZ>);

        //读取文件
        READ_STATE readReturn;
        if(req.fileType=="ply")
        {
            readReturn = v.readPointCloudFile(req.pointCloudFile,FILE_TYPE::PLY,cloud);
        }
        else if(req.fileType=="pcd")
        {
            readReturn = v.readPointCloudFile(req.pointCloudFile,FILE_TYPE::PCD,cloud);
        }
        else
        {
            readReturn = v.readPointCloudFile(req.pointCloudFile,FILE_TYPE::TXT,cloud);
        }
        if(readReturn!=READ_STATE::READ_SUCCEED)
        {
            res.ok=false;
            res.tracks.clear();
            return true;
        }

        //预处理
        PointT minP,maxP,maxCutP,minCutP;
        v.voxelPointCloud(cloud,cloud,Eigen::Vector3f(10,10,10));
        v.getPointCloudMinMax3D(cloud,minP,maxP);

        //分割出主题工作区域
        PointT pWorkMin(baffleX,minP.y,groundZ);
        PointT pWorkMax(maxP.x,maxP.y,maxP.z);
        v.cutPointCloud(cloud,cloud,pWorkMin,pWorkMax);

        v.statisticalPointCloud(cloud,cloud,30,1.0);
        v.copyPointCloud(cloud,cloudSegmentation);

        //v.segmentationCylinder(cloudSegmentation,50,1000,0.01,10,0,1000,cylinderInliers,cylinderCoefficients);
        v.segmentationCylinder(cloudSegmentation,100,1000,0.01,1,0,1000,cylinderInliers,cylinderCoefficients);
        v.leaveIndexesPoint(cloudSegmentation,cloudProjection,cylinderInliers);
        v.removeIndexesPoint(cloudSegmentation,cloudSegmentation,cylinderInliers);
        v.segmentationPlane(cloudSegmentation,1000,1.0,planeInliers,planeCoefficients);


        v.planarProjectionPointCloud(cloudProjection,cloudProjection,planeCoefficients);
        v.getPlanarConcaveHull(cloudProjection,cloudProjection,12);
        v.statisticalPointCloud(cloudProjection,cloudProjection,30,1.0);
        v.getPointCloudMinMax3D(cloudProjection,minP,maxP);
        minCutP=minP;
        maxCutP.x=minP.x+50;
        maxCutP.y=maxP.y;
        maxCutP.z=maxP.z;
        v.cutPointCloud(cloudProjection,cloudProjection,minCutP,maxCutP);
        v.segmentationLine(cloudProjection,1000,1.0,lineInliers,lineCoefficients);
        v.createSectionPlane(lineCoefficients,cylinderCoefficients,req.grooveWidth,sectionPlane);
        v.getPlanePlaneLine(sectionPlane,planeCoefficients,sectionLineCoefficients);
        v.getPlaneCylinderLine(planeCoefficients,cylinderCoefficients,PlaneCylinderLineList);

        v.getLineLineIntersectPoint(sectionLineCoefficients,PlaneCylinderLineList[0],p1);
        v.getLineLineIntersectPoint(sectionLineCoefficients,PlaneCylinderLineList[1],p2);


        //法兰盘姿态转化为向量
        tf::Vector3 avoidVectorLeft = getAngle2Vector(static_cast<double>(req.angleLeft.a),static_cast<double>(req.angleLeft.b),static_cast<double>(req.angleLeft.c));
        tf::Vector3 avoidVectorRight = getAngle2Vector(static_cast<double>(req.angleRight.a),static_cast<double>(req.angleRight.b),static_cast<double>(req.angleRight.c));
        //生成基础点
        v.getArcDividePoint(p1,p2,cylinderCoefficients,req.width,req.avoidLeft,req.avoidRight,
                            Eigen::Vector3f{static_cast<float>(avoidVectorLeft.x()),static_cast<float>(avoidVectorLeft.y()),static_cast<float>(avoidVectorLeft.z())},
                            Eigen::Vector3f{static_cast<float>(avoidVectorRight.x()),static_cast<float>(avoidVectorRight.y()),static_cast<float>(avoidVectorRight.z())},
                            points1,vector1);


        //铸造字区域路径算法

        bool flag=false;
        for (unsigned long i=0;i<req.listPcAreas.size();i++)
        {
            pcl::PointCloud<PointT>::Ptr word(new pcl::PointCloud<PointT>);
            for (unsigned long j=0;j<req.listPcAreas[i].listPcPoint.size();j++)
            {
                PointT p;
                p.x=req.listPcAreas[i].listPcPoint[j].x;
                p.y=req.listPcAreas[i].listPcPoint[j].y;
                p.z=req.listPcAreas[i].listPcPoint[j].z;
                word->push_back(p);
            }
            //平面投影
            v.planarProjectionPointCloud(word,word,planeCoefficients);

            //排序 整理点云
            v.sortAreaPointCloud(word);
            wordAreaPoints.push_back(word);
        }


        v.getTrackKeyPoints(wordAreaPoints,cylinderCoefficients,sectionLineCoefficients,req.length,points1,vector1,listOutPoints,listVectors);

        //出路径
        v.tidyKeyPoints(points1,vector1,listOutPoints[0],listVectors[0],trackPoints,trackVector1);
        v.pressTrackPoints(trackPoints,trackVector1,req.press,trackPoints1);
        if(listOutPoints.size()!=1)
        {
            trackPoints.clear();
            v.tidyKeyPoints(listOutPoints[1],listVectors[1],listOutPoints[2],listVectors[2],trackPoints,trackVector2);
            v.pressTrackPoints(trackPoints,trackVector2,req.press,trackPoints2);
            flag=true;
        }

        //单位转换mm到m
        for(unsigned long i=0;i<trackPoints1.size();i++)
        {
            trackPoints1[i].x=trackPoints1[i].x/1000;
            trackPoints1[i].y=trackPoints1[i].y/1000;
            trackPoints1[i].z=trackPoints1[i].z/1000;
        }
        if(flag)
        {
            for(unsigned long i=0;i<trackPoints2.size();i++)
            {
                trackPoints2[i].x=trackPoints2[i].x/1000;
                trackPoints2[i].y=trackPoints2[i].y/1000;
                trackPoints2[i].z=trackPoints2[i].z/1000;
            }
        }

        //工具转化
        tf::Transform tool2Base,link2Base,tool2Link;
        tf::Vector3 VectorTool= tf1Tool2Tool2.getOrigin();
        tool2Link.setOrigin(tf2Tool2Link.getOrigin());
        tool2Link.setRotation(tf2Tool2Link.getRotation());

        thi_vision::track track1,track2;
        VAix.setX(static_cast<double>(cylinderCoefficients->values[3]));
        VAix.setY(static_cast<double>(cylinderCoefficients->values[4]));
        VAix.setZ(static_cast<double>(cylinderCoefficients->values[5]));

        for (unsigned long i =0;i<trackPoints1.size();i++)
        {
            thi_vision::pose pose;
            tool2Base.setOrigin(tf::Vector3(static_cast<double>(trackPoints1[i].x),static_cast<double>(trackPoints1[i].y),static_cast<double>(trackPoints1[i].z)));
            tool2Base.setRotation(getVectorAngle3D(VectorTool,tf::Vector3(static_cast<double>(trackVector1[i].x),static_cast<double>(trackVector1[i].y),static_cast<double>(trackVector1[i].z)),VAix));
            link2Base=tool2Base*tool2Link.inverse();

            double roll, pitch, yaw;
            tf::Matrix3x3(link2Base.getRotation()).getRPY(roll,pitch,yaw);
            if(i==0)//第一个点的话 增加z方向抬高的落刀点
            {
                pose.point.x=static_cast<float>(link2Base.getOrigin().getX())*1000.0f;
                pose.point.y=static_cast<float>(link2Base.getOrigin().getY())*1000.0f;
                pose.point.z=static_cast<float>(link2Base.getOrigin().getZ()+FIRSTZ1)*1000.0f;
                pose.angle.a=static_cast<float>(yaw*180/M_PI);
                pose.angle.b=static_cast<float>(pitch*180/M_PI);
                pose.angle.c=static_cast<float>(roll*180/M_PI);
                track1.track.push_back(pose);
            }

            pose.point.x=static_cast<float>(link2Base.getOrigin().getX())*1000.0f;
            pose.point.y=static_cast<float>(link2Base.getOrigin().getY())*1000.0f;
            pose.point.z=static_cast<float>(link2Base.getOrigin().getZ())*1000.0f;
            pose.angle.a=static_cast<float>(yaw*180/M_PI);
            pose.angle.b=static_cast<float>(pitch*180/M_PI);
            pose.angle.c=static_cast<float>(roll*180/M_PI);
            track1.track.push_back(pose);
            if(i==trackPoints1.size()-1) //最后一个点的话 增加z方向抬高的起刀点
            {
                pose.point.x=static_cast<float>(link2Base.getOrigin().getX())*1000.0f;
                pose.point.y=static_cast<float>(link2Base.getOrigin().getY())*1000.0f;
                pose.point.z=static_cast<float>(link2Base.getOrigin().getZ()+LASTZ1)*1000.0f;
                pose.angle.a=static_cast<float>(yaw*180/M_PI);
                pose.angle.b=static_cast<float>(pitch*180/M_PI);
                pose.angle.c=static_cast<float>(roll*180/M_PI);


                track1.track.push_back(pose);
            }
        }
        res.tracks.push_back(track1);

        if(flag)
        {
            for (unsigned long i =0;i<trackPoints2.size();i++)
            {
                thi_vision::pose pose;
                tool2Base.setOrigin(tf::Vector3(static_cast<double>(trackPoints2[i].x),static_cast<double>(trackPoints2[i].y),static_cast<double>(trackPoints2[i].z)));
                tool2Base.setRotation(getVectorAngle3D(VectorTool,tf::Vector3(static_cast<double>(trackVector2[i].x),static_cast<double>(trackVector2[i].y),static_cast<double>(trackVector2[i].z)),VAix));
                link2Base=tool2Base*tool2Link.inverse();

                double roll, pitch, yaw;
                tf::Matrix3x3(link2Base.getRotation()).getRPY(roll,pitch,yaw);

                if(i==0)//第一个点的话 增加z方向抬高的落刀点
                {
                    pose.point.x=static_cast<float>(link2Base.getOrigin().getX())*1000.0f;
                    pose.point.y=static_cast<float>(link2Base.getOrigin().getY())*1000.0f;
                    pose.point.z=static_cast<float>(link2Base.getOrigin().getZ()+FIRSTZ2)*1000.0f;
                    pose.angle.a=static_cast<float>(yaw*180/M_PI);
                    pose.angle.b=static_cast<float>(pitch*180/M_PI);
                    pose.angle.c=static_cast<float>(roll*180/M_PI);
                    track2.track.push_back(pose);
                }

                pose.point.x=static_cast<float>(link2Base.getOrigin().getX())*1000.0f;
                pose.point.y=static_cast<float>(link2Base.getOrigin().getY())*1000.0f;
                pose.point.z=static_cast<float>(link2Base.getOrigin().getZ())*1000.0f;
                pose.angle.a=static_cast<float>(yaw*180/M_PI);
                pose.angle.b=static_cast<float>(pitch*180/M_PI);
                pose.angle.c=static_cast<float>(roll*180/M_PI);
                track2.track.push_back(pose);

                if(i==trackPoints2.size()-1)
                {
                    pose.point.x=static_cast<float>(link2Base.getOrigin().getX())*1000.0f;
                    pose.point.y=static_cast<float>(link2Base.getOrigin().getY())*1000.0f;
                    pose.point.z=static_cast<float>(link2Base.getOrigin().getZ()+LASTZ2)*1000.0f;
                    pose.angle.a=static_cast<float>(yaw*180/M_PI);
                    pose.angle.b=static_cast<float>(pitch*180/M_PI);
                    pose.angle.c=static_cast<float>(roll*180/M_PI);
                    track2.track.push_back(pose);
                }
            }
            res.tracks.push_back(track2);
        }

    }
    else if(req.workpiece == 1) //下半件 右半
    {
        Curve C;

        tf::Vector3 VAix;
        ThiVision v;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::ModelCoefficients::Ptr planeFacade(new pcl::ModelCoefficients);
        pcl::ModelCoefficients::Ptr planeSection(new pcl::ModelCoefficients);
        pcl::ModelCoefficients::Ptr planePressure(new pcl::ModelCoefficients);
        pcl::ModelCoefficients::Ptr planeSectionUp(new pcl::ModelCoefficients);
        pcl::ModelCoefficients::Ptr planeSectionDown(new pcl::ModelCoefficients);

        pcl::ModelCoefficients::Ptr lineSection(new pcl::ModelCoefficients);

        pcl::PointCloud<pcl::PointXYZ>::Ptr in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);


        //读取文件
        READ_STATE readReturn;
        if(req.fileType=="ply")
        {
            readReturn = v.readPointCloudFile(req.pointCloudFile,FILE_TYPE::PLY,in);
        }
        else if(req.fileType=="pcd")
        {
            readReturn = v.readPointCloudFile(req.pointCloudFile,FILE_TYPE::PCD,in);
        }
        else
        {
            readReturn = v.readPointCloudFile(req.pointCloudFile,FILE_TYPE::TXT,in);
        }
        if(readReturn!=READ_STATE::READ_SUCCEED)
        {
            res.ok=false;
            res.tracks.clear();
            return true;
        }


        //分割点云平面
        //1、获取点云上下限
        PointT minP,maxP;
        v.getPointCloudMinMax3D(in,minP,maxP);

        //2、分割出主题工作区域
        PointT pWorkMin(baffleX,minP.y,groundZ);
        PointT pWorkMax(maxP.x,maxP.y,maxP.z);
        v.cutPointCloud(in,in,pWorkMin,pWorkMax);

        //3、提取立面
        float hFacadeMin = 140;//最小的立面高度
        v.getPointCloudMinMax3D(in,minP,maxP);
        PointT pFacadeMin(minP.x,minP.y,minP.z);
        PointT pFacadeMax(maxP.x,maxP.y,minP.z+hFacadeMin);
        v.cutPointCloud(in,out,pFacadeMin,pFacadeMax);
        v.segmentationPlane(out,1000,1.0,inliers,planeFacade);

        //3.1 保留立面边界点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr facade(new pcl::PointCloud<pcl::PointXYZ>);
        v.leaveIndexesPoint(out,facade,inliers);
        v.planarProjectionPointCloud(facade,facade,planeFacade);
        v.getPointCloudMinMax3D(facade,minP,maxP);
        maxP.x=minP.x+10;
        v.cutPointCloud(facade,facade,minP,maxP);
        //3.2 得到截面直线
        v.segmentationLine(facade,1000,1.0,inliers,lineSection);

        //4、提取承压台
        v.passThroughForPlane(planeFacade,in,80,180,out);
        v.segmentationPlane(out,1000,1.0,inliers,planePressure);

        //5、切出主体
        v.getPointCloudMinMax3D(in,minP,maxP);
        v.passThroughForPlane(planeFacade,in,80,maxP.y-minP.y,out);
        v.passThroughForPlane(planePressure,out,10,maxP.z-minP.z,out);

        v.statisticalPointCloud(out,out,150,1.0); //这里通过滤波实现了对主体的提取
        v.getPlaneUpOrDownPointCloud(planePressure,out,in,Eigen::Vector3f(0,0,1),true); //保留平面之上的点云


        //6、构建基础截面
        Eigen::Vector3f vFacade(planeFacade->values[0],planeFacade->values[1],planeFacade->values[2]),
                vPressure(planePressure->values[0],planePressure->values[1],planePressure->values[2]);
        Eigen::Vector3f vSection = vFacade.cross(vPressure).normalized();
        if (vSection(0)<0)
            vSection=-vSection;
        v.createSectionPlane(lineSection,vSection,planeSection);
        //7、通过两个基础面来夹取点云
        //7.1夹取平面
        v.movePlaneByAix(planeSection,planeSectionDown,Eigen::Vector3f(1,0,0),5);//35
        v.getPlaneUpOrDownPointCloud(planeSectionDown,in,out,Eigen::Vector3f(1,0,0),true);
        v.movePlaneByAix(planeSection,planeSectionUp,Eigen::Vector3f(1,0,0),35);//55
        v.getPlaneUpOrDownPointCloud(planeSectionUp,out,out,Eigen::Vector3f(1,0,0),false);
        //7.2投影到平面
        v.planarProjectionPointCloud(out,out,planeSectionDown);


        //8、提取表面点
        v.getSurfacePointCloud(out,out,lineSection,vPressure,1,true);

        //9、去除耳板折线点云
        PointT pEar1(req.listPcAreas[0].listPcPoint[0].x,req.listPcAreas[0].listPcPoint[0].y,req.listPcAreas[0].listPcPoint[0].z);
        PointT pEar2(req.listPcAreas[0].listPcPoint[1].x,req.listPcAreas[0].listPcPoint[1].y,req.listPcAreas[0].listPcPoint[1].z);
        pcl::ModelCoefficients::Ptr lineCutEar(new pcl::ModelCoefficients);
        v.retainValidCurvePointCloud(out,planeFacade,pEar1,pEar2,out,lineCutEar);
        Eigen::Vector3f vLineCutEar(lineCutEar->values[3],lineCutEar->values[4],lineCutEar->values[5]);
        vLineCutEar = vLineCutEar.normalized();
        float kLineCutEar = vLineCutEar(2)/vLineCutEar(0);
        bool kIsPlus = (kLineCutEar>0)?true:false;

        //10、折线化曲线点云
        v.getCurveInfo(out,20,vSection,C);

        //11、用道宽 生成点位
        //11.1 重设上半避障距离
        float avoidUp =req.avoidLeft;
        v.resetAvoidUp(pEar1,pEar2,planeFacade,vSection,avoidUp,avoidUp,kIsPlus);

        //11.2 重设下半避障距离
        float avoidDown = req.avoidRight;
        v.resetAvoidDown(C,req.width,planeSection,avoidDown,avoidDown,true);

        //11.3 法兰盘姿态转化为向量
        tf::Vector3 avoidVectorLeft = getAngle2Vector(static_cast<double>(req.angleLeft.a),static_cast<double>(req.angleLeft.b),static_cast<double>(req.angleLeft.c));
        tf::Vector3 avoidVectorRight = getAngle2Vector(static_cast<double>(req.angleRight.a),static_cast<double>(req.angleRight.b),static_cast<double>(req.angleRight.c));
        pcl::PointCloud<pcl::PointXYZ>::Ptr V(new pcl::PointCloud<pcl::PointXYZ>);

        v.getCurveCutPoints(C,req.width,vSection,avoidDown,avoidUp,
                            Eigen::Vector3f{static_cast<float>(avoidVectorRight.x()),static_cast<float>(avoidVectorRight.y()),static_cast<float>(avoidVectorRight.z())},
                            Eigen::Vector3f{static_cast<float>(avoidVectorLeft.x()),static_cast<float>(avoidVectorLeft.y()),static_cast<float>(avoidVectorLeft.z())},
                            out,V);

        pcl::PointCloud<pcl::PointXYZ>::Ptr P(new pcl::PointCloud<pcl::PointXYZ>);
        v.copyPointCloud(out,P);

        //12、求取每一个点位的偏移值
        //12.1 点位投影到立面
        v.planarProjectionPointCloud(out,out,planeFacade);
        //12.2 主体点云投影到立面并且保留打磨区域
        v.retainDownWorkProjection(in,out,planeFacade,vSection,in);
        //12.3 提取上半部分的左右两条直线
        pcl::ModelCoefficients::Ptr lineUpLeft(new pcl::ModelCoefficients);
        pcl::ModelCoefficients::Ptr lineUpRight(new pcl::ModelCoefficients);
        //pcl::io::savePLYFile("/home/xs/p2.ply",*in);
        v.getTopLeftAndRightLine(in,planeFacade,vSection,lineUpLeft,lineUpRight);
        //12.4 提取下半部分的左右两条直线
        pcl::ModelCoefficients::Ptr lineDownLeft(new pcl::ModelCoefficients);
        pcl::ModelCoefficients::Ptr lineDownRight(new pcl::ModelCoefficients);
        v.getDownLeftAndRightLine(in,req.length,req.grooveWidth,planeFacade,vSection,lineDownLeft,lineDownRight);


        //13、求取耳板分割线的偏移值
        //13.1 按照道宽对分割线做采样并投影到立面
        pcl::ModelCoefficients::Ptr lineCutEarPro(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cutKeyProCloud(new pcl::PointCloud<pcl::PointXYZ>);

//        for(int i=0;i<6;i++)
//        {
//          std::cout<<"line0->values["<<i<<"]="<<lineUpRight->values[i]<<"f;"<<std::endl;
//        }



        v.getEarCutLineKeyPoints(pEar1,pEar2,planeFacade,vSection,req.width,lineUpLeft,lineUpRight,cutKeyProCloud,lineCutEarPro);

        //13.2 求斜线基础点和姿态
        pcl::PointCloud<pcl::PointXYZ>::Ptr V1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcEarKeyOR(new pcl::PointCloud<pcl::PointXYZ>);
        v.getEarCutBasePoints(cutKeyProCloud,planeFacade,Eigen::Vector3f{static_cast<float>(avoidVectorLeft.x()),static_cast<float>(avoidVectorLeft.y()),static_cast<float>(avoidVectorLeft.z())},lineCutEar,pcEarKeyOR,V1);

        //13.3 对关键点云进行合成
        unsigned long indexPro=0,indexOR=0,indexV=0;
        if(kIsPlus)
        {
            v.fuseEarBaseAndCurve(P,pcEarKeyOR,P,indexOR);
            v.fuseEarBaseAndCurve(out,cutKeyProCloud,out,indexPro);
            v.fuseEarBaseAndCurve(V,V1,V,indexV);
        }

       // pcl::io::savePLYFile("/home/xs/p.ply",*P);

        //14、求交线点距离 偏移
        std::vector<float> distanceRight,distanceLeft;
        v.getBorderWithBaseOffset(out,indexPro,planeFacade,vSection,lineCutEarPro,lineUpLeft,lineUpRight,lineDownLeft,lineDownRight,distanceLeft,distanceRight,kIsPlus);

        //15、求铸造字距离 偏移
        std::vector<std::vector<float>> leftDistances,rightDistance;
        std::vector<std::vector<unsigned long>> leftIndexs,rightIndexs;
        std::vector<pcl::PointCloud<PointT>::Ptr> wordAreas;

        for (unsigned long i=1;i<req.listPcAreas.size();i++)
        {
            pcl::PointCloud<PointT>::Ptr word(new pcl::PointCloud<PointT>);
            for (unsigned long j=0;j<req.listPcAreas[i].listPcPoint.size();j++)
            {
                PointT p;
                p.x=req.listPcAreas[i].listPcPoint[j].x;
                p.y=req.listPcAreas[i].listPcPoint[j].y;
                p.z=req.listPcAreas[i].listPcPoint[j].z;
                word->push_back(p);
            }
            wordAreas.push_back(word);
        }

        v.getWordsWithBaseOffset(out,planeFacade,vSection,wordAreas,leftDistances,rightDistance,leftIndexs,rightIndexs);

        //求取点位 路径
        std::vector<std::vector<PointT>> tracks,vectors;
        v.getDownTrackPoints(P,V,planeFacade,vSection,leftDistances,rightDistance,leftIndexs,rightIndexs,distanceLeft,distanceRight,tracks,vectors);

        //转化路径
        for(unsigned long i=0;i<tracks.size();i++)
        {
            std::vector<PointT> trackI=tracks[i];
            v.pressTrackPoints(tracks[i],vectors[i],req.press,trackI);
            //单位转换mm到m
            for(unsigned long j=0;j<trackI.size();j++)
            {
                trackI[j].x=trackI[j].x/1000;
                trackI[j].y=trackI[j].y/1000;
                trackI[j].z=trackI[j].z/1000;
            }

            //工具转化
            tf::Transform tool2Base,link2Base,tool2Link;
            tf::Vector3 VectorTool= tf1Tool2Tool2.getOrigin();
            tool2Link.setOrigin(tf2Tool2Link.getOrigin());
            tool2Link.setRotation(tf2Tool2Link.getRotation());

            thi_vision::track track;
            VAix.setX(static_cast<double>(vSection[0]));
            VAix.setY(static_cast<double>(vSection[1]));
            VAix.setZ(static_cast<double>(vSection[2]));

            for (unsigned long j =0;j<trackI.size();j++)
            {
                thi_vision::pose pose;
                tool2Base.setOrigin(tf::Vector3(static_cast<double>(trackI[j].x),static_cast<double>(trackI[j].y),static_cast<double>(trackI[j].z)));
                tool2Base.setRotation(getVectorAngle3D(VectorTool,tf::Vector3(static_cast<double>(vectors[i][j].x),static_cast<double>(vectors[i][j].y),static_cast<double>(vectors[i][j].z)),VAix));

                link2Base=tool2Base*tool2Link.inverse();

                double roll, pitch, yaw;
                tf::Matrix3x3(link2Base.getRotation()).getRPY(roll,pitch,yaw);
                Eigen::Vector3f vn(0,-1,static_cast<float>(sqrt(3)));
                vn=vn.normalized();
                if(j==0)//第一个点的话 增加z方向抬高的落刀点
                {
                    pose.point.x=static_cast<float>(link2Base.getOrigin().getX()+DownFIRSTZ[i]*static_cast<double>(vn(0)))*1000.0f;
                    pose.point.y=static_cast<float>(link2Base.getOrigin().getY()+DownFIRSTZ[i]*static_cast<double>(vn(1)))*1000.0f;
                    pose.point.z=static_cast<float>(link2Base.getOrigin().getZ()+DownFIRSTZ[i]*static_cast<double>(vn(2)))*1000.0f;
                    pose.angle.a=static_cast<float>(yaw*180/M_PI);
                    pose.angle.b=static_cast<float>(pitch*180/M_PI);
                    pose.angle.c=static_cast<float>(roll*180/M_PI);
                    track.track.push_back(pose);
                }

                pose.point.x=static_cast<float>(link2Base.getOrigin().getX())*1000.0f;
                pose.point.y=static_cast<float>(link2Base.getOrigin().getY())*1000.0f;
                pose.point.z=static_cast<float>(link2Base.getOrigin().getZ())*1000.0f;
                pose.angle.a=static_cast<float>(yaw*180/M_PI);
                pose.angle.b=static_cast<float>(pitch*180/M_PI);
                pose.angle.c=static_cast<float>(roll*180/M_PI);
                track.track.push_back(pose);
                if(j==trackI.size()-1) //最后一个点的话 增加z方向抬高的起刀点
                {
                    pose.point.x=static_cast<float>(link2Base.getOrigin().getX()+DownLASTZ[i]*static_cast<double>(vn(0)))*1000.0f;
                    pose.point.y=static_cast<float>(link2Base.getOrigin().getY()+DownLASTZ[i]*static_cast<double>(vn(1)))*1000.0f;
                    pose.point.z=static_cast<float>(link2Base.getOrigin().getZ()+DownLASTZ[i]*static_cast<double>(vn(2)))*1000.0f;
                    pose.angle.a=static_cast<float>(yaw*180/M_PI);
                    pose.angle.b=static_cast<float>(pitch*180/M_PI);
                    pose.angle.c=static_cast<float>(roll*180/M_PI);


                    track.track.push_back(pose);
                }
            }

            res.tracks.push_back(track);
        }

    }
    else
    {
        res.ok = false;
        res.tracks.clear();
        return true;
    }

    // 获取程序结束的时间
    auto end = std::chrono::high_resolution_clock::now();
    // 计算程序运行的时间
    std::chrono::duration<double> elapsed = end - start;
    std::string sTime;
    if(elapsed.count()>=60)
    {
        int m = static_cast<int>(elapsed.count()/60);
        double s = elapsed.count()-60*m;
        sTime = "Runing Time: "+std::to_string(m)+" m "+std::to_string(s)+" s";
    }
    else
    {
        sTime = "Runing Time: "+std::to_string(elapsed.count())+" s";
    }
    ROS_INFO("\033[1;35m %s \033[0m",sTime.c_str());


    res.ok = true;
    return true;
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"thi_vision");
    ros::NodeHandle n;

    n.getParam("up/firstZ1",FIRSTZ1);
    n.getParam("up/firstZ2",FIRSTZ2);
    n.getParam("up/lastZ1",LASTZ1);
    n.getParam("up/lastZ2",LASTZ2);
    n.getParam("down/firstZ1",DownFIRSTZ[0]);
    n.getParam("down/firstZ2",DownFIRSTZ[1]);
    n.getParam("down/firstZ3",DownFIRSTZ[2]);
    n.getParam("down/lastZ1",DownLASTZ[0]);
    n.getParam("down/lastZ2",DownLASTZ[1]);
    n.getParam("down/lastZ3",DownLASTZ[2]);

    n.getParam("baffleX",baffleX);
    n.getParam("groundZ",groundZ);

    tf::TransformListener listener;
    try
    {
        listener.waitForTransform("tool2","tool1",  ros::Time(0), ros::Duration(10));
        listener.lookupTransform("tool2","tool1", ros::Time(0),tf1Tool2Tool2); //圆心到接触点的变换
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        return -1;
    }
    try
    {
        listener.waitForTransform("link6","tool2",  ros::Time(0), ros::Duration(10));
        listener.lookupTransform("link6","tool2", ros::Time(0),tf2Tool2Link); //接触点到法兰盘的变换
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        return -1;
    }
    try
    {
        listener.waitForTransform("link6","tool1",  ros::Time(0), ros::Duration(10));
        listener.lookupTransform("link6","tool1", ros::Time(0),tf1Tool2Link); //圆心到法兰盘的变换
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        return -1;
    }

    ros::ServiceServer serVisionTracks = n.advertiseService("/Thi/visionTracks",visionTracksBC);
    ros::spin();
}
