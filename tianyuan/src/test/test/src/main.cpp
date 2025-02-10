#include <ros/ros.h>
#include <thi_vision/visionTracks.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <vtkRenderWindow.h>
#include <vtkPlaneSource.h>
#include <vtkTubeFilter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/transforms.h>

#define TEST_1_21 1

typedef  pcl::PointXYZ PointT;
static tf::StampedTransform  tf2Tool2Link;

static pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("THI"));
static  pcl::PointCloud<pcl::PointXYZ>::Ptr toolTrack1(new pcl::PointCloud<pcl::PointXYZ>);
static  pcl::PointCloud<pcl::PointXYZ>::Ptr toolTrack2(new pcl::PointCloud<pcl::PointXYZ>);
static  pcl::PointCloud<pcl::PointXYZ>::Ptr toolTrack3(new pcl::PointCloud<pcl::PointXYZ>);
static  pcl::PointCloud<pcl::PointXYZ>::Ptr linkTrack(new pcl::PointCloud<pcl::PointXYZ>);

static  pcl::PointCloud<pcl::PointXYZ>::Ptr in(new pcl::PointCloud<pcl::PointXYZ>);

bool floatIsEqual(float a, float b)
{
    return (std::fabs(a-b)<1e-10f)?true:false;
}

int readTxtPointCloud(const char* fileName, pcl::PointCloud<PointT>& cloud)
{
    std::string line;
    std::ifstream file(fileName);

    if (!file.is_open())
    {
        std::cerr << "Unable to open" << fileName << " ." << std::endl;
        return -1;
    }

    while (std::getline(file, line))
    {
        float x, y, z;
        std::stringstream ss(line);
        ss >> x >> y >> z;
        if(!(floatIsEqual(x,0)&&floatIsEqual(y,0)&&floatIsEqual(z,0)))
            cloud.push_back(pcl::PointXYZ(x, y, z));
    }
    return 0;
}

vtkSmartPointer<vtkPolyData> createPlane(const pcl::ModelCoefficients &coefficients, PointT centralPoint, float w,float h)
{

    vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();

    double p0,p1,p2,p3;
    p0=static_cast<double>(coefficients.values[0]);
    p1=static_cast<double>(coefficients.values[1]);
    p2=static_cast<double>(coefficients.values[2]);
    p3=static_cast<double>(coefficients.values[3]);

    float norm_sqr =static_cast<float>(1.0 / (p0*p0 +p1*p1 +p2*p2));
    plane->SetNormal(p0, p1, p2);
    float t = centralPoint.x * coefficients.values[0] + centralPoint.y * coefficients.values[1] + centralPoint.z * coefficients.values[2] + coefficients.values[3];
    centralPoint.x -= coefficients.values[0] * t * norm_sqr;
    centralPoint.y -= coefficients.values[1] * t * norm_sqr;
    centralPoint.z -= coefficients.values[2] * t * norm_sqr;

    plane->SetCenter(static_cast<double>(centralPoint.x), static_cast<double>(centralPoint.y), static_cast<double>(centralPoint.z));

    {
        double pt1[3], pt2[3], orig[3],center[3];
        plane->GetPoint1(pt1);
        plane->GetPoint2(pt2);
        plane->GetOrigin(orig);
        plane->GetCenter(center);

        double _pt1[3], _pt2[3];
        for(int i = 0; i < 3; i++) {
            _pt1[i] = w * (pt1[i] - orig[i]);
            _pt2[i] = h * (pt2[i] - orig[i]);
        }
        //pt1相对于原坐标系下的坐标值
        for(int i=0; i<3;++i)
        {
            pt1[i] = orig[i] + _pt1[i];
            pt2[i] = orig[i] + _pt2[i];
        }
        plane->SetPoint1(pt1);
        plane->SetPoint2(pt2);

    }
    plane->Update();

    return (plane->GetOutput());
}

void createLinePoint(const pcl::ModelCoefficients::Ptr &coefficients, PointT &p1, PointT &p2, float length, float moveCentreLength)
{
    p1.x=coefficients->values[0]+(moveCentreLength+length/2)*coefficients->values[3];
    p1.y=coefficients->values[1]+(moveCentreLength+length/2)*coefficients->values[4];
    p1.z=coefficients->values[2]+(moveCentreLength+length/2)*coefficients->values[5];

    p2.x=coefficients->values[0]+(moveCentreLength-length/2)*coefficients->values[3];
    p2.y=coefficients->values[1]+(moveCentreLength-length/2)*coefficients->values[4];
    p2.z=coefficients->values[2]+(moveCentreLength-length/2)*coefficients->values[5];
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"test_node");
    ros::NodeHandle n;

    tf::Transform tool2Base,link2Base,tool2Link;

    tf::TransformListener listener;
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

    tool2Link.setOrigin(tf2Tool2Link.getOrigin());
    tool2Link.setRotation(tf2Tool2Link.getRotation());


    ros::ServiceClient ser = n.serviceClient<thi_vision::visionTracks>("/Thi/visionTracks");
    thi_vision::visionTracks srv;
    thi_vision::pcArea pcArea0,pcArea1,pcArea2;
    thi_vision::position point;

#if TEST_1_21
    pcl::PointCloud<pcl::PointXYZ>::Ptr W0(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr W1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr W2(new pcl::PointCloud<pcl::PointXYZ>);

    std::string file("/home/xs/pc/pc_merge/down/down3.ply");

    srv.request.width = 3;
    srv.request.press = 1;
    srv.request.length = 1300;
    srv.request.fileType = "ply";
    srv.request.avoidLeft = 100;
    srv.request.workpiece = 1;
    srv.request.avoidRight =100;
    srv.request.grooveWidth = 20;
    srv.request.pointCloudFile=file;

    srv.request.angleLeft.a = 89.38f;
    srv.request.angleLeft.b = 17.52f;
    srv.request.angleLeft.c = 91.29f;

    srv.request.angleRight.a=  88.18f;
    srv.request.angleRight.b = 66.68f;
    srv.request.angleRight.c = 89.57f;



    W0->push_back(pcl::PointXYZ(1849.92f, 932.137f, 337.46f));
    W0->push_back(pcl::PointXYZ(2088.21f, 935.512f, 417.723f));

    W1->push_back(pcl::PointXYZ(1992.36f, 923.592f, 115.68f));
    W1->push_back(pcl::PointXYZ(1985.26f, 867.112f, -5.98484f));
    W1->push_back(pcl::PointXYZ(2135.2f, 855.941f, -11.5673f));
    W1->push_back(pcl::PointXYZ(2131.55f, 923.573f, 109.394f));

    W2->push_back(pcl::PointXYZ(2233.03f, 921.803f, 103.572f));
    W2->push_back(pcl::PointXYZ(2215.59f, 811.5f, -47.5833f));
    W2->push_back(pcl::PointXYZ(2805.39f, 815.077f, -68.8148f));
    W2->push_back(pcl::PointXYZ(2806.52f, 943.723f, 75.3801f));


    for (unsigned long i=0;i<2;i++)
    {
        point.x = W0->points[i].x;point.y =  W0->points[i].y;point.z =  W0->points[i].z;
        pcArea0.listPcPoint.push_back(point);
    }
    srv.request.listPcAreas.push_back(pcArea0);

    for (unsigned long i=0;i<4;i++)
    {
        point.x = W1->points[i].x;point.y =  W1->points[i].y;point.z =  W1->points[i].z;
        pcArea1.listPcPoint.push_back(point);
    }
    srv.request.listPcAreas.push_back(pcArea1);

    for (unsigned long i=0;i<4;i++)
    {
        point.x = W2->points[i].x;point.y =  W2->points[i].y;point.z =  W2->points[i].z;
        pcArea2.listPcPoint.push_back(point);
    }
    srv.request.listPcAreas.push_back(pcArea2);

#else

    std::string file("/home/xs/pc/pc_merge/down/down4.ply");

    srv.request.width = 1;
    srv.request.press = 15;
    srv.request.length = 1300;
    srv.request.fileType = "ply";
    srv.request.avoidLeft = 5;
    srv.request.workpiece = 1;
    srv.request.avoidRight =10;
    srv.request.grooveWidth = 20;
    srv.request.pointCloudFile=file;

    srv.request.angleLeft.a = 89.38f;
    srv.request.angleLeft.b = 17.52f;
    srv.request.angleLeft.c = 91.29f;

    srv.request.angleRight.a=  88.18f;
    srv.request.angleRight.b = 66.68f;
    srv.request.angleRight.c = 89.57f;

#if 0
    //斜线
    point.x = 1949.33f; point.y = 927.201f;point.z=309.306f;
    pcArea0.listPcPoint.push_back(point);
    point.x = 2282.87f; point.y = 922.494f;point.z= 404.66f;
    pcArea0.listPcPoint.push_back(point);

    //第一片铸造字
    point.x = 2165.22f; point.y = 910.413f;point.z=122.738f;
    pcArea1.listPcPoint.push_back(point);
    point.x = 2160.36f; point.y = 854.125f;point.z= -0.34316f;
    pcArea1.listPcPoint.push_back(point);
    point.x = 2972.13f; point.y = 839.531f;point.z= 4.92128f;
    pcArea1.listPcPoint.push_back(point);
    point.x = 2976.42f; point.y = 902.273f;point.z= 127.098f;
    pcArea1.listPcPoint.push_back(point);

    srv.request.listPcAreas.push_back(pcArea0);
    srv.request.listPcAreas.push_back(pcArea1);
 #else
    //斜线
    point.x = 1886.12f; point.y = 929.303f;point.z=275.76f;
    pcArea0.listPcPoint.push_back(point);
    point.x = 2211.78f; point.y = 922.937f;point.z= 381.919f;
    pcArea0.listPcPoint.push_back(point);

    //第一片铸造字
    point.x = 2165.92f; point.y = 911.395f;point.z=123.846f;
    pcArea1.listPcPoint.push_back(point);
    point.x = 2164.48f; point.y = 863.738f;point.z= -4.29619f;
    pcArea1.listPcPoint.push_back(point);
    point.x = 2971.34f; point.y = 836.841f;point.z= -1.12076f;
    pcArea1.listPcPoint.push_back(point);
    point.x = 2993.09f; point.y = 900.284f;point.z= 124.696f;
    pcArea1.listPcPoint.push_back(point);

    srv.request.listPcAreas.push_back(pcArea0);
    srv.request.listPcAreas.push_back(pcArea1);
#endif
#endif

    sleep(3);
    if(ser.call(srv))
    {
        if(srv.response.ok)
        {
            for (unsigned long i=0;i<srv.response.tracks.size();i++)
            {
                for (const auto& pose :srv.response.tracks[i].track)
                {
                    tf::Quaternion q;
                    pcl::PointXYZ point;

                    linkTrack->push_back(pcl::PointXYZ(pose.point.x,pose.point.y,pose.point.z));

                    link2Base.setOrigin(tf::Vector3(static_cast<double>(pose.point.x/1000),
                                                    static_cast<double>(pose.point.y/1000),
                                                    static_cast<double>(pose.point.z/1000)));
                    q.setRPY(static_cast<double>(pose.angle.c)*M_PI/180,
                             static_cast<double>(pose.angle.b)*M_PI/180,
                             static_cast<double>(pose.angle.a)*M_PI/180);

                    link2Base.setRotation(q);
                    tool2Base=link2Base*tool2Link;

                    point.x = static_cast<float>(tool2Base.getOrigin().x()*1000);
                    point.y = static_cast<float>(tool2Base.getOrigin().y()*1000);
                    point.z = static_cast<float>(tool2Base.getOrigin().z()*1000);

                    if(i==0)
                    {
                        toolTrack1->points.push_back(point);
                    }
                    else if(i==1)
                    {
                        toolTrack2->points.push_back(point);
                    }
                    else if (i==2)
                    {
                        toolTrack3->points.push_back(point);
                    }
                }
            }

        }
        else
        {
            ROS_ERROR("error!!!! pxs");
        }
    }

    pcl::io::loadPLYFile(file,*in);
    for (unsigned long i=0;i<toolTrack1->size()-1;i++)
    {
        std::string l = "toolTrack1_"+std::to_string(i);
        viewer->addLine(toolTrack1->points[i],toolTrack1->points[i+1],l);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,l);
    }
    for (unsigned long i=0;i<toolTrack2->size()-1;i++)
    {
        std::string l = "toolTrack2_"+std::to_string(i);
        viewer->addLine(toolTrack2->points[i],toolTrack2->points[i+1],l);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,1,l);
    }

//    for (unsigned long i=0;i<toolTrack3->size()-1;i++)
//    {
//        std::string l = "toolTrack3_"+std::to_string(i);
//        viewer->addLine(toolTrack3->points[i],toolTrack3->points[i+1],l);
//        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,1,l);
//    }

//    for (unsigned long i=0;i<linkTrack->size()-1;i++)
//    {
//        std::string l = "linkTrack_"+std::to_string(i);
//        viewer->addLine(linkTrack->points[i],linkTrack->points[i+1],l);
//        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,l);
//    }

    viewer->addPointCloud(in,"in");

    pcl::ModelCoefficients::Ptr lineUpLeft(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr lineUpRight(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr lineDownLeft(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr lineDownRight(new pcl::ModelCoefficients);

    lineUpLeft->values.resize(6);
    lineUpRight->values.resize(6);
    lineDownLeft->values.resize(6);
    lineDownRight->values.resize(6);


    lineUpLeft->values[0]=1549.32f;
    lineUpLeft->values[1]=467.181f;
    lineUpLeft->values[2]=139.216f;
    lineUpLeft->values[3]=-0.136138f;
    lineUpLeft->values[4]=0.00427401f;
    lineUpLeft->values[5]=0.990681f;

    lineUpRight->values[0]=3078.21f;
    lineUpRight->values[1]=421.274f;
    lineUpRight->values[2]=48.4217f;
    lineUpRight->values[3]=-0.659606f;
    lineUpRight->values[4]=0.0199396f;
    lineUpRight->values[5]=0.751347f;

    lineDownLeft->values[0]=1594.34f;
    lineDownLeft->values[1]=465.777f;
    lineDownLeft->values[2]=-135.837f;
    lineDownLeft->values[3]=0.00345689f;
    lineDownLeft->values[4]=-0.000293352f;
    lineDownLeft->values[5]=-0.999994f;

    lineDownRight->values[0]=2893.75f;
    lineDownRight->values[1]=426.777f;
    lineDownRight->values[2]=-131.334f;
    lineDownRight->values[3]=0.00345689f;
    lineDownRight->values[4]=-0.000293352f;
    lineDownRight->values[5]=-0.999994f;



    PointT P1,P2;
    createLinePoint(lineUpLeft,P1,P2,1000,0);
    viewer->addLine(P1,P2,"l1");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"l1");

    createLinePoint(lineUpRight,P1,P2,1000,0);
    viewer->addLine(P1,P2,"l2");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"l2");

    createLinePoint(lineDownLeft,P1,P2,1000,0);
    viewer->addLine(P1,P2,"l3");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,1,"l3");

    createLinePoint(lineDownRight,P1,P2,1000,0);
    viewer->addLine(P1,P2,"l4");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,1,"l4");


//    pcl::ModelCoefficients::Ptr planeSection(new pcl::ModelCoefficients);
//    planeSection->values.resize(4);

//    planeSection->values[0] = 0.999713f;
//    planeSection->values[1] = 0.0236297f;
//    planeSection->values[2] = 0.00401588f;
//    planeSection->values[3] = -1432.66f;

//    readTxtPointCloud("/home/xs/11.txt",*out);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    ros::spin();
    return 0;
}
