#include "thivision.h"

ThiVision::ThiVision()
{
}

/**********************************************
* @projectName   ThiVision::readPointCloudFile
* @brief         读取点云数据，将点云文件转化为pcl::PointCloud<PointT>类
* @inParam
*                fileName: 点云文件路径
*                fileType: 点云的文件类型(枚举类型FILE_TYPE::PCD、FILE_TYPE::PLY或者FILE_TYPE::TXT)
* @outParam      cloud: (输出)读取出来点云存储内存
* @return        返回一个读取状态码(枚举类型READ_STATE)
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
READ_STATE ThiVision::readPointCloudFile(std::string fileName, FILE_TYPE fileType, pcl::PointCloud<PointT>::Ptr &cloud)
{
    std::ifstream file(fileName);
    if(file.good())
    {
        if(fileType==FILE_TYPE::PCD)
        {
            if(pcl::io::loadPCDFile(fileName,*cloud)<0)
                return READ_STATE::READ_FAIL;
            else
                return READ_STATE::READ_SUCCEED;
        }
        else if(fileType==FILE_TYPE::PLY)
        {
            if(pcl::io::loadPLYFile(fileName,*cloud)<0)
                return READ_STATE::READ_FAIL;
            else
                return READ_STATE::READ_SUCCEED;
        }
        else
        {
            return READ_STATE::FILETYPE_ERROR;
        }
    }
    else
    {
        return READ_STATE::FILE_INEXIST;
    }
}

/**********************************************
* @projectName   ThiVision::blowUpPointCloud
* @brief         将点云的间距放大num倍(用于单位转换)
* @inParam
*                cloudIn: 输入的点云
*                num: 放大倍数
* @outParam
*                cloudOut: 输出的点云
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-22
**********************************************/
void ThiVision::blowUpPointCloud(pcl::PointCloud<PointT>::Ptr& cloudIn,pcl::PointCloud<PointT>::Ptr& cloudOut,int num)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    for (unsigned long i=0;i<cloudIn->size();i++)
    {
        PointT  p{cloudIn->points[i].x*num,cloudIn->points[i].y*num,cloudIn->points[i].z*num};
        temp->push_back(p);
    }
    cloudOut->clear();
    *cloudOut=*temp;
}

/**********************************************
* @projectName   ThiVision::voxelPointCloud
* @brief         对输入的点云体素网格化
* @inParam
*                cloudIn: 输入的点云
*                size: 体素化的尺寸(size[0] x  size[1] y  size[2] z)
* @outParam
*                cloudOut: (输出)输出的点云
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
void ThiVision::voxelPointCloud(pcl::PointCloud<PointT>::Ptr &cloudIn, pcl::PointCloud<PointT>::Ptr &cloudOut, Eigen::Vector3f size)
{
    pcl::VoxelGrid<PointT> vor;
    vor.setInputCloud(cloudIn);
    vor.setLeafSize(size[0],size[1],size[2]);
    vor.filter(*cloudOut);
}

/**********************************************
* @projectName   ThiVision::getPointCloudMinMax3D
* @brief         得到3D点云数据集在X、Y 和 Z 轴上的最小和最大坐标值
* @inParam
*                cloudIn: 输入的点云
* @outParam      minPoint: (输出)最小范围的点的存储
*                maxPoint: (输出)最大范围的点的存储
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
void ThiVision::getPointCloudMinMax3D(pcl::PointCloud<PointT>::Ptr &cloudIn, PointT &minPoint, PointT &maxPoint)
{
    pcl::getMinMax3D(*cloudIn,minPoint,maxPoint);
}

/**********************************************
* @projectName   ThiVision::cutPointCloud
* @brief         剪切点云(直通滤波)
* @inParam
*                cloudIn: 剪切输入的点云
*                minPoint: xyz值最小点
*                maxPoint: xyz值最大点
* @outParam
*                cloudOut: (输出)剪切输出的点云
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
void ThiVision::cutPointCloud(pcl::PointCloud<PointT>::Ptr &cloudIn, pcl::PointCloud<PointT>::Ptr &cloudOut, PointT &minPoint, PointT &maxPoint)
{
    pcl::PassThrough<PointT> filt;

    filt.setInputCloud(cloudIn);
    filt.setFilterFieldName("x");
    filt.setFilterLimits(minPoint.x,maxPoint.x);
    filt.filter(*cloudOut);

    filt.setInputCloud(cloudOut);
    filt.setFilterFieldName("y");
    filt.setFilterLimits(minPoint.y,maxPoint.y);
    filt.filter(*cloudOut);

    filt.setInputCloud(cloudOut);
    filt.setFilterFieldName("z");
    filt.setFilterLimits(minPoint.z,maxPoint.z);
    filt.filter(*cloudOut);
}

/**********************************************
 * @projectName   ThiVision::statisticalPointCloud
 * @brief         对输入的点云进行概率统计滤波来去除离群点
 * @inParam
 *                cloudIn: 输入的点云
 *                K: 采样领域计算的点数
 *                stdThreshold: 计算的标准差阈值(默认为1.0)
 * @outParam
 *                cloudOut: (输出)输出的点云
 * @return        void
* @author         xs(1874020422@qq.com)
* @date           2024-08-08
 **********************************************/
void ThiVision::statisticalPointCloud(pcl::PointCloud<PointT>::Ptr &cloudIn, pcl::PointCloud<PointT>::Ptr &cloudOut, int K, double stdThreshold)
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloudIn);
    sor.setMeanK(K);
    sor.setStddevMulThresh(stdThreshold);
    sor.filter(*cloudOut);
}

/**********************************************
* @projectName   ThiVision::copyPointCloud
* @brief         复制拷贝一份点云
* @inParam
*                cloudIn: 输入(被拷贝)的点云
* @outParam
*                cloudOut: 输出(拷贝到)的点云
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
void ThiVision::copyPointCloud(pcl::PointCloud<PointT>::Ptr &cloudIn, pcl::PointCloud<PointT>::Ptr &cloudOut)
{
    *cloudOut=*cloudIn;
}

/**********************************************
* @projectName   ThiVision::segmentationCylinder
* @brief         按照圆柱面拟合点云
* @inParam
*                cloudIn: 输入待拟合的点云
*                K: 搜索领域的点数
*                maxIterations: 拟合迭代次数
*                normalWeight: 法向量的距离权重
*                distanceThreshold: 距离标准差
*                rMin: 圆柱的最小半径
*                rMax: 圆柱的最大半径
* @outParam
*                inliers: 拟合内点对应输入点云的索引
*                cylinderCoefficients: 存储拟合圆柱的模型
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
void ThiVision::segmentationCylinder(pcl::PointCloud<PointT>::Ptr &cloudIn, int K, int maxIterations, double normalWeight, double distanceThreshold, double rMin, double rMax, pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &cylinderCoefficients)
{
    pcl::NormalEstimation<PointT,pcl::Normal>::Ptr ne(new pcl::NormalEstimation<PointT,pcl::Normal>);
    pcl::SACSegmentationFromNormals<PointT,pcl::Normal>* seg(new pcl::SACSegmentationFromNormals<PointT,pcl::Normal>);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);

    ne->setSearchMethod(tree);
    ne->setInputCloud(cloudIn);
    ne->setKSearch(K);
    ne->compute(*cloudNormals);

    seg->setOptimizeCoefficients(true);
    seg->setModelType(pcl::SACMODEL_CYLINDER);
    seg->setMethodType(pcl::SAC_RANSAC);
    seg->setNormalDistanceWeight(normalWeight);
    seg->setMaxIterations(maxIterations);
    seg->setDistanceThreshold(distanceThreshold);
    seg->setRadiusLimits(rMin, rMax);
    seg->setInputCloud(cloudIn);
    seg->setInputNormals(cloudNormals);

    seg->segment(*inliers, *cylinderCoefficients);

}

/**********************************************
* @projectName   ThiVision::segmentationPlane
* @brief         按照平面拟合点云
* @inParam
*                cloudIn: 输入待拟合的点云
*                maxIterations: 拟合迭代次数
*                distanceThreshold: 距离标准差
* @outParam
*                inliers: 拟合内点对应输入点云的索引
*                planeCoefficients: 存储拟合平面的模型
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
void ThiVision::segmentationPlane(pcl::PointCloud<PointT>::Ptr &cloudIn, int maxIterations, double distanceThreshold, pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &planeCoefficients)
{
    pcl::SACSegmentation<PointT>* seg(new  pcl::SACSegmentation<PointT>);
    seg->setOptimizeCoefficients(true);
    seg->setModelType(pcl::SACMODEL_PLANE);
    seg->setMethodType(pcl::SAC_RANSAC);
    seg->setMaxIterations(maxIterations);
    seg->setDistanceThreshold(distanceThreshold);
    seg->setInputCloud(cloudIn);
    seg->segment(*inliers,*planeCoefficients);
}

/**********************************************
* @projectName   ThiVision::segmentationLine
* @brief         按照直线拟合点云
* @inParam
*                cloudIn: 输入待拟合的点云
*                maxIterations: 拟合迭代次数
*                distanceThreshold: 距离标准差
* @outParam
*                inliers: 拟合内点对应输入点云的索引
*                lineCoefficients: 存储拟合直线的模型
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
void ThiVision::segmentationLine(pcl::PointCloud<PointT>::Ptr &cloudIn, int maxIterations, double distanceThreshold, pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &lineCoefficients)
{
    pcl::SACSegmentation<PointT>* seg(new  pcl::SACSegmentation<PointT>);
    seg->setOptimizeCoefficients(true);
    seg->setModelType(pcl::SACMODEL_LINE);
    seg->setMethodType(pcl::SAC_RANSAC);
    seg->setMaxIterations(maxIterations);
    seg->setDistanceThreshold(distanceThreshold);
    seg->setInputCloud(cloudIn);
    seg->segment(*inliers,*lineCoefficients);

}

/**********************************************
* @projectName   ThiVision::leaveIndexesPoint
* @brief         保留点云索引对应的点(一般为拟合的内点)
* @inParam
*                cloudIn: 输入的点云
*                inliers: 输入的索引
*                cloudOut: 保留后的点云
* @outParam
*
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
void ThiVision::leaveIndexesPoint(pcl::PointCloud<PointT>::Ptr &cloudIn, pcl::PointCloud<PointT>::Ptr &cloudOut, pcl::PointIndices::Ptr inliers)
{
    pcl::ExtractIndices<PointT> ext;
    ext.setInputCloud(cloudIn);
    ext.setIndices(inliers);
    ext.setNegative(false);
    ext.filter(*cloudOut);
}

/**********************************************
* @projectName   ThiVision::removeIndexesPoint
* @brief         去除点云索引对应的点(一般为拟合的内点)
* @inParam
*                cloudIn: 输入的点云
*                inliers: 输入的索引
*                cloudOut: 移除后的点云
* @outParam
*
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
void ThiVision::removeIndexesPoint(pcl::PointCloud<PointT>::Ptr &cloudIn, pcl::PointCloud<PointT>::Ptr &cloudOut, pcl::PointIndices::Ptr inliers)
{
    pcl::ExtractIndices<PointT> ext;
    ext.setInputCloud(cloudIn);
    ext.setIndices(inliers);
    ext.setNegative(true);
    ext.filter(*cloudOut);
}

/**********************************************
* @projectName   ThiVision::planarProjectionPointCloud
* @brief         对点云进行一个平面投影
* @inParam       cloudIn: 输入待过滤的点云
*                coefficients: 平面方程 coefficients[0] A  coefficients[1] B  coefficients[2] C  coefficients[3] D
* @outParam
*                cloudOut: (输出)输出过滤后的点云
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
void ThiVision::planarProjectionPointCloud(pcl::PointCloud<PointT>::Ptr &cloudIn, pcl::PointCloud<PointT>::Ptr &cloudOut, pcl::ModelCoefficients::Ptr planeCoefficients)
{
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloudIn);
    proj.setModelCoefficients(planeCoefficients);
    proj.filter(*cloudOut);
}

/**********************************************
* @projectName   ThiVision::getPlanarConcaveHull
* @brief         从输入点云中得到凹多边形点云
* @inParam
*                cloudIn: 输入待提取的点云
*                alpha: 多边形边缘的光滑程度
*                   (和点的密度有关,alpha值越小，生成的凹多边形越接近原始点云的轮廓，边界更加锐利。而alpha值越大，生成的凹多边形越平滑，边界更加光滑)
*
* @outParam
*                cloudOut: (输出)输出提取后的点云
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
void ThiVision::getPlanarConcaveHull(pcl::PointCloud<PointT>::Ptr &cloudIn, pcl::PointCloud<PointT>::Ptr &cloudOut, double alpha)
{
    pcl::ConcaveHull<PointT> chull;
    chull.setInputCloud(cloudIn);
    chull.setAlpha(alpha);
    chull.reconstruct(*cloudOut);
}

/**********************************************
* @projectName   ThiVision::createSectionPlane
* @brief         创建截面平面/平面
* @inParam
*                lineCoefficients: 面上的一条直线
*                cylinderCoefficients: 拟合的圆柱(提供法向)
*                length: 槽的宽度
* @outParam
*                planeCoefficients: 创建的截面方程
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
void ThiVision::createSectionPlane(pcl::ModelCoefficients::Ptr &lineCoefficients, pcl::ModelCoefficients::Ptr &cylinderCoefficients,float length, pcl::ModelCoefficients::Ptr &planeCoefficients)
{
    planeCoefficients->values.resize(4);

    float x = lineCoefficients->values[0];
    float y = lineCoefficients->values[1];
    float z = lineCoefficients->values[2];
    float a = cylinderCoefficients->values[3];
    float b = cylinderCoefficients->values[4];
    float c = cylinderCoefficients->values[5];
    float d = -(a*x+b*y+c*z);

    Eigen::Vector3f n(a,b,c);
    Eigen::Vector3f nx(1,0,0);
    if(n.dot(nx)<0)
    {
        length = -length;
    }
    planeCoefficients->values[0]=a;
    planeCoefficients->values[1]=b;
    planeCoefficients->values[2]=c;
    planeCoefficients->values[3]=d-length*sqrt(a*a+b*b+c*c);
}

/**********************************************
* @projectName   ThiVision::createSectionPlane
* @brief         创建截面平面/平面(重载)
* @inParam
*                lineCoefficients: 面上的一条直线
*                vAixs: 面的法向量
* @outParam
*                planeCoefficients: 创建的截面方程
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
void ThiVision::createSectionPlane(pcl::ModelCoefficients::Ptr &lineCoefficients, Eigen::Vector3f &vAixs, pcl::ModelCoefficients::Ptr &planeCoefficients)
{
    planeCoefficients->values.resize(4);

    float x = lineCoefficients->values[0];
    float y = lineCoefficients->values[1];
    float z = lineCoefficients->values[2];
    float a = vAixs[0];
    float b = vAixs[1];
    float c = vAixs[2];
    float d = -(a*x+b*y+c*z);
    planeCoefficients->values[0]=a;
    planeCoefficients->values[1]=b;
    planeCoefficients->values[2]=c;
    planeCoefficients->values[3]=d;
}

/**********************************************
* @projectName   ThiVision::getPlanePlaneLine
* @brief         得到两个平面的交线
* @inParam
*                plane1: 平面1
*                plane2: 平面2
* @outParam
*                line: 两个平面的交线
* @return        成功返回0,否则返回-1
* @author        xs(1874020422@qq.com)
* @date          2024-08-09
**********************************************/
int ThiVision::getPlanePlaneLine(pcl::ModelCoefficients::Ptr plane1,pcl::ModelCoefficients::Ptr plane2,pcl::ModelCoefficients::Ptr &line)
{
    if(plane1->values.size()!=4||plane2->values.size()!=4)
        return -1;

    float A1,B1,C1,D1,A2,B2,C2,D2;

    A1=plane1->values[0];
    B1=plane1->values[1];
    C1=plane1->values[2];
    D1=plane1->values[3];

    A2=plane2->values[0];
    B2=plane2->values[1];
    C2=plane2->values[2];
    D2=plane2->values[3];

    Eigen::Vector3f pN1{A1,B1,C1},pN2{A2,B2,C2};
    if(std::abs(static_cast<double>(pN1.dot(pN2)-pN1.norm()*pN2.norm()))<1e-10)
        return -2;


    //    float K =A2*B1-A1*B2;
    //    float x0 = (D1*B2-D2*B1)/K;
    //    float y0 = (D2*A1-D1*A2)/K;

    //    float k=C1*B2-C2*B1;
    //    float y0=(D1*C2-D2*C1)/k;
    //    float z0=(D2*B1-D1*B2)/k;

    float k=C2*A1-A2*C1;
    float x0=(C1*D2-D1*C2)/k;
    float z0=(A2*D1-D2*A1)/k;


    Eigen::Vector3f n1{A1,B1,C1};
    Eigen::Vector3f n2{A2,B2,C2};

    Eigen::Vector3f vectorLine;
    vectorLine=n1.cross(n2).normalized();

    line->values.resize(6);
    line->values[0]=x0;
    line->values[1]=0;
    line->values[2]=z0;
    line->values[3]=vectorLine.x();
    line->values[4]=vectorLine.y();
    line->values[5]=vectorLine.z();
    return 0;
}

/**********************************************
* @projectName   ThiVision::getPlaneCylinderLine
* @brief         得到一个平面和圆柱面的交线
* @inParam
*                plane: 平面参数
*                cylinder: 圆柱面参数
* @outParam
*                line: 输出的两条直线参数列表
* @return        成功返回0,否则返回-1
* @author        xs(1874020422@qq.com)
* @date          2024-08-09
**********************************************/
int ThiVision::getPlaneCylinderLine(pcl::ModelCoefficients::Ptr plane, pcl::ModelCoefficients::Ptr cylinder, std::vector<pcl::ModelCoefficients::Ptr> &line)
{

    if(plane->values.size()!=4||cylinder->values.size()!=7)
        return -1;

    pcl::ModelCoefficients::Ptr cylinderSection(new pcl::ModelCoefficients);
    cylinderSection->values.resize(4);
    cylinderSection->values[0]=cylinder->values[3];
    cylinderSection->values[1]=cylinder->values[4];
    cylinderSection->values[2]=cylinder->values[5];
    cylinderSection->values[3]=-(cylinder->values[0]*cylinder->values[3]+cylinder->values[1]*cylinder->values[4]+cylinder->values[2]*cylinder->values[5]);

    pcl::ModelCoefficients::Ptr planeCylinderSectionLine(new pcl::ModelCoefficients);
    getPlanePlaneLine(plane,cylinderSection,planeCylinderSectionLine);

    //求直线和球的交点
    //截线点向值
    float x0=planeCylinderSectionLine->values[0];
    float y0=planeCylinderSectionLine->values[1];
    float z0=planeCylinderSectionLine->values[2];
    float a=planeCylinderSectionLine->values[3];
    float b=planeCylinderSectionLine->values[4];
    float c=planeCylinderSectionLine->values[5];

    //球心坐标和半径
    float o=cylinder->values[0];
    float p=cylinder->values[1];
    float k=cylinder->values[2];
    float r=cylinder->values[6];

    //二次方程系数
    float A=a*a+b*b+c*c;
    float B=2*(a*(x0-o)+b*(y0-p)+c*(z0-k));
    float C=(x0-o)*(x0-o)+(y0-p)*(y0-p)+(z0-k)*(z0-k)-r*r;

    float D=sqrt(B*B-4*A*C);

    //求解二次方程
    float t1,t2;
    t1=(-B+D)/2*A;
    t2=(-B-D)/2*A;

    //求出截线和圆柱交点
    PointT p1,p2;
    p1.x=x0+a*t1;
    p1.y=y0+b*t1;
    p1.z=z0+c*t1;

    p2.x=x0+a*t2;
    p2.y=y0+b*t2;
    p2.z=z0+c*t2;


    //得到平面和柱面交线
    pcl::ModelCoefficients::Ptr line1(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr line2(new pcl::ModelCoefficients);
    line1->values.resize(6);
    line2->values.resize(6);

    line1->values[0]=p1.x;
    line1->values[1]=p1.y;
    line1->values[2]=p1.z;
    line1->values[3]=cylinder->values[3];
    line1->values[4]=cylinder->values[4];
    line1->values[5]=cylinder->values[5];

    line2->values[0]=p2.x;
    line2->values[1]=p2.y;
    line2->values[2]=p2.z;
    line2->values[3]=cylinder->values[3];
    line2->values[4]=cylinder->values[4];
    line2->values[5]=cylinder->values[5];

    //填充
    line.push_back(line1);
    line.push_back(line2);


    return 0;
}

/**********************************************
* @projectName   ThiVision::getLineLineIntersectPoint
* @brief         得到两条直线的交点
* @inParam
*                line1: 直线1
*                line2: 直线2
* @outParam
*                point: 两条直线的交点
* @return        成功返回0,否则返回-1
* @author        xs(1874020422@qq.com)
* @date          2024-08-09
**********************************************/
int ThiVision::getLineLineIntersectPoint(pcl::ModelCoefficients::Ptr &line1,pcl::ModelCoefficients::Ptr &line2,PointT &point)
{
    if(line1->values.size()!=6||line2->values.size()!=6)
        return -1;
    Eigen::VectorXf l1(6),l2(6);

    for (unsigned long i=0;i<6;i++)
    {
        l1[static_cast<long>(i)]=line1->values[i];
        l2[static_cast<long>(i)]=line2->values[i];
    }

    Eigen::Vector4f point1,point2;
    pcl::lineToLineSegment(l1,l2,point1,point2);
    point.x=(point1[0]+point2[0])/2;
    point.y=(point1[1]+point2[1])/2;
    point.z=(point1[2]+point2[2])/2;

    return 0;
}

/**********************************************
* @projectName   ThiVision::getArcCenterAngle
* @brief         得到圆弧线的角度和圆心位置
* @inParam
*                pLeft: 圆弧左侧点(不一定是左侧的，该函数会自己判断)
*                pRight: 圆弧右侧点(不一定是右侧的，该函数会自己判断)
* @outParam
*                center: 该圆弧圆心的位置
*                angle: 圆弧的角度
* @return        成功返回0,否则返回-1
* @author        xs(1874020422@qq.com)
* @date          2024-08-09
**********************************************/
int ThiVision::getArcCenterAngle(PointT pLeft, PointT pRight, pcl::ModelCoefficients::Ptr cylinder, PointT &center, float &angle)
{
    if(cylinder->values.size()!=7)
        return -1;

    PointT mid;
    pcl::ModelCoefficients::Ptr line1(new pcl::ModelCoefficients),line2(new pcl::ModelCoefficients);

    mid.x=(pLeft.x+pRight.x)/2;
    mid.y=(pLeft.y+pRight.y)/2;
    mid.z=(pLeft.z+pRight.z)/2;

    Eigen::Vector3f l{pLeft.x-pRight.x,pLeft.y-pRight.y,pLeft.z-pRight.z};
    Eigen::Vector3f n{cylinder->values[3],cylinder->values[4],cylinder->values[5]};
    Eigen::Vector3f ln=l.cross(n).normalized();

    line1->values.resize(6);
    line2->values.resize(6);

    line1->values[0]=mid.x;
    line1->values[1]=mid.y;
    line1->values[2]=mid.z;
    line1->values[3]=ln[0];
    line1->values[4]=ln[1];
    line1->values[5]=ln[2];

    for (unsigned long i=0;i<6;i++)
    {
        line2->values[i]=cylinder->values[i];
    }
    if(getLineLineIntersectPoint(line1,line2,center)<0)
        return -1;

    float lMidP=sqrt((mid.x-center.x)*(mid.x-center.x)+(mid.y-center.y)*(mid.y-center.y)+(mid.z-center.z)*(mid.z-center.z));
    angle=2*std::acos(lMidP/cylinder->values[6]);
    return 0;
}

/**********************************************
* @projectName   ThiVision::getArcDividePoint
* @brief         对圆弧作道宽的均分点和姿态向量(含有左右侧避障区)计算
* @inParam       pLeft: 圆弧左侧点
*                pRight: 圆弧右侧点
*                cylinder: 圆柱面模型
*                width: 道宽
*                avoidLeft: 左侧避障圆弧长度
*                avoidRight: 右侧避障圆弧长度
*                avoidVectorLeft: 左侧避障姿态向量
*                avoidVectorRight: 右侧避障姿态向量
* @outParam
*                points: 输出圆弧均分路径点
*                vector: 输出圆弧路径点对应姿态向量
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-09
**********************************************/
int ThiVision::getArcDividePoint(PointT pLeft, PointT pRight, pcl::ModelCoefficients::Ptr cylinder, float width, float avoidLeft, float avoidRight,Eigen::Vector3f avoidVectorLeft,Eigen::Vector3f avoidVectorRight, std::vector<PointT> &points, std::vector<PointT> &vector)
{
    //这里要增加左右判断条件和法相方向条件
    if(pRight.y > pLeft.y)
    {
        PointT temp=pRight;
        pRight=pLeft;
        pLeft=temp;
    }
    PointT center;
    float angle;
    if(getArcCenterAngle(pLeft,pRight,cylinder,center,angle)<0)
        return -1;

    float datleAngle=width/cylinder->values[6];

    Eigen::Vector3f n{cylinder->values[3],cylinder->values[4],cylinder->values[5]};
    Eigen::Vector3f vl{pLeft.x-center.x,pLeft.y-center.y,pLeft.z-center.z};
    Eigen::Vector3f vr{pRight.x-center.x,pRight.y-center.y,pRight.z-center.z};
    Eigen::Vector3f vmid{(pLeft.x+pRight.x)/2-center.x,(pLeft.y+pRight.y)/2-center.y,(pLeft.z+pRight.z)/2-center.z};
    if(n(0)>0)
    {
        n=-n; //旋转轴需要朝垂直于旋转向量的平面并朝外
    }
    int divideTime=static_cast<int>(angle/datleAngle);
    int avoidLeftTime=static_cast<int>(avoidLeft/width)+1;
    int avoidRightTime=static_cast<int>(avoidRight/width)+1 ;

    points.push_back(pRight);
    vector.push_back({avoidVectorRight.x(),avoidVectorRight.y(),avoidVectorRight.z()});
    for (int i=0;i<divideTime;i++)
    {
        PointT p;
        Eigen::Vector3f datleAngleVector=getRotatedVector(vr,n,(i+1)*static_cast<double>(datleAngle));
        p.x=center.x+datleAngleVector[0]*cylinder->values[6];
        p.y=center.y+datleAngleVector[1]*cylinder->values[6];
        p.z=center.z+datleAngleVector[2]*cylinder->values[6];

        PointT v;
        if(i<avoidRightTime)
        {
            v.x=avoidVectorRight.x();
            v.y=avoidVectorRight.y();
            v.z=avoidVectorRight.z();
        }
        else if(i>divideTime-avoidLeftTime)
        {
            v.x=avoidVectorLeft.x();
            v.y=avoidVectorLeft.y();
            v.z=avoidVectorLeft.z();
        }
        else
        {
            v.x=datleAngleVector[0];
            v.y=datleAngleVector[1];
            v.z=datleAngleVector[2];
        }


        points.push_back(p);
        vector.push_back(v);

    }
    points.push_back(pLeft);
    vector.push_back({avoidVectorLeft.x(),avoidVectorLeft.y(),avoidVectorLeft.z()});

    return 0;
}

/**********************************************
* @projectName   ThiVision::movePointsAxial
* @brief         把路径点和姿态向量按照圆柱轴向平移
* @inParam
*                length: 平移的长度
*                cylinder: 圆柱模型
*                inPoints: 输入的路径点
*                inVector: 输入的姿态向量
* @outParam
*                outPoints: 输出的路径点
*                outVector: 输出的姿态向量
* @return        成功返回0,失败返回-1
* @author        xs(1874020422@qq.com)
* @date          2024-08-09
**********************************************/
int ThiVision::movePointsAxial(float length, pcl::ModelCoefficients::Ptr cylinder, std::vector<PointT> inPoints, std::vector<PointT> inVector, std::vector<PointT> &outPoints, std::vector<PointT> &outVector)
{
    if(cylinder->values.size()!=7)
        return -1;
    Eigen::Vector3f vectorAxial{cylinder->values[3],cylinder->values[4],cylinder->values[5]};
    if (vectorAxial[0]<0)
    {
        vectorAxial=-vectorAxial;
    }
    for (unsigned long i=0;i<inPoints.size();i++)
    {
        PointT p;
        p.x=inPoints[i].x+length*vectorAxial[0];
        p.y=inPoints[i].y+length*vectorAxial[1];
        p.z=inPoints[i].z+length*vectorAxial[2];
        outPoints.push_back(p);
    }
    for(unsigned long i=0;i<inVector.size();i++)
    {
        outVector.push_back(inVector[i]);
    }
    return 0;
}

/**********************************************
* @projectName   ThiVision::getPointLineDistance
* @brief         得到点到直线的距离
* @inParam
*                point: 输入的一点
*                line: 输入的一条直线
* @outParam
*                distance: 求得的距离
* @return        成功返回0,否则返回-1
* @author        xs(1874020422@qq.com)
* @date          2024-08-09
**********************************************/
int ThiVision::getPointLineDistance(const PointT point,pcl::ModelCoefficients::Ptr line,float &distance)
{
    if(line->values.size()!=6)
        return -1;
    Eigen::Vector3f pn{line->values[0]-point.x,line->values[1]-point.y,line->values[2]-point.z};
    Eigen::Vector3f vn{line->values[3],line->values[4],line->values[5]};
    Eigen::Vector3f vd=pn.cross(vn);
    distance=vd.norm()/vn.norm();
    return 0;
}

/**********************************************
* @projectName   ThiVision::getPointPointDistance
* @brief         得到两点之间的距离
* @inParam
*                point1: 点一
*                point2: 点二
* @outParam
*                distance: 两点间的距离
* @return        成功返回0
* @author        xs(1874020422@qq.com)
* @date          2024-08-10
**********************************************/
int ThiVision::getPointPointDistance(const PointT point1, const PointT point2, float &distance)
{
    distance =sqrt((point1.x-point2.x)*(point1.x-point2.x)+(point1.y-point2.y)*(point1.y-point2.y)+(point1.z-point2.z)*(point1.z-point2.z));
    return 0;
}

/**********************************************
* @projectName   ThiVision::getRotatedVector
* @brief         将原始向量绕某一轴向量旋转angle度
* @inParam
*                original: 原始向量
*                axis: 轴向量
*                angle: 旋转角度(弧度制)
* @outParam      void
* @return        旋转后的向量
* @author        xs(1874020422@qq.com)
* @date          2024-08-09
**********************************************/
Eigen::Vector3f ThiVision::getRotatedVector(const Eigen::Vector3f &original, const Eigen::Vector3f &axis, double angle)
{
    Eigen::Vector3f u = axis.normalized();
    double cosAngle = cos(angle);
    double sinAngle = sin(angle);
    Eigen::Matrix3f K;
    K << 0, -u.z(), u.y(),
            u.z(), 0, -u.x(),
            -u.y(), u.x(), 0;
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity() + sinAngle * K + (1 - cosAngle) * K * K;
    return (R*original).normalized();
}

/**********************************************
* @projectName   ThiVision::sortAreaPointCloud
* @brief         对点云按照xoy平面逆时针排序
* @inParam
*                cloudIn: 输入的点云
* @outParam
*                cloudIn: 直接修改输入的点云为排序后的点云
* @return        成功返回0,失败返回-1
* @author        xs(1874020422@qq.com)
* @date          2024-08-09
**********************************************/
int ThiVision::sortAreaPointCloud(pcl::PointCloud<PointT>::Ptr &cloudIn)
{
    if(cloudIn->size()!=4)
        return -1;

    unsigned long indexMaxX[4]={0,1,2,3};
    for(unsigned long i=0;i<cloudIn->size()-1;i++)
    {
        for (unsigned long j=0;j<cloudIn->size()-i-1;j++)
        {
            if(cloudIn->points[indexMaxX[j]].x>cloudIn->points[indexMaxX[j+1]].x)
            {
                unsigned long tempIndex=indexMaxX[j];
                indexMaxX[j]=indexMaxX[j+1];
                indexMaxX[j+1]=tempIndex;
            }
        }
    }

    if(cloudIn->points[indexMaxX[0]].y<cloudIn->points[indexMaxX[1]].y)
    {
        unsigned long tempIndex=indexMaxX[0];
        indexMaxX[0]=indexMaxX[1];
        indexMaxX[1]=tempIndex;
    }

    if(cloudIn->points[indexMaxX[2]].y>cloudIn->points[indexMaxX[3]].y)
    {
        unsigned long tempIndex=indexMaxX[2];
        indexMaxX[2]=indexMaxX[3];
        indexMaxX[3]=tempIndex;
    }
    pcl::PointCloud<PointT>::Ptr Temp(new pcl::PointCloud<PointT>);
    for (int i=0;i<4;i++)
    {
        Temp->push_back(cloudIn->points[indexMaxX[i]]);
    }
    cloudIn->clear();
    cloudIn=Temp;
    return 0;
    //x由小到大
    //y由大到小,再由小到大
    //      ^x   3 2
    //      |    0 1
    //  <---|y

    //    0、1总体y < 3、2；0 x>1 x  3 x>2 x

}

/**********************************************
* @projectName   ThiVision::getTrackKeyPoints
* @brief         通过对铸造字区域的划分将打磨区域的路径关键点分离出来
* @inParam
*                wordAreaPoints: 多片铸造字区域点云
*                cylinder: 圆柱模型参数
*                line: 截面基础线
*                length: 可打磨的最长区域
*                points: 输入的基础路径点集
*                vectors: 输入的基础路径方向向量
* @outParam
*                listOutPoints: 输出的路径点集列表(一片铸造字 会有3份点集 0是正向区域字路径点 1是反向边界点 2是反向区域字路径点)
*                listOutVectors: 输出的路径方向向量集列表(一片铸造字 会有3份方向向量集 0是正向区域字路径点方向向量 1是反向边界点方向向量 2是反向区域字路径点方向向量)
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-13
**********************************************/
int ThiVision::getTrackKeyPoints(std::vector<pcl::PointCloud<PointT>::Ptr> wordAreaPoints, pcl::ModelCoefficients::Ptr cylinder, pcl::ModelCoefficients::Ptr line,float length,std::vector<PointT> points,std::vector<PointT> vectors,std::vector<std::vector<PointT>> &listOutPoints,std::vector<std::vector<PointT>> &listOutVectors)
{


    if(cylinder->values.size()!=7||line->values.size()!=6)
        return -1;

    Eigen::Vector3f vCy{cylinder->values[3],cylinder->values[4],cylinder->values[5]};
    vCy=vCy.normalized();

    if(vCy[0]<0)
        vCy=-vCy;
    std::vector<PointT> outPoints1,outVector1;

    //有区域
    if(wordAreaPoints.size()!=0)
    {
        std::vector<PointT> outPoints2,outVector2;
        std::vector<PointT> outPoints3,outVector3;

        //截面半径向量
        Eigen::Vector3f nLine{line->values[3],line->values[4],line->values[5]};
        nLine=nLine.normalized();
        Eigen::Vector3f nR=vCy.cross(nLine).normalized();

        //points投影
        std::vector<PointT> projectPoints;
        for (unsigned long i=0;i<points.size();i++)
        {
            pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
            lI->values.resize(6);
            lI->values[0]=points[i].x;
            lI->values[1]=points[i].y;
            lI->values[2]=points[i].z;
            lI->values[3]=nR[0];
            lI->values[4]=nR[1];
            lI->values[5]=nR[2];

            PointT intersectP;
            getLineLineIntersectPoint(lI,line,intersectP);
            projectPoints.push_back(intersectP);
        }

        //反向points投影
        std::vector<PointT> reverseProjectPoints;
        for (unsigned long i=0;i<projectPoints.size();i++)
        {
            PointT reverseProjectPoint;
            reverseProjectPoint.x=projectPoints[i].x+length*vCy[0];
            reverseProjectPoint.y=projectPoints[i].y+length*vCy[1];
            reverseProjectPoint.z=projectPoints[i].z+length*vCy[2];
            reverseProjectPoints.push_back(reverseProjectPoint);
        }

        //反向基准线
        pcl::ModelCoefficients::Ptr reverseLine(new pcl::ModelCoefficients);
        reverseLine->values.resize(6);
        reverseLine->values[0]=line->values[0]+length*vCy[0];
        reverseLine->values[1]=line->values[1]+length*vCy[1];
        reverseLine->values[2]=line->values[2]+length*vCy[2];
        reverseLine->values[3]=line->values[3];
        reverseLine->values[4]=line->values[4];
        reverseLine->values[5]=line->values[5];

        //正向
        pcl::PointCloud<PointT>::Ptr forwardWord(new pcl::PointCloud<PointT>);
        //反向
        pcl::PointCloud<PointT>::Ptr reverseWord(new pcl::PointCloud<PointT>);

        for (unsigned long i=0;i<wordAreaPoints[0]->size();i++)
        {
            PointT P=wordAreaPoints[0]->points[i];
            forwardWord->push_back(P);
        }
        for (unsigned long i=0;i<wordAreaPoints[0]->size();i++)
        {
            PointT P=wordAreaPoints[0]->points[3-i];
            reverseWord->push_back(P);
        }


        //4个交点
        PointT forwardWordIntersectPoint[4];
        PointT reverseWordIntersectPoint[4];
        for (unsigned long i=0;i<4;i++)
        {
            pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
            lI->values.resize(6);
            lI->values[0]=forwardWord->points[i].x;
            lI->values[1]=forwardWord->points[i].y;
            lI->values[2]=forwardWord->points[i].z;
            lI->values[3]=vCy[0];
            lI->values[4]=vCy[1];
            lI->values[5]=vCy[2];

            getLineLineIntersectPoint(lI,line,forwardWordIntersectPoint[i]);
        }

        for (unsigned long i=0;i<4;i++)
        {
            pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
            lI->values.resize(6);
            lI->values[0]=reverseWord->points[i].x;
            lI->values[1]=reverseWord->points[i].y;
            lI->values[2]=reverseWord->points[i].z;
            lI->values[3]=vCy[0];
            lI->values[4]=vCy[1];
            lI->values[5]=vCy[2];

            getLineLineIntersectPoint(lI,reverseLine,reverseWordIntersectPoint[i]);
        }

        //得到最大投影点距离对应的下标maxIndex1>maxIndex0
        float maxD=0;
        unsigned long maxIndex0=0,maxIndex1=0;
        for(unsigned long i=0;i<4;i++)
        {
            for(unsigned long j=i+1;j<4;j++)
            {
                float curD;
                getPointPointDistance(forwardWordIntersectPoint[i],forwardWordIntersectPoint[j],curD);
                if(curD>maxD)
                {
                    maxIndex0=i;
                    maxIndex1=j;
                    //1>0
                    if(maxIndex0>maxIndex1)
                    {
                        unsigned long temp=maxIndex0;
                        maxIndex0=maxIndex1;
                        maxIndex1=temp;
                    }
                    maxD=curD;
                }
            }

        }

        float reverseMaxD=0;
        unsigned long reverseMaxIndex0=0,reverseMaxIndex1=0;
        for(unsigned long i=0;i<4;i++)
        {
            for(unsigned long j=i+1;j<4;j++)
            {
                float curD;
                getPointPointDistance(reverseWordIntersectPoint[i],reverseWordIntersectPoint[j],curD);
                if(curD>reverseMaxD)
                {
                    reverseMaxIndex0=i;
                    reverseMaxIndex1=j;
                    //1>0
                    if(reverseMaxIndex0>reverseMaxIndex1)
                    {
                        unsigned long temp=reverseMaxIndex0;
                        reverseMaxIndex0=reverseMaxIndex1;
                        reverseMaxIndex1=temp;
                    }
                    reverseMaxD=curD;
                }
            }

        }



        //得到区域类型和对应的关键点下标
        unsigned long sumIndex=maxIndex0+maxIndex1;
        if(sumIndex==1)//1、0号点
        {
            forwardWordArea.type='A';
            PointT p0=forwardWordIntersectPoint[0];
            PointT p1=forwardWordIntersectPoint[1];
            Eigen::Vector3f vBefor0{projectPoints[0].x-p0.x,projectPoints[0].y-p0.y,projectPoints[0].z-p0.z};
            Eigen::Vector3f vBefor1{projectPoints[0].x-p1.x,projectPoints[0].y-p1.y,projectPoints[0].z-p1.z};

            //1
            for(unsigned long i=0;i<projectPoints.size();i++)
            {
                Eigen::Vector3f vCur{projectPoints[i].x-p1.x,projectPoints[i].y-p1.y,projectPoints[i].z-p1.z};

                if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                {
                    forwardWordArea.index[0]=i;
                    break;
                }
                if(std::acos(vCur.normalized().dot(vBefor1.normalized()))>static_cast<float>(M_PI/2))
                {
                    break;
                }
                if(vCur.norm()<=vBefor1.norm())
                {
                    forwardWordArea.index[0]=i;
                }
                vBefor1=vCur;
            }

            //0
            for(unsigned long i=0;i<projectPoints.size();i++)
            {
                Eigen::Vector3f vCur{projectPoints[i].x-p0.x,projectPoints[i].y-p0.y,projectPoints[i].z-p0.z};

                if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                {
                    forwardWordArea.index[1]=i;
                    break;
                }
                if(std::acos(vCur.normalized().dot(vBefor0.normalized()))>static_cast<float>(M_PI/2))
                {
                    forwardWordArea.index[1]=i;
                    break;
                }
                if(vCur.norm()<=vBefor0.norm())
                {
                    forwardWordArea.index[1]=i;
                }
                vBefor0=vCur;
            }



        }
        else if(sumIndex==2||sumIndex==4)
        {
            if(sumIndex==2)//2、1、0号点
            {
                forwardWordArea.type='B';
                PointT p0=forwardWordIntersectPoint[0];
                PointT p1=forwardWordIntersectPoint[1];
                PointT p2=forwardWordIntersectPoint[2];
                Eigen::Vector3f vBefor0{projectPoints[0].x-p0.x,projectPoints[0].y-p0.y,projectPoints[0].z-p0.z};
                Eigen::Vector3f vBefor1{projectPoints[0].x-p1.x,projectPoints[0].y-p1.y,projectPoints[0].z-p1.z};
                Eigen::Vector3f vBefor2{projectPoints[0].x-p2.x,projectPoints[0].y-p2.y,projectPoints[0].z-p2.z};
                //2
                for(unsigned long i=0;i<projectPoints.size();i++)
                {
                    Eigen::Vector3f vCur{projectPoints[i].x-p2.x,projectPoints[i].y-p2.y,projectPoints[i].z-p2.z};

                    if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                    {
                        forwardWordArea.index[0]=i;
                        break;
                    }
                    if(std::acos(vCur.normalized().dot(vBefor2.normalized()))>static_cast<float>(M_PI/2))
                    {
                        break;
                    }
                    if(vCur.norm()<=vBefor2.norm())
                    {
                        forwardWordArea.index[0]=i;
                    }
                    vBefor2=vCur;

                }

                //1
                for(unsigned long i=0;i<projectPoints.size();i++)
                {
                    Eigen::Vector3f vCur{projectPoints[i].x-p1.x,projectPoints[i].y-p1.y,projectPoints[i].z-p1.z};

                    if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                    {
                        forwardWordArea.index[1]=i;
                        break;
                    }
                    if(std::acos(vCur.normalized().dot(vBefor1.normalized()))>static_cast<float>(M_PI/2))
                    {
                        forwardWordArea.index[1]=i;
                        break;
                    }
                    if(vCur.norm()<=vBefor1.norm())
                    {
                        forwardWordArea.index[1]=i;
                    }
                    vBefor1=vCur;
                }

                //0
                for(unsigned long i=0;i<projectPoints.size();i++)
                {
                    Eigen::Vector3f vCur{projectPoints[i].x-p0.x,projectPoints[i].y-p0.y,projectPoints[i].z-p0.z};

                    if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                    {
                        forwardWordArea.index[2]=i;
                        break;
                    }
                    if(std::acos(vCur.normalized().dot(vBefor0.normalized()))>static_cast<float>(M_PI/2))
                    {
                        forwardWordArea.index[2]=i;
                        break;
                    }
                    if(vCur.norm()<=vBefor0.norm())
                    {
                        forwardWordArea.index[2]=i;
                    }
                    vBefor0=vCur;
                }

            }
            else//1、0、3号点
            {
                forwardWordArea.type='C';
                PointT p0=forwardWordIntersectPoint[0];
                PointT p1=forwardWordIntersectPoint[1];
                PointT p3=forwardWordIntersectPoint[3];
                Eigen::Vector3f vBefor0{projectPoints[0].x-p0.x,projectPoints[0].y-p0.y,projectPoints[0].z-p0.z};
                Eigen::Vector3f vBefor1{projectPoints[0].x-p1.x,projectPoints[0].y-p1.y,projectPoints[0].z-p1.z};
                Eigen::Vector3f vBefor3{projectPoints[0].x-p3.x,projectPoints[0].y-p3.y,projectPoints[0].z-p3.z};

                //1
                for(unsigned long i=0;i<projectPoints.size();i++)
                {
                    Eigen::Vector3f vCur{projectPoints[i].x-p1.x,projectPoints[i].y-p1.y,projectPoints[i].z-p1.z};

                    if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                    {
                        forwardWordArea.index[0]=i;
                        break;
                    }
                    if(std::acos(vCur.normalized().dot(vBefor1.normalized()))>static_cast<float>(M_PI/2))
                    {
                        break;
                    }
                    if(vCur.norm()<=vBefor1.norm())
                    {
                        forwardWordArea.index[0]=i;
                    }
                    vBefor1=vCur;

                }

                //0
                for(unsigned long i=0;i<projectPoints.size();i++)
                {
                    Eigen::Vector3f vCur{projectPoints[i].x-p0.x,projectPoints[i].y-p0.y,projectPoints[i].z-p0.z};

                    if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                    {
                        forwardWordArea.index[1]=i;
                        break;
                    }
                    if(std::acos(vCur.normalized().dot(vBefor0.normalized()))>static_cast<float>(M_PI/2))
                    {
                        forwardWordArea.index[1]=i;
                        break;
                    }
                    if(vCur.norm()<=vBefor0.norm())
                    {
                        forwardWordArea.index[1]=i;
                    }
                    vBefor0=vCur;

                }

                //3
                for(unsigned long i=0;i<projectPoints.size();i++)
                {
                    Eigen::Vector3f vCur{projectPoints[i].x-p3.x,projectPoints[i].y-p3.y,projectPoints[i].z-p3.z};

                    if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                    {
                        forwardWordArea.index[2]=i;
                        break;
                    }
                    if(std::acos(vCur.normalized().dot(vBefor3.normalized()))>static_cast<float>(M_PI/2))
                    {
                        forwardWordArea.index[2]=i;
                        break;
                    }
                    if(vCur.norm()<=vBefor3.norm())
                    {
                        forwardWordArea.index[2]=i;
                    }
                    vBefor3=vCur;

                }
            }
        }
        else if(sumIndex==5)//2、1、0、3
        {
            forwardWordArea.type='D';
            PointT p0=forwardWordIntersectPoint[0];
            PointT p1=forwardWordIntersectPoint[1];
            PointT p2=forwardWordIntersectPoint[2];
            PointT p3=forwardWordIntersectPoint[3];
            Eigen::Vector3f vBefor0{projectPoints[0].x-p0.x,projectPoints[0].y-p0.y,projectPoints[0].z-p0.z};
            Eigen::Vector3f vBefor1{projectPoints[0].x-p1.x,projectPoints[0].y-p1.y,projectPoints[0].z-p1.z};
            Eigen::Vector3f vBefor2{projectPoints[0].x-p2.x,projectPoints[0].y-p2.y,projectPoints[0].z-p2.z};
            Eigen::Vector3f vBefor3{projectPoints[0].x-p3.x,projectPoints[0].y-p3.y,projectPoints[0].z-p3.z};

            //2
            for(unsigned long i=0;i<projectPoints.size();i++)
            {
                Eigen::Vector3f vCur{projectPoints[i].x-p2.x,projectPoints[i].y-p2.y,projectPoints[i].z-p2.z};

                if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                {
                    forwardWordArea.index[0]=i;
                    break;
                }
                if(std::acos(vCur.normalized().dot(vBefor2.normalized()))>static_cast<float>(M_PI/2))
                {
                    break;
                }
                if(vCur.norm()<=vBefor2.norm())
                {
                    forwardWordArea.index[0]=i;
                }
                vBefor2=vCur;
            }

            //1
            for(unsigned long i=0;i<projectPoints.size();i++)
            {
                Eigen::Vector3f vCur{projectPoints[i].x-p1.x,projectPoints[i].y-p1.y,projectPoints[i].z-p1.z};

                if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                {
                    forwardWordArea.index[1]=i;
                    break;
                }
                if(std::acos(vCur.normalized().dot(vBefor1.normalized()))>static_cast<float>(M_PI/2))
                {
                    forwardWordArea.index[1]=i;
                    break;
                }
                if(vCur.norm()<=vBefor1.norm())
                {
                    forwardWordArea.index[1]=i;
                }
                vBefor1=vCur;
            }

            //0
            for(unsigned long i=0;i<projectPoints.size();i++)
            {
                Eigen::Vector3f vCur{projectPoints[i].x-p0.x,projectPoints[i].y-p0.y,projectPoints[i].z-p0.z};

                if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                {
                    forwardWordArea.index[2]=i;
                    break;
                }
                if(std::acos(vCur.normalized().dot(vBefor0.normalized()))>static_cast<float>(M_PI/2))
                {
                    forwardWordArea.index[2]=i;
                    break;
                }
                if(vCur.norm()<=vBefor0.norm())
                {
                    forwardWordArea.index[2]=i;
                }
                vBefor0=vCur;
            }

            //3
            for(unsigned long i=0;i<projectPoints.size();i++)
            {
                Eigen::Vector3f vCur{projectPoints[i].x-p3.x,projectPoints[i].y-p3.y,projectPoints[i].z-p3.z};

                if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                {
                    forwardWordArea.index[3]=i;
                    break;
                }
                if(std::acos(vCur.normalized().dot(vBefor3.normalized()))>static_cast<float>(M_PI/2))
                {
                    forwardWordArea.index[3]=i;
                    break;
                }
                if(vCur.norm()<=vBefor3.norm())
                {
                    forwardWordArea.index[3]=i;
                }
                vBefor3=vCur;

            }


        }
        else
        {
            return -2;
        }

        unsigned long reverseSumIndex=reverseMaxIndex0+reverseMaxIndex1;
        if(reverseSumIndex==1)//1、0号点
        {
            reverseWordArea.type='A';
            PointT p0=reverseWordIntersectPoint[0];
            PointT p1=reverseWordIntersectPoint[1];
            Eigen::Vector3f vBefor0{reverseProjectPoints[0].x-p0.x,reverseProjectPoints[0].y-p0.y,reverseProjectPoints[0].z-p0.z};
            Eigen::Vector3f vBefor1{reverseProjectPoints[0].x-p1.x,reverseProjectPoints[0].y-p1.y,reverseProjectPoints[0].z-p1.z};

            //1
            for(unsigned long i=0;i<reverseProjectPoints.size();i++)
            {
                Eigen::Vector3f vCur{reverseProjectPoints[i].x-p1.x,reverseProjectPoints[i].y-p1.y,reverseProjectPoints[i].z-p1.z};

                if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                {
                    reverseWordArea.index[0]=i;
                    break;
                }
                if(std::acos(vCur.normalized().dot(vBefor1.normalized()))>static_cast<float>(M_PI/2))
                {
                    break;
                }
                if(vCur.norm()<=vBefor1.norm())
                {
                    reverseWordArea.index[0]=i;
                }
                vBefor1=vCur;
            }

            //0
            for(unsigned long i=0;i<reverseProjectPoints.size();i++)
            {
                Eigen::Vector3f vCur{reverseProjectPoints[i].x-p0.x,reverseProjectPoints[i].y-p0.y,reverseProjectPoints[i].z-p0.z};

                if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                {
                    reverseWordArea.index[1]=i;
                    break;
                }
                if(std::acos(vCur.normalized().dot(vBefor0.normalized()))>static_cast<float>(M_PI/2))
                {
                    reverseWordArea.index[1]=i;
                    break;
                }
                if(vCur.norm()<=vBefor0.norm())
                {
                    reverseWordArea.index[1]=i;
                }
                vBefor0=vCur;
            }



        }
        else if(reverseSumIndex==2||reverseSumIndex==4)
        {
            if(reverseSumIndex==2)//2、1、0号点
            {
                reverseWordArea.type='B';
                PointT p0=reverseWordIntersectPoint[0];
                PointT p1=reverseWordIntersectPoint[1];
                PointT p2=reverseWordIntersectPoint[2];
                Eigen::Vector3f vBefor0{reverseProjectPoints[0].x-p0.x,reverseProjectPoints[0].y-p0.y,reverseProjectPoints[0].z-p0.z};
                Eigen::Vector3f vBefor1{reverseProjectPoints[0].x-p1.x,reverseProjectPoints[0].y-p1.y,reverseProjectPoints[0].z-p1.z};
                Eigen::Vector3f vBefor2{reverseProjectPoints[0].x-p2.x,reverseProjectPoints[0].y-p2.y,reverseProjectPoints[0].z-p2.z};
                //2
                for(unsigned long i=0;i<projectPoints.size();i++)
                {
                    Eigen::Vector3f vCur{reverseProjectPoints[i].x-p2.x,reverseProjectPoints[i].y-p2.y,reverseProjectPoints[i].z-p2.z};

                    if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                    {
                        reverseWordArea.index[0]=i;
                        break;
                    }
                    if(std::acos(vCur.normalized().dot(vBefor2.normalized()))>static_cast<float>(M_PI/2))
                    {
                        break;
                    }
                    if(vCur.norm()<=vBefor2.norm())
                    {
                        reverseWordArea.index[0]=i;
                    }
                    vBefor2=vCur;

                }

                //1
                for(unsigned long i=0;i<reverseProjectPoints.size();i++)
                {
                    Eigen::Vector3f vCur{reverseProjectPoints[i].x-p1.x,reverseProjectPoints[i].y-p1.y,reverseProjectPoints[i].z-p1.z};

                    if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                    {
                        reverseWordArea.index[1]=i;
                        break;
                    }
                    if(std::acos(vCur.normalized().dot(vBefor1.normalized()))>static_cast<float>(M_PI/2))
                    {
                        reverseWordArea.index[1]=i;
                        break;
                    }
                    if(vCur.norm()<=vBefor1.norm())
                    {
                        reverseWordArea.index[1]=i;
                    }
                    vBefor1=vCur;
                }

                //0
                for(unsigned long i=0;i<reverseProjectPoints.size();i++)
                {
                    Eigen::Vector3f vCur{reverseProjectPoints[i].x-p0.x,reverseProjectPoints[i].y-p0.y,reverseProjectPoints[i].z-p0.z};

                    if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                    {
                        reverseWordArea.index[2]=i;
                        break;
                    }
                    if(std::acos(vCur.normalized().dot(vBefor0.normalized()))>static_cast<float>(M_PI/2))
                    {
                        reverseWordArea.index[2]=i;
                        break;
                    }
                    if(vCur.norm()<=vBefor0.norm())
                    {
                        reverseWordArea.index[2]=i;
                    }
                    vBefor0=vCur;
                }

            }
            else//1、0、3号点
            {
                reverseWordArea.type='C';
                PointT p0=reverseWordIntersectPoint[0];
                PointT p1=reverseWordIntersectPoint[1];
                PointT p3=reverseWordIntersectPoint[3];
                Eigen::Vector3f vBefor0{reverseProjectPoints[0].x-p0.x,reverseProjectPoints[0].y-p0.y,reverseProjectPoints[0].z-p0.z};
                Eigen::Vector3f vBefor1{reverseProjectPoints[0].x-p1.x,reverseProjectPoints[0].y-p1.y,reverseProjectPoints[0].z-p1.z};
                Eigen::Vector3f vBefor3{reverseProjectPoints[0].x-p3.x,reverseProjectPoints[0].y-p3.y,reverseProjectPoints[0].z-p3.z};

                //1
                for(unsigned long i=0;i<reverseProjectPoints.size();i++)
                {
                    Eigen::Vector3f vCur{reverseProjectPoints[i].x-p1.x,reverseProjectPoints[i].y-p1.y,reverseProjectPoints[i].z-p1.z};

                    if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                    {
                        reverseWordArea.index[0]=i;
                        break;
                    }
                    if(std::acos(vCur.normalized().dot(vBefor1.normalized()))>static_cast<float>(M_PI/2))
                    {
                        break;
                    }
                    if(vCur.norm()<=vBefor1.norm())
                    {
                        reverseWordArea.index[0]=i;
                    }
                    vBefor1=vCur;

                }

                //0
                for(unsigned long i=0;i<reverseProjectPoints.size();i++)
                {
                    Eigen::Vector3f vCur{reverseProjectPoints[i].x-p0.x,reverseProjectPoints[i].y-p0.y,reverseProjectPoints[i].z-p0.z};

                    if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                    {
                        reverseWordArea.index[1]=i;
                        break;
                    }
                    if(std::acos(vCur.normalized().dot(vBefor0.normalized()))>static_cast<float>(M_PI/2))
                    {
                        reverseWordArea.index[1]=i;
                        break;
                    }
                    if(vCur.norm()<=vBefor0.norm())
                    {
                        reverseWordArea.index[1]=i;
                    }
                    vBefor0=vCur;

                }

                //3
                for(unsigned long i=0;i<reverseProjectPoints.size();i++)
                {
                    Eigen::Vector3f vCur{reverseProjectPoints[i].x-p3.x,reverseProjectPoints[i].y-p3.y,reverseProjectPoints[i].z-p3.z};

                    if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                    {
                        reverseWordArea.index[2]=i;
                        break;
                    }
                    if(std::acos(vCur.normalized().dot(vBefor3.normalized()))>static_cast<float>(M_PI/2))
                    {
                        reverseWordArea.index[2]=i;
                        break;
                    }
                    if(vCur.norm()<=vBefor3.norm())
                    {
                        reverseWordArea.index[2]=i;
                    }
                    vBefor3=vCur;

                }
            }
        }
        else if(reverseSumIndex==5)//2、1、0、3
        {
            reverseWordArea.type='D';
            PointT p0=reverseWordIntersectPoint[0];
            PointT p1=reverseWordIntersectPoint[1];
            PointT p2=reverseWordIntersectPoint[2];
            PointT p3=reverseWordIntersectPoint[3];
            Eigen::Vector3f vBefor0{reverseProjectPoints[0].x-p0.x,reverseProjectPoints[0].y-p0.y,reverseProjectPoints[0].z-p0.z};
            Eigen::Vector3f vBefor1{reverseProjectPoints[0].x-p1.x,reverseProjectPoints[0].y-p1.y,reverseProjectPoints[0].z-p1.z};
            Eigen::Vector3f vBefor2{reverseProjectPoints[0].x-p2.x,reverseProjectPoints[0].y-p2.y,reverseProjectPoints[0].z-p2.z};
            Eigen::Vector3f vBefor3{reverseProjectPoints[0].x-p3.x,reverseProjectPoints[0].y-p3.y,reverseProjectPoints[0].z-p3.z};

            //2
            for(unsigned long i=0;i<reverseProjectPoints.size();i++)
            {
                Eigen::Vector3f vCur{reverseProjectPoints[i].x-p2.x,reverseProjectPoints[i].y-p2.y,reverseProjectPoints[i].z-p2.z};

                if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                {
                    reverseWordArea.index[0]=i;
                    break;
                }
                if(std::acos(vCur.normalized().dot(vBefor2.normalized()))>static_cast<float>(M_PI/2))
                {
                    break;
                }
                if(vCur.norm()<=vBefor2.norm())
                {
                    reverseWordArea.index[0]=i;
                }
                vBefor2=vCur;
            }

            //1
            for(unsigned long i=0;i<reverseProjectPoints.size();i++)
            {
                Eigen::Vector3f vCur{reverseProjectPoints[i].x-p1.x,reverseProjectPoints[i].y-p1.y,reverseProjectPoints[i].z-p1.z};

                if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                {
                    reverseWordArea.index[1]=i;
                    break;
                }
                if(std::acos(vCur.normalized().dot(vBefor1.normalized()))>static_cast<float>(M_PI/2))
                {
                    reverseWordArea.index[1]=i;
                    break;
                }
                if(vCur.norm()<=vBefor1.norm())
                {
                    reverseWordArea.index[1]=i;
                }
                vBefor1=vCur;
            }

            //0
            for(unsigned long i=0;i<reverseProjectPoints.size();i++)
            {
                Eigen::Vector3f vCur{reverseProjectPoints[i].x-p0.x,reverseProjectPoints[i].y-p0.y,reverseProjectPoints[i].z-p0.z};

                if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                {
                    reverseWordArea.index[2]=i;
                    break;
                }
                if(std::acos(vCur.normalized().dot(vBefor0.normalized()))>static_cast<float>(M_PI/2))
                {
                    reverseWordArea.index[2]=i;
                    break;
                }
                if(vCur.norm()<=vBefor0.norm())
                {
                    reverseWordArea.index[2]=i;
                }
                vBefor0=vCur;
            }

            //3
            for(unsigned long i=0;i<reverseProjectPoints.size();i++)
            {
                Eigen::Vector3f vCur{reverseProjectPoints[i].x-p3.x,reverseProjectPoints[i].y-p3.y,reverseProjectPoints[i].z-p3.z};

                if(floatIsEqual(vCur[0],0)&&floatIsEqual(vCur[1],0)&&floatIsEqual(vCur[2],0))
                {
                    reverseWordArea.index[3]=i;
                    break;
                }
                if(std::acos(vCur.normalized().dot(vBefor3.normalized()))>static_cast<float>(M_PI/2))
                {
                    reverseWordArea.index[3]=i;
                    break;
                }
                if(vCur.norm()<=vBefor3.norm())
                {
                    reverseWordArea.index[3]=i;
                }
                vBefor3=vCur;

            }


        }
        else
        {
            return -2;
        }

        //std::cout<<forwardWordArea.type<<"  "<<reverseWordArea.type<<std::endl;
        //通过交点求关键点
        if(forwardWordArea.type=='A')
        {
            //关键0、1点构成的线
            pcl::ModelCoefficients::Ptr lP01(new pcl::ModelCoefficients);
            pcl::ModelCoefficients::Ptr lP23(new pcl::ModelCoefficients);
            Eigen::Vector3f vLP01{forwardWord->points[0].x-forwardWord->points[1].x,forwardWord->points[0].y-forwardWord->points[1].y,forwardWord->points[0].z-forwardWord->points[1].z};
            Eigen::Vector3f vLP23{forwardWord->points[2].x-forwardWord->points[3].x,forwardWord->points[2].y-forwardWord->points[3].y,forwardWord->points[2].z-forwardWord->points[3].z};
            lP01->values.resize(6);
            lP23->values.resize(6);
            vLP01=vLP01.normalized();
            vLP23=vLP23.normalized();

            lP01->values[0]=forwardWord->points[0].x;
            lP01->values[1]=forwardWord->points[0].y;
            lP01->values[2]=forwardWord->points[0].z;
            lP01->values[3]=vLP01[0];
            lP01->values[4]=vLP01[1];
            lP01->values[5]=vLP01[2];

            lP23->values[0]=forwardWord->points[2].x;
            lP23->values[1]=forwardWord->points[2].y;
            lP23->values[2]=forwardWord->points[2].z;
            lP23->values[3]=vLP23[0];
            lP23->values[4]=vLP23[1];
            lP23->values[5]=vLP23[2];

            for (unsigned long i=0;i<points.size();i++)
            {
                if((forwardWordArea.index[0]<=i)&&(i<=forwardWordArea.index[1]))
                {
                    //第i条直线
                    pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                    lI->values.resize(6);
                    lI->values[0]=projectPoints[i].x;
                    lI->values[1]=projectPoints[i].y;
                    lI->values[2]=projectPoints[i].z;
                    lI->values[3]=vCy[0];
                    lI->values[4]=vCy[1];
                    lI->values[5]=vCy[2];

                    //求第i个交点
                    PointT pI,p;
                    float dI,d;
                    getLineLineIntersectPoint(lP01,lI,pI);
                    getPointPointDistance(pI,projectPoints[i],dI);

                    getLineLineIntersectPoint(lP23,lI,p);
                    getPointPointDistance(p,projectPoints[i],d);
                    if(dI>d)
                    {
                        dI=length;
                    }

                    PointT pOutI;
                    pOutI.x=points[i].x+dI*vCy[0];
                    pOutI.y=points[i].y+dI*vCy[1];
                    pOutI.z=points[i].z+dI*vCy[2];


                    outPoints1.push_back(pOutI);
                    outVector1.push_back(vectors[i]);

                }
                else
                {
                    PointT pOutI;
                    pOutI.x=points[i].x+length*vCy[0];
                    pOutI.y=points[i].y+length*vCy[1];
                    pOutI.z=points[i].z+length*vCy[2];

                    outPoints1.push_back(pOutI);
                    outVector1.push_back(vectors[i]);
                }
            }
        }
        else if(forwardWordArea.type=='B'||forwardWordArea.type=='C')
        {
            if(forwardWordArea.type=='B')
            {
                //关键0、1、2点构成的线
                pcl::ModelCoefficients::Ptr lP01(new pcl::ModelCoefficients);
                pcl::ModelCoefficients::Ptr lP12(new pcl::ModelCoefficients);
                pcl::ModelCoefficients::Ptr lP23(new pcl::ModelCoefficients);
                Eigen::Vector3f vLP01{forwardWord->points[0].x-forwardWord->points[1].x,forwardWord->points[0].y-forwardWord->points[1].y,forwardWord->points[0].z-forwardWord->points[1].z};
                Eigen::Vector3f vLP12{forwardWord->points[1].x-forwardWord->points[2].x,forwardWord->points[1].y-forwardWord->points[2].y,forwardWord->points[1].z-forwardWord->points[2].z};
                Eigen::Vector3f vLP23{forwardWord->points[2].x-forwardWord->points[3].x,forwardWord->points[2].y-forwardWord->points[3].y,forwardWord->points[2].z-forwardWord->points[3].z};
                lP01->values.resize(6);
                lP12->values.resize(6);
                lP23->values.resize(6);
                vLP01=vLP01.normalized();
                vLP12=vLP12.normalized();
                vLP23=vLP23.normalized();

                lP01->values[0]=forwardWord->points[0].x;
                lP01->values[1]=forwardWord->points[0].y;
                lP01->values[2]=forwardWord->points[0].z;
                lP01->values[3]=vLP01[0];
                lP01->values[4]=vLP01[1];
                lP01->values[5]=vLP01[2];

                lP12->values[0]=forwardWord->points[1].x;
                lP12->values[1]=forwardWord->points[1].y;
                lP12->values[2]=forwardWord->points[1].z;
                lP12->values[3]=vLP12[0];
                lP12->values[4]=vLP12[1];
                lP12->values[5]=vLP12[2];

                lP23->values[0]=forwardWord->points[2].x;
                lP23->values[1]=forwardWord->points[2].y;
                lP23->values[2]=forwardWord->points[2].z;
                lP23->values[3]=vLP23[0];
                lP23->values[4]=vLP23[1];
                lP23->values[5]=vLP23[2];

                for (unsigned long i=0;i<points.size();i++)
                {
                    if((forwardWordArea.index[0]<=i)&&(i<forwardWordArea.index[1]))
                    {
                        //第i条直线
                        pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                        lI->values.resize(6);
                        lI->values[0]=projectPoints[i].x;
                        lI->values[1]=projectPoints[i].y;
                        lI->values[2]=projectPoints[i].z;
                        lI->values[3]=vCy[0];
                        lI->values[4]=vCy[1];
                        lI->values[5]=vCy[2];

                        //求第i个交点
                        PointT pI,p;
                        float dI,d;
                        getLineLineIntersectPoint(lP12,lI,pI);
                        getPointPointDistance(pI,projectPoints[i],dI);

                        getLineLineIntersectPoint(lP23,lI,p);
                        getPointPointDistance(p,projectPoints[i],d);
                        if(dI>d)
                        {
                            dI=length;
                        }


                        PointT pOutI;
                        pOutI.x=points[i].x+dI*vCy[0];
                        pOutI.y=points[i].y+dI*vCy[1];
                        pOutI.z=points[i].z+dI*vCy[2];

                        outPoints1.push_back(pOutI);
                        outVector1.push_back(vectors[i]);

                    }
                    else if((forwardWordArea.index[1]<=i)&&(i<=forwardWordArea.index[2]))
                    {
                        //第i条直线
                        pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                        lI->values.resize(6);
                        lI->values[0]=projectPoints[i].x;
                        lI->values[1]=projectPoints[i].y;
                        lI->values[2]=projectPoints[i].z;
                        lI->values[3]=vCy[0];
                        lI->values[4]=vCy[1];
                        lI->values[5]=vCy[2];

                        //求第i个交点
                        PointT pI,p;
                        float dI,d;
                        getLineLineIntersectPoint(lP01,lI,pI);
                        getPointPointDistance(pI,projectPoints[i],dI);


                        getLineLineIntersectPoint(lP23,lI,p);
                        getPointPointDistance(p,projectPoints[i],d);
                        if(dI>d)
                        {
                            dI=length;
                        }

                        PointT pOutI;
                        pOutI.x=points[i].x+dI*vCy[0];
                        pOutI.y=points[i].y+dI*vCy[1];
                        pOutI.z=points[i].z+dI*vCy[2];

                        outPoints1.push_back(pOutI);
                        outVector1.push_back(vectors[i]);
                    }
                    else
                    {
                        PointT pOutI;
                        pOutI.x=points[i].x+length*vCy[0];
                        pOutI.y=points[i].y+length*vCy[1];
                        pOutI.z=points[i].z+length*vCy[2];

                        outPoints1.push_back(pOutI);
                        outVector1.push_back(vectors[i]);
                    }
                }
            }
            else
            {
                //关键3、0、1点构成的线
                pcl::ModelCoefficients::Ptr lP01(new pcl::ModelCoefficients);
                pcl::ModelCoefficients::Ptr lP03(new pcl::ModelCoefficients);
                pcl::ModelCoefficients::Ptr lP23(new pcl::ModelCoefficients);
                Eigen::Vector3f vLP01{forwardWord->points[0].x-forwardWord->points[1].x,forwardWord->points[0].y-forwardWord->points[1].y,forwardWord->points[0].z-forwardWord->points[1].z};
                Eigen::Vector3f vLP03{forwardWord->points[0].x-forwardWord->points[3].x,forwardWord->points[0].y-forwardWord->points[3].y,forwardWord->points[0].z-forwardWord->points[3].z};
                Eigen::Vector3f vLP23{forwardWord->points[2].x-forwardWord->points[3].x,forwardWord->points[2].y-forwardWord->points[3].y,forwardWord->points[2].z-forwardWord->points[3].z};
                lP01->values.resize(6);
                lP03->values.resize(6);
                lP23->values.resize(6);
                vLP01=vLP01.normalized();
                vLP03=vLP03.normalized();
                vLP23=vLP23.normalized();

                lP01->values[0]=forwardWord->points[0].x;
                lP01->values[1]=forwardWord->points[0].y;
                lP01->values[2]=forwardWord->points[0].z;
                lP01->values[3]=vLP01[0];
                lP01->values[4]=vLP01[1];
                lP01->values[5]=vLP01[2];

                lP03->values[0]=forwardWord->points[0].x;
                lP03->values[1]=forwardWord->points[0].y;
                lP03->values[2]=forwardWord->points[0].z;
                lP03->values[3]=vLP03[0];
                lP03->values[4]=vLP03[1];
                lP03->values[5]=vLP03[2];

                lP23->values[0]=forwardWord->points[2].x;
                lP23->values[1]=forwardWord->points[2].y;
                lP23->values[2]=forwardWord->points[2].z;
                lP23->values[3]=vLP23[0];
                lP23->values[4]=vLP23[1];
                lP23->values[5]=vLP23[2];

                for (unsigned long i=0;i<points.size();i++)
                {
                    if((forwardWordArea.index[0]<=i)&&(i<forwardWordArea.index[1]))
                    {
                        //第i条直线
                        pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                        lI->values.resize(6);
                        lI->values[0]=projectPoints[i].x;
                        lI->values[1]=projectPoints[i].y;
                        lI->values[2]=projectPoints[i].z;
                        lI->values[3]=vCy[0];
                        lI->values[4]=vCy[1];
                        lI->values[5]=vCy[2];

                        //求第i个交点
                        PointT pI,p;
                        float dI,d;
                        getLineLineIntersectPoint(lP01,lI,pI);
                        getPointPointDistance(pI,projectPoints[i],dI);


                        getLineLineIntersectPoint(lP23,lI,p);
                        getPointPointDistance(p,projectPoints[i],d);
                        if(dI>d)
                        {
                            dI=length;
                        }


                        PointT pOutI;
                        pOutI.x=points[i].x+dI*vCy[0];
                        pOutI.y=points[i].y+dI*vCy[1];
                        pOutI.z=points[i].z+dI*vCy[2];

                        outPoints1.push_back(pOutI);
                        outVector1.push_back(vectors[i]);

                    }
                    else if((forwardWordArea.index[1]<=i)&&(i<=forwardWordArea.index[2]))
                    {
                        //第i条直线
                        pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                        lI->values.resize(6);
                        lI->values[0]=projectPoints[i].x;
                        lI->values[1]=projectPoints[i].y;
                        lI->values[2]=projectPoints[i].z;
                        lI->values[3]=vCy[0];
                        lI->values[4]=vCy[1];
                        lI->values[5]=vCy[2];

                        //求第i个交点
                        PointT pI,p;
                        float dI,d;
                        getLineLineIntersectPoint(lP03,lI,pI);
                        getPointPointDistance(pI,projectPoints[i],dI);


                        getLineLineIntersectPoint(lP23,lI,p);
                        getPointPointDistance(p,projectPoints[i],d);
                        if(dI>d)
                        {
                            dI=length;
                        }

                        PointT pOutI;
                        pOutI.x=points[i].x+dI*vCy[0];
                        pOutI.y=points[i].y+dI*vCy[1];
                        pOutI.z=points[i].z+dI*vCy[2];

                        outPoints1.push_back(pOutI);
                        outVector1.push_back(vectors[i]);
                    }
                    else
                    {
                        PointT pOutI;
                        pOutI.x=points[i].x+length*vCy[0];
                        pOutI.y=points[i].y+length*vCy[1];
                        pOutI.z=points[i].z+length*vCy[2];

                        outPoints1.push_back(pOutI);
                        outVector1.push_back(vectors[i]);
                    }
                }
            }
        }
        else if(forwardWordArea.type=='D')
        {
            //关键3、0、1、2点构成的线
            pcl::ModelCoefficients::Ptr lP01(new pcl::ModelCoefficients);
            pcl::ModelCoefficients::Ptr lP03(new pcl::ModelCoefficients);
            pcl::ModelCoefficients::Ptr lP12(new pcl::ModelCoefficients);
            pcl::ModelCoefficients::Ptr lP23(new pcl::ModelCoefficients);
            Eigen::Vector3f vLP01{forwardWord->points[0].x-forwardWord->points[1].x,forwardWord->points[0].y-forwardWord->points[1].y,forwardWord->points[0].z-forwardWord->points[1].z};
            Eigen::Vector3f vLP03{forwardWord->points[0].x-forwardWord->points[3].x,forwardWord->points[0].y-forwardWord->points[3].y,forwardWord->points[0].z-forwardWord->points[3].z};
            Eigen::Vector3f vLP12{forwardWord->points[1].x-forwardWord->points[2].x,forwardWord->points[1].y-forwardWord->points[2].y,forwardWord->points[1].z-forwardWord->points[2].z};
            Eigen::Vector3f vLP23{forwardWord->points[2].x-forwardWord->points[3].x,forwardWord->points[2].y-forwardWord->points[3].y,forwardWord->points[2].z-forwardWord->points[3].z};
            lP01->values.resize(6);
            lP03->values.resize(6);
            lP12->values.resize(6);
            lP23->values.resize(6);
            vLP01=vLP01.normalized();
            vLP03=vLP03.normalized();
            vLP12=vLP12.normalized();
            vLP23=vLP23.normalized();

            lP01->values[0]=forwardWord->points[0].x;
            lP01->values[1]=forwardWord->points[0].y;
            lP01->values[2]=forwardWord->points[0].z;
            lP01->values[3]=vLP01[0];
            lP01->values[4]=vLP01[1];
            lP01->values[5]=vLP01[2];

            lP03->values[0]=forwardWord->points[0].x;
            lP03->values[1]=forwardWord->points[0].y;
            lP03->values[2]=forwardWord->points[0].z;
            lP03->values[3]=vLP03[0];
            lP03->values[4]=vLP03[1];
            lP03->values[5]=vLP03[2];

            lP12->values[0]=forwardWord->points[1].x;
            lP12->values[1]=forwardWord->points[1].y;
            lP12->values[2]=forwardWord->points[1].z;
            lP12->values[3]=vLP12[0];
            lP12->values[4]=vLP12[1];
            lP12->values[5]=vLP12[2];

            lP23->values[0]=forwardWord->points[2].x;
            lP23->values[1]=forwardWord->points[2].y;
            lP23->values[2]=forwardWord->points[2].z;
            lP23->values[3]=vLP23[0];
            lP23->values[4]=vLP23[1];
            lP23->values[5]=vLP23[2];


            for (unsigned long i=0;i<points.size();i++)
            {
                if((forwardWordArea.index[0]<=i)&&(i<forwardWordArea.index[1]))
                {
                    //第i条直线
                    pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                    lI->values.resize(6);
                    lI->values[0]=projectPoints[i].x;
                    lI->values[1]=projectPoints[i].y;
                    lI->values[2]=projectPoints[i].z;
                    lI->values[3]=vCy[0];
                    lI->values[4]=vCy[1];
                    lI->values[5]=vCy[2];

                    //求第i个交点
                    PointT pI,p;
                    float dI,d;
                    getLineLineIntersectPoint(lP12,lI,pI);
                    getPointPointDistance(pI,projectPoints[i],dI);


                    getLineLineIntersectPoint(lP23,lI,p);
                    getPointPointDistance(p,projectPoints[i],d);
                    if(dI>d)
                    {
                        dI=length;
                    }

                    PointT pOutI;
                    pOutI.x=points[i].x+dI*vCy[0];
                    pOutI.y=points[i].y+dI*vCy[1];
                    pOutI.z=points[i].z+dI*vCy[2];

                    outPoints1.push_back(pOutI);
                    outVector1.push_back(vectors[i]);

                }
                else if((forwardWordArea.index[1]<=i)&&(i<forwardWordArea.index[2]))
                {
                    //第i条直线
                    pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                    lI->values.resize(6);
                    lI->values[0]=projectPoints[i].x;
                    lI->values[1]=projectPoints[i].y;
                    lI->values[2]=projectPoints[i].z;
                    lI->values[3]=vCy[0];
                    lI->values[4]=vCy[1];
                    lI->values[5]=vCy[2];

                    //求第i个交点
                    PointT pI,p;
                    float dI,d;
                    getLineLineIntersectPoint(lP01,lI,pI);
                    getPointPointDistance(pI,projectPoints[i],dI);


                    getLineLineIntersectPoint(lP23,lI,p);
                    getPointPointDistance(p,projectPoints[i],d);
                    if(dI>d)
                    {
                        dI=length;
                    }

                    PointT pOutI;
                    pOutI.x=points[i].x+dI*vCy[0];
                    pOutI.y=points[i].y+dI*vCy[1];
                    pOutI.z=points[i].z+dI*vCy[2];

                    outPoints1.push_back(pOutI);
                    outVector1.push_back(vectors[i]);
                }
                else if((forwardWordArea.index[2]<=i)&&(i<=forwardWordArea.index[3]))
                {
                    //第i条直线
                    pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                    lI->values.resize(6);
                    lI->values[0]=projectPoints[i].x;
                    lI->values[1]=projectPoints[i].y;
                    lI->values[2]=projectPoints[i].z;
                    lI->values[3]=vCy[0];
                    lI->values[4]=vCy[1];
                    lI->values[5]=vCy[2];

                    //求第i个交点
                    PointT pI,p;
                    float dI,d;
                    getLineLineIntersectPoint(lP03,lI,pI);
                    getPointPointDistance(pI,projectPoints[i],dI);


                    getLineLineIntersectPoint(lP23,lI,p);
                    getPointPointDistance(p,projectPoints[i],d);
                    if(dI>d)
                    {
                        dI=length;
                    }

                    PointT pOutI;
                    pOutI.x=points[i].x+dI*vCy[0];
                    pOutI.y=points[i].y+dI*vCy[1];
                    pOutI.z=points[i].z+dI*vCy[2];

                    outPoints1.push_back(pOutI);
                    outVector1.push_back(vectors[i]);
                }
                else
                {
                    PointT pOutI;
                    pOutI.x=points[i].x+length*vCy[0];
                    pOutI.y=points[i].y+length*vCy[1];
                    pOutI.z=points[i].z+length*vCy[2];

                    outPoints1.push_back(pOutI);
                    outVector1.push_back(vectors[i]);
                }
            }

        }
        else
        {
            return -1;
        }

        if(reverseWordArea.type=='A')
        {
            //关键0、1点构成的线
            pcl::ModelCoefficients::Ptr lP01(new pcl::ModelCoefficients);
            pcl::ModelCoefficients::Ptr lP23(new pcl::ModelCoefficients);
            Eigen::Vector3f vLP01{reverseWord->points[0].x-reverseWord->points[1].x,reverseWord->points[0].y-reverseWord->points[1].y,reverseWord->points[0].z-reverseWord->points[1].z};
            Eigen::Vector3f vLP23{reverseWord->points[2].x-reverseWord->points[3].x,reverseWord->points[2].y-reverseWord->points[3].y,reverseWord->points[2].z-reverseWord->points[3].z};
            lP01->values.resize(6);
            lP23->values.resize(6);
            vLP01=vLP01.normalized();
            vLP01=vLP23.normalized();

            lP01->values[0]=reverseWord->points[0].x;
            lP01->values[1]=reverseWord->points[0].y;
            lP01->values[2]=reverseWord->points[0].z;
            lP01->values[3]=vLP01[0];
            lP01->values[4]=vLP01[1];
            lP01->values[5]=vLP01[2];


            lP23->values[0]=reverseWord->points[2].x;
            lP23->values[1]=reverseWord->points[2].y;
            lP23->values[2]=reverseWord->points[2].z;
            lP23->values[3]=vLP23[0];
            lP23->values[4]=vLP23[1];
            lP23->values[5]=vLP23[2];

            for (unsigned long i=0;i<points.size();i++)
            {
                if((reverseWordArea.index[0]<=i)&&(i<=reverseWordArea.index[1]))
                {
                    //第i条直线
                    pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                    lI->values.resize(6);
                    lI->values[0]=reverseProjectPoints[i].x;
                    lI->values[1]=reverseProjectPoints[i].y;
                    lI->values[2]=reverseProjectPoints[i].z;
                    lI->values[3]=vCy[0];
                    lI->values[4]=vCy[1];
                    lI->values[5]=vCy[2];

                    //求第i个交点
                    PointT pI,p;
                    float dI,d;
                    getLineLineIntersectPoint(lP01,lI,pI);
                    getPointPointDistance(pI,reverseProjectPoints[i],dI);

                    getLineLineIntersectPoint(lP23,lI,p);
                    getPointPointDistance(p,reverseProjectPoints[i],d);
                    if(dI>d)
                    {
                        dI=length;
                    }

                    PointT pOutI3,pOutI2;
                    pOutI2.x=points[i].x+length*vCy[0];
                    pOutI2.y=points[i].y+length*vCy[1];
                    pOutI2.z=points[i].z+length*vCy[2];

                    pOutI3.x=points[i].x+(length-dI)*vCy[0];
                    pOutI3.y=points[i].y+(length-dI)*vCy[1];
                    pOutI3.z=points[i].z+(length-dI)*vCy[2];

                    outPoints2.push_back(pOutI2);
                    outVector2.push_back(vectors[i]);
                    outPoints3.push_back(pOutI3);
                    outVector3.push_back(vectors[i]);

                }
            }
        }
        else if(reverseWordArea.type=='B'||reverseWordArea.type=='C')
        {
            if(reverseWordArea.type=='B')
            {
                //关键0、1、2点构成的线
                pcl::ModelCoefficients::Ptr lP01(new pcl::ModelCoefficients);
                pcl::ModelCoefficients::Ptr lP12(new pcl::ModelCoefficients);
                Eigen::Vector3f vLP01{reverseWord->points[0].x-reverseWord->points[1].x,reverseWord->points[0].y-reverseWord->points[1].y,reverseWord->points[0].z-reverseWord->points[1].z};
                Eigen::Vector3f vLP12{reverseWord->points[1].x-reverseWord->points[2].x,reverseWord->points[1].y-reverseWord->points[2].y,reverseWord->points[1].z-reverseWord->points[2].z};
                lP01->values.resize(6);
                lP12->values.resize(6);
                vLP01=vLP01.normalized();
                vLP12=vLP12.normalized();
                pcl::ModelCoefficients::Ptr lP23(new pcl::ModelCoefficients);
                Eigen::Vector3f vLP23{reverseWord->points[2].x-reverseWord->points[3].x,reverseWord->points[2].y-reverseWord->points[3].y,reverseWord->points[2].z-reverseWord->points[3].z};
                lP23->values.resize(6);
                vLP01=vLP23.normalized();

                lP01->values[0]=reverseWord->points[0].x;
                lP01->values[1]=reverseWord->points[0].y;
                lP01->values[2]=reverseWord->points[0].z;
                lP01->values[3]=vLP01[0];
                lP01->values[4]=vLP01[1];
                lP01->values[5]=vLP01[2];

                lP12->values[0]=reverseWord->points[1].x;
                lP12->values[1]=reverseWord->points[1].y;
                lP12->values[2]=reverseWord->points[1].z;
                lP12->values[3]=vLP12[0];
                lP12->values[4]=vLP12[1];
                lP12->values[5]=vLP12[2];

                lP23->values[0]=reverseWord->points[2].x;
                lP23->values[1]=reverseWord->points[2].y;
                lP23->values[2]=reverseWord->points[2].z;
                lP23->values[3]=vLP23[0];
                lP23->values[4]=vLP23[1];
                lP23->values[5]=vLP23[2];

                for (unsigned long i=0;i<points.size();i++)
                {
                    if((reverseWordArea.index[0]<=i)&&(i<reverseWordArea.index[1]))
                    {
                        //第i条直线
                        pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                        lI->values.resize(6);
                        lI->values[0]=reverseProjectPoints[i].x;
                        lI->values[1]=reverseProjectPoints[i].y;
                        lI->values[2]=reverseProjectPoints[i].z;
                        lI->values[3]=vCy[0];
                        lI->values[4]=vCy[1];
                        lI->values[5]=vCy[2];

                        //求第i个交点
                        PointT pI,p;
                        float dI,d;
                        getLineLineIntersectPoint(lP12,lI,pI);
                        getPointPointDistance(pI,reverseProjectPoints[i],dI);

                        getLineLineIntersectPoint(lP23,lI,p);
                        getPointPointDistance(p,reverseProjectPoints[i],d);
                        if(dI>d)
                        {
                            dI=length;
                        }

                        PointT pOutI3,pOutI2;
                        pOutI2.x=points[i].x+length*vCy[0];
                        pOutI2.y=points[i].y+length*vCy[1];
                        pOutI2.z=points[i].z+length*vCy[2];

                        pOutI3.x=points[i].x+(length-dI)*vCy[0];
                        pOutI3.y=points[i].y+(length-dI)*vCy[1];
                        pOutI3.z=points[i].z+(length-dI)*vCy[2];

                        outPoints2.push_back(pOutI2);
                        outVector2.push_back(vectors[i]);
                        outPoints3.push_back(pOutI3);
                        outVector3.push_back(vectors[i]);

                    }
                    else if((reverseWordArea.index[1]<=i)&&(i<=reverseWordArea.index[2]))
                    {
                        //第i条直线
                        pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                        lI->values.resize(6);
                        lI->values[0]=reverseProjectPoints[i].x;
                        lI->values[1]=reverseProjectPoints[i].y;
                        lI->values[2]=reverseProjectPoints[i].z;
                        lI->values[3]=vCy[0];
                        lI->values[4]=vCy[1];
                        lI->values[5]=vCy[2];

                        //求第i个交点
                        PointT pI,p;
                        float dI,d;
                        getLineLineIntersectPoint(lP01,lI,pI);
                        getPointPointDistance(pI,reverseProjectPoints[i],dI);
                        getLineLineIntersectPoint(lP23,lI,p);
                        getPointPointDistance(p,reverseProjectPoints[i],d);
                        if(dI>d)
                        {
                            dI=length;
                        }

                        PointT pOutI3,pOutI2;
                        pOutI2.x=points[i].x+length*vCy[0];
                        pOutI2.y=points[i].y+length*vCy[1];
                        pOutI2.z=points[i].z+length*vCy[2];

                        pOutI3.x=points[i].x+(length-dI)*vCy[0];
                        pOutI3.y=points[i].y+(length-dI)*vCy[1];
                        pOutI3.z=points[i].z+(length-dI)*vCy[2];

                        outPoints2.push_back(pOutI2);
                        outVector2.push_back(vectors[i]);
                        outPoints3.push_back(pOutI3);
                        outVector3.push_back(vectors[i]);
                    }
                }
            }
            else
            {
                //关键3、0、1点构成的线
                pcl::ModelCoefficients::Ptr lP01(new pcl::ModelCoefficients);
                pcl::ModelCoefficients::Ptr lP03(new pcl::ModelCoefficients);
                Eigen::Vector3f vLP01{reverseWord->points[0].x-reverseWord->points[1].x,reverseWord->points[0].y-reverseWord->points[1].y,reverseWord->points[0].z-reverseWord->points[1].z};
                Eigen::Vector3f vLP03{reverseWord->points[0].x-reverseWord->points[3].x,reverseWord->points[0].y-reverseWord->points[3].y,reverseWord->points[0].z-reverseWord->points[3].z};
                lP01->values.resize(6);
                lP03->values.resize(6);
                vLP01=vLP01.normalized();
                vLP03=vLP03.normalized();
                pcl::ModelCoefficients::Ptr lP23(new pcl::ModelCoefficients);
                Eigen::Vector3f vLP23{reverseWord->points[2].x-reverseWord->points[3].x,reverseWord->points[2].y-reverseWord->points[3].y,reverseWord->points[2].z-reverseWord->points[3].z};
                lP23->values.resize(6);
                vLP01=vLP23.normalized();

                lP01->values[0]=reverseWord->points[0].x;
                lP01->values[1]=reverseWord->points[0].y;
                lP01->values[2]=reverseWord->points[0].z;
                lP01->values[3]=vLP01[0];
                lP01->values[4]=vLP01[1];
                lP01->values[5]=vLP01[2];

                lP03->values[0]=reverseWord->points[0].x;
                lP03->values[1]=reverseWord->points[0].y;
                lP03->values[2]=reverseWord->points[0].z;
                lP03->values[3]=vLP03[0];
                lP03->values[4]=vLP03[1];
                lP03->values[5]=vLP03[2];

                lP23->values[0]=reverseWord->points[2].x;
                lP23->values[1]=reverseWord->points[2].y;
                lP23->values[2]=reverseWord->points[2].z;
                lP23->values[3]=vLP23[0];
                lP23->values[4]=vLP23[1];
                lP23->values[5]=vLP23[2];

                for (unsigned long i=0;i<points.size();i++)
                {
                    if((reverseWordArea.index[0]<=i)&&(i<reverseWordArea.index[1]))
                    {
                        //第i条直线
                        pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                        lI->values.resize(6);
                        lI->values[0]=reverseProjectPoints[i].x;
                        lI->values[1]=reverseProjectPoints[i].y;
                        lI->values[2]=reverseProjectPoints[i].z;
                        lI->values[3]=vCy[0];
                        lI->values[4]=vCy[1];
                        lI->values[5]=vCy[2];

                        //求第i个交点
                        PointT pI,p;
                        float dI,d;
                        getLineLineIntersectPoint(lP01,lI,pI);
                        getPointPointDistance(pI,reverseProjectPoints[i],dI);
                        getLineLineIntersectPoint(lP23,lI,p);
                        getPointPointDistance(p,reverseProjectPoints[i],d);
                        if(dI>d)
                        {
                            dI=length;
                        }

                        PointT pOutI3,pOutI2;
                        pOutI2.x=points[i].x+length*vCy[0];
                        pOutI2.y=points[i].y+length*vCy[1];
                        pOutI2.z=points[i].z+length*vCy[2];

                        pOutI3.x=points[i].x+(length-dI)*vCy[0];
                        pOutI3.y=points[i].y+(length-dI)*vCy[1];
                        pOutI3.z=points[i].z+(length-dI)*vCy[2];

                        outPoints2.push_back(pOutI2);
                        outVector2.push_back(vectors[i]);
                        outPoints3.push_back(pOutI3);
                        outVector3.push_back(vectors[i]);

                    }
                    else if((reverseWordArea.index[1]<=i)&&(i<=reverseWordArea.index[2]))
                    {
                        //第i条直线
                        pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                        lI->values.resize(6);
                        lI->values[0]=reverseProjectPoints[i].x;
                        lI->values[1]=reverseProjectPoints[i].y;
                        lI->values[2]=reverseProjectPoints[i].z;
                        lI->values[3]=vCy[0];
                        lI->values[4]=vCy[1];
                        lI->values[5]=vCy[2];

                        //求第i个交点
                        PointT pI,p;
                        float dI,d;
                        getLineLineIntersectPoint(lP03,lI,pI);
                        getPointPointDistance(pI,reverseProjectPoints[i],dI);
                        getLineLineIntersectPoint(lP23,lI,p);
                        getPointPointDistance(p,reverseProjectPoints[i],d);
                        if(dI>d)
                        {
                            dI=length;
                        }

                        PointT pOutI3,pOutI2;
                        pOutI2.x=points[i].x+length*vCy[0];
                        pOutI2.y=points[i].y+length*vCy[1];
                        pOutI2.z=points[i].z+length*vCy[2];

                        pOutI3.x=points[i].x+(length-dI)*vCy[0];
                        pOutI3.y=points[i].y+(length-dI)*vCy[1];
                        pOutI3.z=points[i].z+(length-dI)*vCy[2];

                        outPoints2.push_back(pOutI2);
                        outVector2.push_back(vectors[i]);
                        outPoints3.push_back(pOutI3);
                        outVector3.push_back(vectors[i]);
                    }
                }
            }
        }
        else if(reverseWordArea.type=='D')
        {
            //关键3、0、1、2点构成的线
            pcl::ModelCoefficients::Ptr lP01(new pcl::ModelCoefficients);
            pcl::ModelCoefficients::Ptr lP03(new pcl::ModelCoefficients);
            pcl::ModelCoefficients::Ptr lP12(new pcl::ModelCoefficients);
            Eigen::Vector3f vLP01{reverseWord->points[0].x-reverseWord->points[1].x,reverseWord->points[0].y-reverseWord->points[1].y,reverseWord->points[0].z-reverseWord->points[1].z};
            Eigen::Vector3f vLP03{reverseWord->points[0].x-reverseWord->points[3].x,reverseWord->points[0].y-reverseWord->points[3].y,reverseWord->points[0].z-reverseWord->points[3].z};
            Eigen::Vector3f vLP12{reverseWord->points[1].x-reverseWord->points[2].x,reverseWord->points[1].y-reverseWord->points[2].y,reverseWord->points[1].z-reverseWord->points[2].z};
            lP01->values.resize(6);
            lP03->values.resize(6);
            lP12->values.resize(6);
            vLP01=vLP01.normalized();
            vLP03=vLP03.normalized();
            vLP12=vLP12.normalized();

            pcl::ModelCoefficients::Ptr lP23(new pcl::ModelCoefficients);
            Eigen::Vector3f vLP23{reverseWord->points[2].x-reverseWord->points[3].x,reverseWord->points[2].y-reverseWord->points[3].y,reverseWord->points[2].z-reverseWord->points[3].z};
            lP23->values.resize(6);
            vLP01=vLP23.normalized();

            lP01->values[0]=reverseWord->points[0].x;
            lP01->values[1]=reverseWord->points[0].y;
            lP01->values[2]=reverseWord->points[0].z;
            lP01->values[3]=vLP01[0];
            lP01->values[4]=vLP01[1];
            lP01->values[5]=vLP01[2];

            lP03->values[0]=reverseWord->points[0].x;
            lP03->values[1]=reverseWord->points[0].y;
            lP03->values[2]=reverseWord->points[0].z;
            lP03->values[3]=vLP03[0];
            lP03->values[4]=vLP03[1];
            lP03->values[5]=vLP03[2];

            lP12->values[0]=reverseWord->points[1].x;
            lP12->values[1]=reverseWord->points[1].y;
            lP12->values[2]=reverseWord->points[1].z;
            lP12->values[3]=vLP12[0];
            lP12->values[4]=vLP12[1];
            lP12->values[5]=vLP12[2];

            lP23->values[0]=reverseWord->points[2].x;
            lP23->values[1]=reverseWord->points[2].y;
            lP23->values[2]=reverseWord->points[2].z;
            lP23->values[3]=vLP23[0];
            lP23->values[4]=vLP23[1];
            lP23->values[5]=vLP23[2];



            for (unsigned long i=0;i<points.size();i++)
            {
                if((reverseWordArea.index[0]<=i)&&(i<reverseWordArea.index[1]))
                {
                    //第i条直线
                    pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                    lI->values.resize(6);
                    lI->values[0]=reverseProjectPoints[i].x;
                    lI->values[1]=reverseProjectPoints[i].y;
                    lI->values[2]=reverseProjectPoints[i].z;
                    lI->values[3]=vCy[0];
                    lI->values[4]=vCy[1];
                    lI->values[5]=vCy[2];

                    //求第i个交点
                    PointT pI,p;
                    float dI,d;
                    getLineLineIntersectPoint(lP12,lI,pI);
                    getPointPointDistance(pI,reverseProjectPoints[i],dI);
                    getLineLineIntersectPoint(lP23,lI,p);
                    getPointPointDistance(p,reverseProjectPoints[i],d);
                    if(dI>d)
                    {
                        dI=length;
                    }

                    PointT pOutI3,pOutI2;
                    pOutI2.x=points[i].x+length*vCy[0];
                    pOutI2.y=points[i].y+length*vCy[1];
                    pOutI2.z=points[i].z+length*vCy[2];

                    pOutI3.x=points[i].x+(length-dI)*vCy[0];
                    pOutI3.y=points[i].y+(length-dI)*vCy[1];
                    pOutI3.z=points[i].z+(length-dI)*vCy[2];

                    outPoints2.push_back(pOutI2);
                    outVector2.push_back(vectors[i]);
                    outPoints3.push_back(pOutI3);
                    outVector3.push_back(vectors[i]);

                }
                else if((reverseWordArea.index[1]<=i)&&(i<reverseWordArea.index[2]))
                {
                    //第i条直线
                    pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                    lI->values.resize(6);
                    lI->values[0]=reverseProjectPoints[i].x;
                    lI->values[1]=reverseProjectPoints[i].y;
                    lI->values[2]=reverseProjectPoints[i].z;
                    lI->values[3]=vCy[0];
                    lI->values[4]=vCy[1];
                    lI->values[5]=vCy[2];

                    //求第i个交点
                    PointT pI,p;
                    float dI,d;
                    getLineLineIntersectPoint(lP01,lI,pI);
                    getPointPointDistance(pI,reverseProjectPoints[i],dI);
                    getLineLineIntersectPoint(lP23,lI,p);
                    getPointPointDistance(p,reverseProjectPoints[i],d);
                    if(dI>d)
                    {
                        dI=length;
                    }

                    PointT pOutI3,pOutI2;
                    pOutI2.x=points[i].x+length*vCy[0];
                    pOutI2.y=points[i].y+length*vCy[1];
                    pOutI2.z=points[i].z+length*vCy[2];

                    pOutI3.x=points[i].x+(length-dI)*vCy[0];
                    pOutI3.y=points[i].y+(length-dI)*vCy[1];
                    pOutI3.z=points[i].z+(length-dI)*vCy[2];

                    outPoints2.push_back(pOutI2);
                    outVector2.push_back(vectors[i]);
                    outPoints3.push_back(pOutI3);
                    outVector3.push_back(vectors[i]);
                }
                else if((reverseWordArea.index[2]<=i)&&(i<=reverseWordArea.index[3]))
                {
                    //第i条直线
                    pcl::ModelCoefficients::Ptr lI(new pcl::ModelCoefficients);
                    lI->values.resize(6);
                    lI->values[0]=reverseProjectPoints[i].x;
                    lI->values[1]=reverseProjectPoints[i].y;
                    lI->values[2]=reverseProjectPoints[i].z;
                    lI->values[3]=vCy[0];
                    lI->values[4]=vCy[1];
                    lI->values[5]=vCy[2];

                    //求第i个交点
                    PointT pI,p;
                    float dI,d;
                    getLineLineIntersectPoint(lP03,lI,pI);
                    getPointPointDistance(pI,reverseProjectPoints[i],dI);
                    getLineLineIntersectPoint(lP23,lI,p);
                    getPointPointDistance(p,reverseProjectPoints[i],d);
                    if(dI>d)
                    {
                        dI=length;
                    }

                    PointT pOutI3,pOutI2;
                    pOutI2.x=points[i].x+length*vCy[0];
                    pOutI2.y=points[i].y+length*vCy[1];
                    pOutI2.z=points[i].z+length*vCy[2];

                    pOutI3.x=points[i].x+(length-dI)*vCy[0];
                    pOutI3.y=points[i].y+(length-dI)*vCy[1];
                    pOutI3.z=points[i].z+(length-dI)*vCy[2];

                    outPoints2.push_back(pOutI2);
                    outVector2.push_back(vectors[i]);
                    outPoints3.push_back(pOutI3);
                    outVector3.push_back(vectors[i]);
                }
            }

        }
        else
        {
            return -1;
        }


        listOutPoints.push_back(outPoints1);
        listOutVectors.push_back(outVector1);

        listOutPoints.push_back(outPoints2);
        listOutVectors.push_back(outVector2);


        listOutPoints.push_back(outPoints3);
        listOutVectors.push_back(outVector3);
    }
    else
    {
        for (unsigned long i=0;i<points.size();i++)
        {
            PointT pOutI;
            pOutI.x=points[i].x+length*vCy[0];
            pOutI.y=points[i].y+length*vCy[1];
            pOutI.z=points[i].z+length*vCy[2];

            outPoints1.push_back(pOutI);
            outVector1.push_back(vectors[i]);
        }
        listOutPoints.push_back(outPoints1);
        listOutVectors.push_back(outVector1);
    }

    return 0;
}

/**********************************************
* @projectName   ThiVision::tidyKeyPoints
* @brief         整理路径关键点，使得排序成为路径
* @inParam
*                points1: 输入1号点集
*                vectors1: 输入1号点集方向向量
*                points2: 输入1号点集
*                vectors2: 输入1号点集方向向量
* @outParam
*                outPoints: 输出的路径点集
*                outVectors: 输出的路径点集方向向量
* @return        成功返回0
* @author        xs(1874020422@qq.com)
* @date          2024-08-09
* **********************************************/
int ThiVision::tidyKeyPoints(std::vector<PointT> points1, std::vector<PointT> vectors1, std::vector<PointT> points2, std::vector<PointT> vectors2, std::vector<PointT> &outPoints, std::vector<PointT> &outVectors)
{
    TrackPoint track1{points1,vectors1};
    TrackPoint track2{points2,vectors2};
    lineTrimmer.curIndex=0;
    lineTrimmer.isAcross=true;//是否为横
    lineTrimmer.curTrack=track1;
    lineTrimmer.beforTrack=track2;
    outPoints.push_back(lineTrimmer.curTrack.point[lineTrimmer.curIndex]);
    outVectors.push_back(lineTrimmer.curTrack.vector[lineTrimmer.curIndex]);
    for (unsigned long i=0;i<points1.size()+points2.size()-1;i++)
    {
        if(lineTrimmer.isAcross)
        {
            TrackPoint tempTrack=lineTrimmer.curTrack;
            lineTrimmer.curTrack=lineTrimmer.beforTrack;
            lineTrimmer.beforTrack=tempTrack;
            lineTrimmer.isAcross=!lineTrimmer.isAcross;
        }
        else
        {
            lineTrimmer.curIndex++;
            lineTrimmer.isAcross=!lineTrimmer.isAcross;
        }
        outPoints.push_back(lineTrimmer.curTrack.point[lineTrimmer.curIndex]);
        outVectors.push_back(lineTrimmer.curTrack.vector[lineTrimmer.curIndex]);
    }
    return 0;

}

/**********************************************
* @projectName   ThiVision::pressTrackPoints
* @brief         把路径点按照法线反方向平移一定长度
* @inParam       points: 输入的点集
*                vectors: 输入的向量集
*                press: 下压的量(长度)
* @outParam      outPoints: 输出的路径点集
* @return        成功返回0
* @author        xs(1874020422@qq.com)
* @date          2024-12-06
**********************************************/
int ThiVision::pressTrackPoints(std::vector<PointT> points,std::vector<PointT> vectors,float press,std::vector<PointT> &outPoints)
{
    std::vector<PointT> temp;
    for(unsigned long i=0;i<points.size();i++)
    {
        Eigen::Vector3f v(vectors[i].x,vectors[i].y,vectors[i].z);
        v=-v.normalized();
        PointT p(points[i].x+v(0)*press,points[i].y+v(1)*press,points[i].z+v(2)*press);
        temp.push_back(p);
    }
    outPoints.clear();
    outPoints = temp;
    return 0;
}



/**********************************************
* @projectName   ThiVision::getPoint2PlaneDistance
* @brief         得到点到平面的距离
* @inParam       coefficients: 平面参数
*                point: 点坐标
* @outParam      void
* @return        点到平面的距离
* @author        xs(1874020422@qq.com)
* @date          2024-09-12
**********************************************/
float ThiVision::getPoint2PlaneDistance(const pcl::ModelCoefficients::Ptr coefficients, const PointT point)
{
    float A=coefficients->values[0];
    float B=coefficients->values[1];
    float C=coefficients->values[2];
    float D=coefficients->values[3];

    float X0=point.x;
    float Y0=point.y;
    float Z0=point.z;

    //d=|Ax0+By0+Cz0+D|/√(A²+B²+C²)
    return std::abs(A*X0+B*Y0+C*Z0+D)/sqrt(A*A+B*B+C*C);
}

/**********************************************
* @projectName   ThiVision::passThroughForPlane
* @brief         把点云按照距离某一平面的一定距离范围进行切片处理
* @inParam       coefficients: 平面参数
*                cloudIn: 输入的点云
*                minDistance: 最小距离
*                maxDistance: 最大距离
* @outParam
*                cloudOut: 输出的点云
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-09-11
**********************************************/
void ThiVision::passThroughForPlane(const pcl::ModelCoefficients::Ptr coefficients,const pcl::PointCloud<PointT>::Ptr &cloudIn,float minDistance,float maxDistance,pcl::PointCloud<PointT>::Ptr &cloudOut)
{
    pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
    for (unsigned long i=0;i<cloudIn->size();i++)
    {
        float d =getPoint2PlaneDistance(coefficients,cloudIn->points[i]);
        if(minDistance<=d&&d<=maxDistance)
        {
            out->push_back(cloudIn->points[i]);
        }
    }
    cloudOut->clear();
    *cloudOut=*out;
}

/**********************************************
* @projectName   ThiVision::getPlaneUpOrDownPointCloud
* @brief         得到平面上或则下的点云
* @inParam       plane: 输入的平面参数
*                cloudIn: 输入的点云数据
*                vAix: 判断的正方向向量 如果朝向为这边 点云为上
*                isUp: 是否保留平面上的点云
* @outParam      cloudOut: 输出保留后的点云
* @return        平面参数错误返回-1 成功返回0
* @author        xs(1874020422@qq.com)
* @date          2024-12-04
**********************************************/
int ThiVision::getPlaneUpOrDownPointCloud(pcl::ModelCoefficients::Ptr &plane, pcl::PointCloud<PointT>::Ptr &cloudIn, pcl::PointCloud<PointT>::Ptr &cloudOut,Eigen::Vector3f vAix, bool isUp)
{
    if(plane->values.size()!=4)
        return -1;
    pcl::PointCloud<PointT>::Ptr temp(new  pcl::PointCloud<PointT>());
    Eigen::Vector3f n(plane->values[0],plane->values[1],plane->values[2]);
    vAix = vAix.normalized();
    float d = plane->values[3];
    if(n.dot(vAix)<0)
    {
        d=-d;
        n=-n;
    }

    for(unsigned long i = 0;i<cloudIn->size();i++)
    {
        float dp = cloudIn->points[i].x*n(0)+cloudIn->points[i].y*n(1)+cloudIn->points[i].z*n(2)+d;
        if(isUp)
        {
            if(dp>0)
            {
                temp->push_back(cloudIn->points[i]);
            }
        }
        else
        {
            if(dp<0)
            {
                temp->push_back(cloudIn->points[i]);
            }
        }
    }
    cloudOut->clear();
    *cloudOut = *temp;
    return 0;
}

/**********************************************
* @projectName  ThiVision::movePlaneByAix
* @brief        把平面沿着轴向方向平移一定长度形成新的平面
* @inParam      planeIn: 输入的平面
*               vAix: 平面法向量平移的正方向
*               length: 平移的长度
* @outParam
*               planeOut: 输出的平面
* @return       成功返回0，失败返回-1
* @author       xs(1874020422@qq.com)
* @date         2024-12-11
**********************************************/
int ThiVision::movePlaneByAix(pcl::ModelCoefficients::Ptr &planeIn, pcl::ModelCoefficients::Ptr &planeOut,Eigen::Vector3f vAix, float length)
{
    if(planeIn->values.size()!=4)
        return -1;
    Eigen::Vector3f vP(planeIn->values[0],planeIn->values[1],planeIn->values[2]);
    float d =planeIn->values[3]-length*sqrt(vP(0)*vP(0)+vP(1)*vP(1)+vP(2)*vP(2));
    if(vP.dot(vAix)<0)
    {
        vP=-vP;
        d=-planeIn->values[3]-length*sqrt(vP(0)*vP(0)+vP(1)*vP(1)+vP(2)*vP(2));
    }


    planeOut->values.resize(4);
    planeOut->values[0]=vP(0);
    planeOut->values[1]=vP(1);
    planeOut->values[2]=vP(2);
    planeOut->values[3]=d;
    return 0;
}

/**********************************************
* @projectName   ThiVision::getSurfacePointCloud
* @brief         提取点云的表面点
* @inParam       cloudIn: 输入的点云
*                lineCutBase: 切割平面的基础线
*                vCut: 截平面向量 可以和上面的基础线构成切割面
*                ratio: 切割精度 平面每次平移的长度
*                isRight: 是否保留锁夹右侧表面点 即保留y小的点云 否则保留y大的点
* @outParam
*
* @return        参数错误返回-1，成功返回0
* @author        xs(1874020422@qq.com)
* @date          2025-01-07
**********************************************/
int ThiVision::getSurfacePointCloud(pcl::PointCloud<PointT>::Ptr &cloudIn,pcl::PointCloud<PointT>::Ptr &cloudOut,pcl::ModelCoefficients::Ptr lineCutBase,Eigen::Vector3f vCut,float ratio,bool isRight)
{
    if(lineCutBase->values.size()!=6)
        return -1;

    PointT minP,maxP;
    pcl::ModelCoefficients::Ptr planeCut(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr lineCut(new pcl::ModelCoefficients);

    PointT pMinY = cloudIn->points[0];
    for (unsigned long i=1;i<cloudIn->size();i++)
    {
        if(pMinY.y>cloudIn->points[i].y)
        {
            pMinY=cloudIn->points[i];
        }
    }

    lineCut->values.resize(6);
    lineCut->values[0]=pMinY.x;
    lineCut->values[1]=pMinY.y;
    lineCut->values[2]=pMinY.z;
    lineCut->values[3]=lineCutBase->values[3];
    lineCut->values[4]=lineCutBase->values[4];
    lineCut->values[5]=lineCutBase->values[5];
    createSectionPlane(lineCut,vCut,planeCut);
    getPointCloudMinMax3D(cloudIn,minP,maxP);

    unsigned long n = static_cast<unsigned long>((maxP.z-minP.z)/ratio);

    for(unsigned long i=0;i<n;i++)
    {
        getPlaneUpOrDownPointCloud(planeCut,cloudIn,temp,Eigen::Vector3f(0,0,1),true);
        movePlaneByAix(planeCut,planeCut,Eigen::Vector3f(0,0,1),ratio);
        getPlaneUpOrDownPointCloud(planeCut,temp,temp,Eigen::Vector3f(0,0,1),false);

        if(temp->size()!=0)
        {
            pMinY = temp->points[0];
            if(isRight)
            {
                for (unsigned long j=0;j<temp->size();j++)
                {
                    if(pMinY.y>temp->points[j].y)
                    {
                        pMinY=temp->points[j];
                    }
                }
            }
            else
            {
                for (unsigned long j=0;j<temp->size();j++)
                {
                    if(pMinY.y<temp->points[j].y)
                    {
                        pMinY=temp->points[j];
                    }
                }
            }
            out->push_back(pMinY);
        }

    }

    cloudOut->clear();
    *cloudOut=*out;
    return 0;
}

/**********************************************
* @projectName   ThiVision::getCurveInfo
* @brief         将投影在某一截面的曲线点云近似处理为多条折线
* @inParam       cloudIn: 输入的点云
*                minDstance: 每条折线的最小长度
*                vSection: 截面法向量
* @outParam      C: Curve类型结构体，包含了折线构成的点和直线方向向量和法向量
*
* @return        成功返回0
* @author        xs(1874020422@qq.com)
* @date          2024-12-24
**********************************************/
int ThiVision::getCurveInfo(pcl::PointCloud<PointT>::Ptr &cloudIn, float minDstance,Eigen::Vector3f vSection, Curve &C)
{
    float curD=0;
    PointT curP=cloudIn->points[0];
    C.points.push_back(curP);

    for (unsigned long i=1;i<cloudIn->size();i++)
    {
        getPointPointDistance(curP,cloudIn->points[i],curD);
        if(curD>minDstance)
        {
            Eigen::Vector3f vN;
            Eigen::Vector3f vL(cloudIn->points[i].x-curP.x,cloudIn->points[i].y-curP.y,cloudIn->points[i].z-curP.z);
            vL=vL.normalized();

            if(vSection[0]<0)
                vSection=-vSection;
            vN=vSection.cross(vL);
            curP=cloudIn->points[i];

            vN(2) = (vN(2)>0)?vN(2):0;
            vN = vN.normalized();

            C.points.push_back(curP);
            C.vectorLine.push_back(vL);
            C.vectorNormal.push_back(vN);
        }
    }
    return 0;
}

/**********************************************
* @projectName   ThiVision::retainValidCurvePointCloud
* @brief         保留耳板分界线内有效曲线点云
* @inParam       cloudIn: 输入的曲线点云
*                plane: 立面平面
*                pEar1: 耳板分界线点1
*                pEar2: 耳板分界线点2
* @outParam
*                cloudOut: 输出的曲线点云
*                lineCutEar: 输出的耳板分界线
* @return        成功返回结构体分界变化索引，失败返回-1
* @author        xs(1874020422@qq.com)
* @date          2024-12-24
**********************************************/
int ThiVision::retainValidCurvePointCloud(pcl::PointCloud<PointT>::Ptr &cloudIn,const pcl::ModelCoefficients::Ptr plane,PointT pEar1,PointT pEar2,pcl::PointCloud<PointT>::Ptr &cloudOut,pcl::ModelCoefficients::Ptr &lineCutEar)
{
    pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>);
    Eigen::Vector3f vL(pEar1.x-pEar2.x,pEar1.y-pEar2.y,pEar1.z-pEar2.z);
    Eigen::Vector3f vFacade(plane->values[0],plane->values[1],plane->values[2]);
    vL=vL.normalized();
    vFacade = vFacade.normalized();

    lineCutEar->values.resize(6);
    lineCutEar->values[0]=pEar1.x;
    lineCutEar->values[1]=pEar1.y;
    lineCutEar->values[2]=pEar1.z;
    lineCutEar->values[3]=vL(0);
    lineCutEar->values[4]=vL(1);
    lineCutEar->values[5]=vL(2);


    pcl::ModelCoefficients::Ptr planeEarCut(new pcl::ModelCoefficients);
    Eigen::Vector3f vEarCut = vFacade.cross(vL).normalized();
    createSectionPlane(lineCutEar,vEarCut,planeEarCut);

    getPlaneUpOrDownPointCloud(planeEarCut,cloudIn,temp,Eigen::Vector3f(0,0,1),false);
    cloudOut->clear();
    *cloudOut = *temp;

    return 0;
}


/**********************************************
* @projectName   ThiVision::getCurveCutPoints
* @brief         对折线化的曲线进行道宽分割点的生成
* @inParam       C: 输入的折线化曲线
*                width: 道宽
*                avoidDown: 下面的避障长度
*                avoidUp: 上面的避障长度
*                avoidVectorDown: 下面的避障向量
*                avoidVectorUp: 上面的避障向量
* @outParam
*                cloudOut: 输出的路径关键点
*                cloudVector: 输出的关键点向量
* @return        失败返回-1(参数有误),成功返回0
* @author        xs(1874020422@qq.com)
* @date          2025-01-07
**********************************************/
int ThiVision::getCurveCutPoints(Curve &C,float width,Eigen::Vector3f vAixs,float avoidDown,float avoidUp,Eigen::Vector3f avoidVectorDown,Eigen::Vector3f avoidVectorUp,pcl::PointCloud<PointT>::Ptr &cloudOut,pcl::PointCloud<PointT>::Ptr &cloudVector)
{

    if(C.vectorLine.size()==0)
        return -1;

    pcl::PointCloud<PointT>::Ptr T(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr V(new pcl::PointCloud<PointT>);
    vAixs=vAixs.normalized();
    if(vAixs(0)<0)
        vAixs=-vAixs;

    PointT curP=C.points[0];
    Eigen::Vector3f curv = C.vectorNormal[0];
    T->push_back(curP);
    V->push_back(PointT(curv.x(),curv.y(),curv.z()));
    for (unsigned long curIndex=0;curIndex<C.vectorLine.size();)
    {
        float curx=curP.x+width*C.vectorLine[curIndex].x();
        float cury=curP.y+width*C.vectorLine[curIndex].y();
        float curz=curP.z+width*C.vectorLine[curIndex].z();
        curv = C.vectorNormal[curIndex];
        Eigen::Vector3f vCurP(curx-C.points[curIndex+1].x,cury-C.points[curIndex+1].y,curz-C.points[curIndex+1].z);
        vCurP=vCurP.normalized();
        if(vCurP.dot(C.vectorLine[curIndex])>0)
        {
            if(C.vectorLine.size()-1==curIndex)
                break;
            float distance;
            getPointPointDistance(curP,C.points[curIndex+1],distance);
            float cosA=C.vectorLine[curIndex+1].dot(-C.vectorLine[curIndex]);
            float a=1;
            float b=-2*distance*cosA;
            float c=distance*distance-width*width;
            float x1=(-b+sqrt(b*b-4*c*a))/2;
            float x2=(-b-sqrt(b*b-4*c*a))/2;
            float l =(x1>0)?x1:x2;
            curx=C.points[curIndex+1].x+l*C.vectorLine[curIndex+1].x();
            cury=C.points[curIndex+1].y+l*C.vectorLine[curIndex+1].y();
            curz=C.points[curIndex+1].z+l*C.vectorLine[curIndex+1].z();
            curv=C.vectorNormal[curIndex+1];
            curIndex++;
        }

        curP.x=curx;
        curP.y=cury;
        curP.z=curz;

        T->push_back(curP);
        V->push_back(PointT(curv.x(),curv.y(),curv.z()));
    }


    unsigned long downIndex = static_cast<unsigned long>(avoidDown/width)+1;
    unsigned long upIndex = V->size()-static_cast<unsigned long>(avoidUp/width)-2;

    if(avoidDown<=static_cast<float>(0))
        downIndex=static_cast<unsigned long>(0);
    if(avoidUp<=static_cast<float>(0))
        upIndex=V->size();

    //是否均匀变化
#define ISEVEN true
#if ISEVEN
    unsigned long indexD = upIndex - downIndex;
    float athe = abs(acos(avoidVectorDown.dot(avoidVectorUp)));
#endif

    for (unsigned long i=0;i<V->size();i++)
    {
        if(i<downIndex)
        {
            V->points[i].x=avoidVectorDown.x();
            V->points[i].y=avoidVectorDown.y();
            V->points[i].z=avoidVectorDown.z();
        }
        else if(i>=upIndex)
        {
            V->points[i].x=avoidVectorUp.x();
            V->points[i].y=avoidVectorUp.y();
            V->points[i].z=avoidVectorUp.z();
        }
#if ISEVEN
        else
        {
            Eigen::Vector3f vEven=getRotatedVector(avoidVectorDown,vAixs,static_cast<double>(athe/indexD)*(i-downIndex));//均匀变化由垂直变换为水平
            V->points[i].x=vEven.x();
            V->points[i].y=vEven.y();
            V->points[i].z=vEven.z();
        }
#endif
    }

    cloudOut->clear();
    cloudVector->clear();
    *cloudVector=*V;
    *cloudOut=*T;
    return 0;
}

/**********************************************
* @projectName   ThiVision::retainDownWorkProjection
* @brief         把点云的有效打磨区域保留并投影到一平面上
* @inParam       cloudIn: 输入的点云数据
*                keyPointsIn: 输入的路径关键点集
*                plane: 输入的投影平面
*                vAixs: 轴向量(截平面向量)
* @outParam      cloudOut: 输出的点云
* @return        平面参数错误返回-1，成功返回0
* @author        xs(1874020422@qq.com)
* @date          2025-01-07
**********************************************/
int ThiVision::retainDownWorkProjection(pcl::PointCloud<PointT>::Ptr &cloudIn,pcl::PointCloud<PointT>::Ptr &keyPointsIn,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vAixs,pcl::PointCloud<PointT>::Ptr &cloudOut)
{
    if(plane->values.size()!=4)
        return -1;

    pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>());

    planarProjectionPointCloud(cloudIn,temp,plane);

    Eigen::Vector3f np(plane->values[0],plane->values[1],plane->values[2]);
    Eigen::Vector3f vAixsPro=(vAixs-((vAixs.dot(np))*np.normalized())).normalized();
    Eigen::Vector3f vPlaneCut = (vAixsPro.cross(np)).normalized();

    pcl::ModelCoefficients::Ptr planeCut(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr lineCut(new pcl::ModelCoefficients);
    planeCut->values.resize(4);
    lineCut->values.resize(6);

    lineCut->values[0]=keyPointsIn->points[keyPointsIn->size()-1].x;
    lineCut->values[1]=keyPointsIn->points[keyPointsIn->size()-1].y;
    lineCut->values[2]=keyPointsIn->points[keyPointsIn->size()-1].z;
    lineCut->values[3]=vAixsPro[0];
    lineCut->values[4]=vAixsPro[1];
    lineCut->values[5]=vAixsPro[2];

    createSectionPlane(lineCut,vPlaneCut,planeCut);

    getPlaneUpOrDownPointCloud(planeCut,temp,temp,Eigen::Vector3f(0,0,1),false);

    cloudOut->clear();
    *cloudOut=*temp;
    return 0;
}

/**********************************************
* @projectName   ThiVision::getTopLeftAndRightLine
* @brief         得到下半件顶部左右边界直线
* @inParam       cloudIn: 输入已经投影和分割有效区域后的点云
*                plane: 输入的投影平面
*                vAixs: 轴向量(截平面向量)
* @outParam      lineLeft: 输出的上左边界直线
*                lineRight: 输出的上右边界直线
* @return        平面参数错误返回-1，成功返回0
* @author        xs(1874020422@qq.com)
* @date          2025-01-07
**********************************************/
int ThiVision::getTopLeftAndRightLine(pcl::PointCloud<PointT>::Ptr &cloudIn,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vAixs,pcl::ModelCoefficients::Ptr &lineLeft,pcl::ModelCoefficients::Ptr &lineRight)
{
    if(plane->values.size()!=4)
        return -1;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr left(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr right(new pcl::PointCloud<PointT>());

    Eigen::Vector3f np(plane->values[0],plane->values[1],plane->values[2]);
    Eigen::Vector3f vAixsPro=(vAixs-((vAixs.dot(np))*np.normalized())).normalized();
    Eigen::Vector3f vPlaneCut = (vAixsPro.cross(np)).normalized();

    PointT minP,maxP;
    getPointCloudMinMax3D(cloudIn,minP,maxP);
    PointT pmin(minP.x,minP.y,(maxP.z+minP.z)/2),pmax(maxP.x,maxP.y,maxP.z);
    cutPointCloud(cloudIn,temp,pmin,pmax);

    getPointCloudMinMax3D(temp,minP,maxP);
    PointT pmin1(minP.x,minP.y,maxP.z-500),pmax1(minP.x+20,maxP.y,maxP.z);
    cutPointCloud(temp,left,pmin1,pmax1);

   // getPointCloudMinMax3D(temp,minP,maxP);
    PointT pmin2(maxP.x-20,minP.y,maxP.z-500),pmax2(maxP.x,maxP.y,maxP.z);
    cutPointCloud(temp,right,pmin2,pmax2);

    getXZSurfacePointCloud(left,left,vAixsPro,vPlaneCut,1,false);
    getXZSurfacePointCloud(right,right,vAixsPro,vPlaneCut,1,true);

    segmentationLine(left,1000,1.0,inliers,lineLeft);
    segmentationLine(right,1000,1.0,inliers,lineRight);
    return 0;
}

/**********************************************
* @projectName   ThiVision::getDownLeftAndRightLine
* @brief         得到下半件底部左右边界直线
* @inParam       cloudIn: 输入已经投影和分割有效区域后的点云
*                length: 去除前槽后剩下的打磨长度
*                grooveWidth: 前槽长
*                plane: 输入的投影平面
*                vAixs: 轴向量(截平面向量)
* @outParam      lineLeft: 输出的下左边界直线
*                lineRight: 输出的下右边界直线
* @return        平面参数错误返回-1，成功返回0
* @author        xs(1874020422@qq.com)
* @date          2025-01-07
**********************************************/
int ThiVision::getDownLeftAndRightLine(pcl::PointCloud<PointT>::Ptr &cloudIn,float length,float grooveWidth,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vAixs,pcl::ModelCoefficients::Ptr &lineLeft,pcl::ModelCoefficients::Ptr &lineRight)
{
    if(plane->values.size()!=4)
        return -1;

    PointT minP,maxP;
    getPointCloudMinMax3D(cloudIn,minP,maxP);
    PointT min(minP.x,minP.y,minP.z);
    PointT max(maxP.x,maxP.y,minP.z+10);
    pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>());
    cutPointCloud(cloudIn,temp,min,max);

    PointT pMinX = temp->points[0];
    for (unsigned long i=1;i<temp->size();i++)
    {
        if(pMinX.x>temp->points[i].x)
        {
            pMinX=temp->points[i];
        }
    }

    Eigen::Vector3f np(plane->values[0],plane->values[1],plane->values[2]);
    Eigen::Vector3f vAixsPro=(vAixs-((vAixs.dot(np))*np.normalized())).normalized();
    Eigen::Vector3f vPlaneCut = (vAixsPro.cross(np)).normalized();

    lineLeft->values.resize(6);
    lineRight->values.resize(6);

    lineLeft->values[0] = pMinX.x+grooveWidth*vAixsPro[0];
    lineLeft->values[1] = pMinX.y+grooveWidth*vAixsPro[1];
    lineLeft->values[2] = pMinX.z+grooveWidth*vAixsPro[2];
    lineLeft->values[3] = vPlaneCut[0];
    lineLeft->values[4] = vPlaneCut[1];
    lineLeft->values[5] = vPlaneCut[2];

    lineRight->values[0] = pMinX.x+(grooveWidth+length)*vAixsPro[0];
    lineRight->values[1] = pMinX.y+(grooveWidth+length)*vAixsPro[1];
    lineRight->values[2] = pMinX.z+(grooveWidth+length)*vAixsPro[2];
    lineRight->values[3] = vPlaneCut[0];
    lineRight->values[4] = vPlaneCut[1];
    lineRight->values[5] = vPlaneCut[2];

    return 0;

}

/**********************************************
* @projectName   ThiVision::getXZSurfacePointCloud
* @brief         在XZ平面提取点云的表面点
* @inParam       cloudIn: 输入的点云
*                vCutLine: 切割平面的基础线方向向量
*                vCutPlane: 截平面向量 可以和上面的基础线构成切割面
*                ratio: 切割精度 平面每次平移的长度
*                isRight: 是否保留锁夹右侧表面点 即保留x大的点云 否则保留x小的点
* @outParam
*
* @return        参数错误返回-1，成功返回0
* @author        xs(1874020422@qq.com)
* @date          2025-01-07
**********************************************/
int ThiVision::getXZSurfacePointCloud(pcl::PointCloud<PointT>::Ptr &cloudIn,pcl::PointCloud<PointT>::Ptr &cloudOut,Eigen::Vector3f vCutLine,Eigen::Vector3f vCutPlane,float ratio,bool isRight)
{
    PointT minP,maxP;
    pcl::ModelCoefficients::Ptr planeCut(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr lineCut(new pcl::ModelCoefficients);

    PointT pMinZ = cloudIn->points[0];
    for (unsigned long i=1;i<cloudIn->size();i++)
    {
        if(pMinZ.z>cloudIn->points[i].z)
        {
            pMinZ=cloudIn->points[i];
        }
    }

    lineCut->values.resize(6);
    lineCut->values[0]=pMinZ.x;
    lineCut->values[1]=pMinZ.y;
    lineCut->values[2]=pMinZ.z;
    lineCut->values[3]=vCutLine[0];
    lineCut->values[4]=vCutLine[1];
    lineCut->values[5]=vCutLine[2];
    createSectionPlane(lineCut,vCutPlane,planeCut);
    getPointCloudMinMax3D(cloudIn,minP,maxP);

    unsigned long n = static_cast<unsigned long>((maxP.z-minP.z)/ratio);

    for(unsigned long i=0;i<n;i++)
    {
        getPlaneUpOrDownPointCloud(planeCut,cloudIn,temp,Eigen::Vector3f(0,0,1),true);
        movePlaneByAix(planeCut,planeCut,Eigen::Vector3f(0,0,1),ratio);
        getPlaneUpOrDownPointCloud(planeCut,temp,temp,Eigen::Vector3f(0,0,1),false);

        if(temp->size()!=0)
        {
            PointT pMinX = temp->points[0];
            if(isRight)
            {
                for (unsigned long j=0;j<temp->size();j++)
                {
                    if(pMinX.x<temp->points[j].x)
                    {
                        pMinX=temp->points[j];
                    }
                }
            }
            else
            {
                for (unsigned long j=0;j<temp->size();j++)
                {
                    if(pMinX.x>temp->points[j].x)
                    {
                        pMinX=temp->points[j];
                    }
                }
            }
            out->push_back(pMinX);
        }

    }

    cloudOut->clear();
    *cloudOut=*out;
    return 0;
}

/**********************************************
* @projectName   ThiVision::getBorderWithBaseOffset
* @brief         得到打磨基路径点关于左右边界的偏移值
* @inParam       cloudIn:    输入的基路径点云
*                boundaryIndex: 斜线点云分界索引
*                plane: 输入的投影平面
*                vAixs: 轴向量(截平面向量)
*                lineEarCutPro: 耳板分界线在立面的投影线
*                lineUpLeft: 左上边界线
*                lineUpRight: 右上边界线
*                lineDownLeft: 左下边界线
*                lineDownRight: 右下边界线
*                kIsPlus: 耳板分界线斜率是否为正
* @outParam      distanceLeft: 左边边界的距离集合
*                distanceRight: 右边边界的距离集合
* @return        平面参数错误返回-1，成功返回0
* @author        xs(1874020422@qq.com)
* @date          2025-01-15
**********************************************/
int ThiVision::getBorderWithBaseOffset(pcl::PointCloud<PointT>::Ptr &cloudIn,unsigned long boundaryIndex,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vAixs,
                                       pcl::ModelCoefficients::Ptr &lineEarCutPro,pcl::ModelCoefficients::Ptr &lineUpLeft,pcl::ModelCoefficients::Ptr &lineUpRight,
                                       pcl::ModelCoefficients::Ptr &lineDownLeft,pcl::ModelCoefficients::Ptr &lineDownRight,std::vector<float> &distanceLeft,std::vector<float> &distanceRight,bool kIsPlus)
{
    if(plane->values.size()!=4)
        return -1;

    Eigen::Vector3f np(plane->values[0],plane->values[1],plane->values[2]);
    Eigen::Vector3f vAixsPro=(vAixs-((vAixs.dot(np))*np.normalized())).normalized();

    pcl::ModelCoefficients::Ptr lineCur(new pcl::ModelCoefficients);
    lineCur->values.resize(6);

    PointT pointUpLeft,pointUpRight,pointDownLeft,pointDownRight;

    for (unsigned long i=0;i<cloudIn->size();i++)
    {
        lineCur->values[0]=cloudIn->points[i].x;
        lineCur->values[1]=cloudIn->points[i].y;
        lineCur->values[2]=cloudIn->points[i].z;
        lineCur->values[3]=vAixsPro[0];
        lineCur->values[4]=vAixsPro[1];
        lineCur->values[5]=vAixsPro[2];

        getLineLineIntersectPoint(lineCur,lineUpLeft,pointUpLeft);
        getLineLineIntersectPoint(lineCur,lineDownLeft,pointDownLeft);

        if(kIsPlus)
        {
            getLineLineIntersectPoint(lineCur,lineUpRight,pointUpRight);
        }
        else
        {
            PointT p1,p2;
            getLineLineIntersectPoint(lineCur,lineUpRight,p1);
            getLineLineIntersectPoint(lineCur,lineEarCutPro,p2);

            pointUpRight = (p1.x<p2.x)?p1:p2;

        }
        getLineLineIntersectPoint(lineCur,lineDownRight,pointDownRight);


        float distance=0;
        if(pointUpLeft.x<pointDownLeft.x)
        {
            getPointPointDistance(cloudIn->points[i],pointUpLeft,distance);
            if(pointUpLeft.x<cloudIn->points[i].x)
            {
                distance=-distance;
            }
        }
        else
        {
            getPointPointDistance(cloudIn->points[i],pointDownLeft,distance);
            if(pointDownLeft.x<cloudIn->points[i].x)
            {
                distance=-distance;
            }
        }

        if(i>=boundaryIndex&&kIsPlus)
        {

            distanceLeft.push_back(0);
        }

        else
        {
            distanceLeft.push_back(distance);
        }


        float dUR,dDR;
        getPointPointDistance(cloudIn->points[i],pointUpRight,dUR);
        getPointPointDistance(cloudIn->points[i],pointDownRight,dDR);

        distance = (dUR<dDR)?dUR:dDR;

        distanceRight.push_back(distance);

    }

    return 0;
}

/**********************************************
* @projectName  ThiVision::getWordsWithBaseOffset
* @brief        得到下半件每片铸造字的偏差
* @inParam      cloudIn: 输入的基础点位
*               plane: 立面平面参数
*               vAixs: 轴向量
*               wordAreaPoints: 铸造字区域
*               leftDistances:  每片铸造字左区域偏差集合
*               rightDistances: 每片铸造字右区域偏差集合
* @outParam
*               leftIndexs: 每片铸造字左区域索引集合
*               rightIndexs: 每片铸造字右区域索引集合
* @return       平面参数错误返回-1，成功返回0
* @author        xs(1874020422@qq.com)
* @date          2025-01-15
**********************************************/
int ThiVision::getWordsWithBaseOffset(pcl::PointCloud<PointT>::Ptr &cloudIn,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vAixs,std::vector<pcl::PointCloud<PointT>::Ptr> wordAreaPoints,std::vector<std::vector<float>> &leftDistances,std::vector<std::vector<float>> &rightDistances,std::vector<std::vector<unsigned long>> &leftIndexs,std::vector<std::vector<unsigned long>> &rightIndexs)
{
    if(plane->values.size()!=4)
        return -1;
    for(unsigned long index=0;index<wordAreaPoints.size();index++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr word(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr wordSort(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<pcl::ModelCoefficients::Ptr> Lines;

        Eigen::Vector3f np(plane->values[0],plane->values[1],plane->values[2]);
        Eigen::Vector3f vAixsPro=(vAixs-((vAixs.dot(np))*np.normalized())).normalized();

        std::vector<float> leftDistance;
        std::vector<float> rightDistance;
        std::vector<unsigned long> leftIndex;
        std::vector<unsigned long> rightIndex;

        //区域投影
        planarProjectionPointCloud(wordAreaPoints[index],word,plane);

        //点云排序 1 2
        //为      0 3
        unsigned long indexMaxX[4]={0,1,2,3};
        for(unsigned long i=0;i<word->size()-1;i++)
        {
            for (unsigned long j=0;j<word->size()-i-1;j++)
            {
                if(word->points[indexMaxX[j]].x>word->points[indexMaxX[j+1]].x)
                {
                    unsigned long tempIndex=indexMaxX[j];
                    indexMaxX[j]=indexMaxX[j+1];
                    indexMaxX[j+1]=tempIndex;
                }
            }
        }

        if(word->points[indexMaxX[0]].z>word->points[indexMaxX[1]].z)
        {
            unsigned long tempIndex=indexMaxX[0];
            indexMaxX[0]=indexMaxX[1];
            indexMaxX[1]=tempIndex;
        }

        if(word->points[indexMaxX[2]].z<word->points[indexMaxX[3]].z)
        {
            unsigned long tempIndex=indexMaxX[2];
            indexMaxX[2]=indexMaxX[3];
            indexMaxX[3]=tempIndex;
        }

        for (unsigned long i=0;i<4;i++)
        {
            wordSort->push_back(word->points[indexMaxX[i]]);
        }

        //

        //求铸造字的边界线
        for (unsigned long i=0;i<4;i++)
        {
            pcl::ModelCoefficients::Ptr line(new pcl::ModelCoefficients);
            line->values.resize(6);

            unsigned long frontIndex=i,backIndex=i+1;
            backIndex=(frontIndex==3)?0:backIndex;

            Eigen::Vector3f vLine(wordSort->points[backIndex].x-wordSort->points[frontIndex].x,
                                  wordSort->points[backIndex].y-wordSort->points[frontIndex].y,
                                  wordSort->points[backIndex].z-wordSort->points[frontIndex].z);
            vLine=vLine.normalized();

            line->values[0]=wordSort->points[frontIndex].x;
            line->values[1]=wordSort->points[frontIndex].y;
            line->values[2]=wordSort->points[frontIndex].z;
            line->values[3]=vLine[0];
            line->values[4]=vLine[1];
            line->values[5]=vLine[2];

            Lines.push_back(line);
        }

        //计算偏差距离
        for (unsigned long i=0;i<cloudIn->size();i++)
        {

            std::vector<PointT> ps;
            pcl::ModelCoefficients::Ptr line(new pcl::ModelCoefficients);
            line->values.resize(6);

            line->values[0]=cloudIn->points[i].x;
            line->values[1]=cloudIn->points[i].y;
            line->values[2]=cloudIn->points[i].z;
            line->values[3]=vAixsPro[0];
            line->values[4]=vAixsPro[1];
            line->values[5]=vAixsPro[2];

            PointT p[4];

            //保留直线端点内的交点
            for (unsigned long j=0;j<4;j++)
            {
                //pcl::lineToLineSegment 函数对平行的直线 求取的是最小公垂足
                Eigen::Vector3f vlj(Lines[j]->values[3],Lines[j]->values[4],Lines[j]->values[5]);
                float athe = abs(acos(vAixsPro.dot(vlj))*static_cast<float>(180/M_PI));

                if(athe>5&&athe<175)
                {
                    unsigned long frontIndex=j,backIndex=j+1;
                    backIndex=(frontIndex==3)?0:backIndex;

                    getLineLineIntersectPoint(line,Lines[j],p[j]);

                    PointT midP((wordSort->points[frontIndex].x+wordSort->points[backIndex].x)/2,
                                (wordSort->points[frontIndex].y+wordSort->points[backIndex].y)/2,
                                (wordSort->points[frontIndex].z+wordSort->points[backIndex].z)/2);
                    float midD,pD;
                    getPointPointDistance(midP,wordSort->points[frontIndex],midD);
                    getPointPointDistance(midP,p[j],pD);

                    if(pD<=midD)
                    {
                        ps.push_back(p[j]);
                    }
                }

            }
            //

            //留下x小的交点 为左边界点
            if(ps.size() ==2) //1.24 注意这个==2  还有只有一个交点的情况哦
            {
                PointT pMinX = ps[0];
                for (unsigned long j=1;j<ps.size();j++)
                {
                    if(pMinX.x>ps[j].x)
                    {
                        pMinX=ps[j];
                    }
                }
                float distance;
                getPointPointDistance(cloudIn->points[i],pMinX,distance);
                if(cloudIn->points[i].x>pMinX.x)
                    distance=-distance;
                leftDistance.push_back(distance);
                leftIndex.push_back(i);
            }

            //留下x大的交点 为右边界点
            if(ps.size() ==2)
            {
                PointT pMaxX = ps[0];
                for (unsigned long j=1;j<ps.size();j++)
                {
                    if(pMaxX.x<=ps[j].x)
                    {
                        pMaxX=ps[j];
                    }
                }
                float distance;
                getPointPointDistance(cloudIn->points[i],pMaxX,distance);
                if(cloudIn->points[i].x>pMaxX.x)
                    distance=-distance;
                rightDistance.push_back(distance);
                rightIndex.push_back(i);
            }
        }
        //

        leftDistances.push_back(leftDistance);
        rightDistances.push_back(rightDistance);
        leftIndexs.push_back(leftIndex);
        rightIndexs.push_back(rightIndex);

    }
    return 0;
}

/**********************************************
* @projectName   ThiVision::getDownTrackPoints
* @brief         根据下半件的边缘线和铸造字线生成路径点
* @inParam
*                basePoints: 输入的基路径点
*                baseVector:	输入的基路径向量
*                plane:	投影的立面
*                vAixs:	轴向量
*                leftWordsDistances: 多片铸造字左边界偏差集合
*                rightWordsDistances: 多片铸造字右边界偏差集合
*                leftWordsIndexs: 多片铸造字左边界索引集合
*                rightWordsIndexs: 多片铸造字右边界索引集合
*                distanceLineLeft: 左边界线偏差集合
*                distanceLineRight: 右边界线偏差集合
* @outParam
*                outTrackPoint: 输出的排序好的路径点
*                outTrackVector: 输出的排序好的路径点向量
* @return        成功返回0
* @author        xs(1874020422@qq.com)
* @date          2025-01-15
**********************************************/
int ThiVision::getDownTrackPoints(pcl::PointCloud<PointT>::Ptr &basePoints,pcl::PointCloud<PointT>::Ptr &baseVector,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vAixs,
                                  std::vector<std::vector<float>> &leftWordsDistances,std::vector<std::vector<float>> &rightWordsDistances,
                                  std::vector<std::vector<unsigned long>> &leftWordsIndexs,std::vector<std::vector<unsigned long>> &rightWordsIndexs,
                                  std::vector<float> &distanceLineLeft,std::vector<float> &distanceLineRight,std::vector<std::vector<PointT>> &outTrackPoint,std::vector<std::vector<PointT>> &outTrackVector)
{
    std::vector<PointT> outPs1,outVs1;
    std::vector<PointT> outPs2,outVs2;
    std::vector<PointT> outPs3,outVs3;

    Eigen::Vector3f np(plane->values[0],plane->values[1],plane->values[2]);
    Eigen::Vector3f vAixsPro=(vAixs-((vAixs.dot(np))*np.normalized())).normalized();


    //一片区域
    if(leftWordsIndexs.size()==1)
    {
        unsigned long index=0;
        std::vector<PointT> points1,v1;
        std::vector<PointT> points2,v2;
        std::vector<PointT> points3,v3;
        std::vector<PointT> points4,v4;

        for(unsigned long i=0;i<basePoints->size();i++)
        {
            PointT p;
            p.x=basePoints->points[i].x+distanceLineLeft[i]*vAixsPro[0];
            p.y=basePoints->points[i].y+distanceLineLeft[i]*vAixsPro[1];
            p.z=basePoints->points[i].z+distanceLineLeft[i]*vAixsPro[2];

            points1.push_back(p);
            v1.push_back(baseVector->points[i]);

            bool isFind = findWordIndex(leftWordsIndexs[0],i,index);
            if(isFind)
            {
                p.x=basePoints->points[i].x+leftWordsDistances[0][index]*vAixsPro[0];
                p.y=basePoints->points[i].y+leftWordsDistances[0][index]*vAixsPro[1];
                p.z=basePoints->points[i].z+leftWordsDistances[0][index]*vAixsPro[2];

                points2.push_back(p);
                v2.push_back(baseVector->points[i]);
            }

            else
            {
                p.x=basePoints->points[i].x+distanceLineRight[i]*vAixsPro[0];
                p.y=basePoints->points[i].y+distanceLineRight[i]*vAixsPro[1];
                p.z=basePoints->points[i].z+distanceLineRight[i]*vAixsPro[2];

                points2.push_back(p);
                v2.push_back(baseVector->points[i]);
            }

            isFind = findWordIndex(rightWordsIndexs[0],i,index);
            if(isFind)
            {
                if(rightWordsDistances[0][index]<distanceLineRight[i])
                {
                    p.x=basePoints->points[i].x+rightWordsDistances[0][index]*vAixsPro[0];
                    p.y=basePoints->points[i].y+rightWordsDistances[0][index]*vAixsPro[1];
                    p.z=basePoints->points[i].z+rightWordsDistances[0][index]*vAixsPro[2];

                    points3.push_back(p);
                    v3.push_back(baseVector->points[i]);

                    p.x=basePoints->points[i].x+distanceLineRight[i]*vAixsPro[0];
                    p.y=basePoints->points[i].y+distanceLineRight[i]*vAixsPro[1];
                    p.z=basePoints->points[i].z+distanceLineRight[i]*vAixsPro[2];

                    points4.push_back(p);
                    v4.push_back(baseVector->points[i]);
                }

            }

        }

        if(points1.size()!=0&&points2.size()!=0)
        {
            tidyKeyPoints(points1,v1,points2,v2,outPs1,outVs1);
            outTrackPoint.push_back(outPs1);
            outTrackVector.push_back(outVs1);
        }

        if(points3.size()!=0&&points4.size()!=0)
        {
            tidyKeyPoints(points3,v3,points4,v4,outPs2,outVs2);
            outTrackPoint.push_back(outPs2);
            outTrackVector.push_back(outVs2);
        }
    }

    //两片区域
    else if (leftWordsIndexs.size()==2)
    {
        std::vector<PointT> points1,v1;
        std::vector<PointT> points2,v2;
        std::vector<PointT> points3,v3;
        std::vector<PointT> points4,v4;
        std::vector<PointT> points5,v5;
        std::vector<PointT> points6,v6;

        //提前便利 取出左右 铸造字
        unsigned long lAreaIndex=0,rAreaIndex=1;
        for(unsigned long i=0;i<basePoints->size();i++)
        {
            unsigned long index1=0,index2=0;
            bool useL1 = findWordIndex(leftWordsIndexs[0],i,index1);
            bool useL2 = findWordIndex(leftWordsIndexs[1],i,index2);
            if(useL1&&useL2)
            {
                rAreaIndex = (leftWordsDistances[0][index1]<leftWordsDistances[1][index2])?1:0;
                lAreaIndex =  (leftWordsDistances[0][index1]<leftWordsDistances[1][index2])?0:1;
                break;
            }
        }

        for(unsigned long i=0;i<basePoints->size();i++)
        {
            unsigned long index1=0,index2=0;
            unsigned long leftAreaIndex=lAreaIndex,rightAreaIndex=rAreaIndex,comIndex=0;

            PointT p;
            p.x=basePoints->points[i].x+distanceLineLeft[i]*vAixsPro[0];
            p.y=basePoints->points[i].y+distanceLineLeft[i]*vAixsPro[1];
            p.z=basePoints->points[i].z+distanceLineLeft[i]*vAixsPro[2];

            points1.push_back(p);
            v1.push_back(baseVector->points[i]);

            float minD=distanceLineRight[i];
            bool useL1 = findWordIndex(leftWordsIndexs[0],i,index1);
            bool useL2 = findWordIndex(leftWordsIndexs[1],i,index2);
            if(useL1&&useL2)
            {
                minD = (leftWordsDistances[0][index1]<leftWordsDistances[1][index2])?leftWordsDistances[0][index1]:leftWordsDistances[1][index2];
                rightAreaIndex = (leftWordsDistances[0][index1]<leftWordsDistances[1][index2])?1:0;
                leftAreaIndex =  (leftWordsDistances[0][index1]<leftWordsDistances[1][index2])?0:1;
                comIndex = (leftWordsDistances[0][index1]<leftWordsDistances[1][index2])?index1:index2;
            }
            else if (useL1&&!useL2)
            {
                minD = leftWordsDistances[0][index1];
            }
            else if (!useL1&&useL2)
            {
                minD = leftWordsDistances[1][index2];

            }

            p.x=basePoints->points[i].x+minD*vAixsPro[0];
            p.y=basePoints->points[i].y+minD*vAixsPro[1];
            p.z=basePoints->points[i].z+minD*vAixsPro[2];

            points2.push_back(p);
            v2.push_back(baseVector->points[i]);


            bool useR1 = findWordIndex(rightWordsIndexs[leftAreaIndex],i,index1);
            if(useR1)
            {
                if(useL1&&useL2)
                {
                    minD = leftWordsDistances[rightAreaIndex][comIndex];
                }
                else
                {
                    minD =  distanceLineRight[i];
                }

                if(minD>rightWordsDistances[leftAreaIndex][index1])
                {
                    p.x=basePoints->points[i].x+rightWordsDistances[leftAreaIndex][index1]*vAixsPro[0];
                    p.y=basePoints->points[i].y+rightWordsDistances[leftAreaIndex][index1]*vAixsPro[1];
                    p.z=basePoints->points[i].z+rightWordsDistances[leftAreaIndex][index1]*vAixsPro[2];

                    points3.push_back(p);
                    v3.push_back(baseVector->points[i]);


                    p.x=basePoints->points[i].x+minD*vAixsPro[0];
                    p.y=basePoints->points[i].y+minD*vAixsPro[1];
                    p.z=basePoints->points[i].z+minD*vAixsPro[2];

                    points4.push_back(p);
                    v4.push_back(baseVector->points[i]);
                }

            }



            bool useR2 = findWordIndex(rightWordsIndexs[rightAreaIndex],i,index2);

            if(useR2)
            {
                if(distanceLineRight[i]>rightWordsDistances[rightAreaIndex][index2])
                {
                    p.x=basePoints->points[i].x+rightWordsDistances[rightAreaIndex][index2]*vAixsPro[0];
                    p.y=basePoints->points[i].y+rightWordsDistances[rightAreaIndex][index2]*vAixsPro[1];
                    p.z=basePoints->points[i].z+rightWordsDistances[rightAreaIndex][index2]*vAixsPro[2];

                    points5.push_back(p);
                    v5.push_back(baseVector->points[i]);

                    p.x=basePoints->points[i].x+distanceLineRight[i]*vAixsPro[0];
                    p.y=basePoints->points[i].y+distanceLineRight[i]*vAixsPro[1];
                    p.z=basePoints->points[i].z+distanceLineRight[i]*vAixsPro[2];

                    points6.push_back(p);
                    v6.push_back(baseVector->points[i]);
                }

            }


        }

        if(points1.size()!=0&&points2.size()!=0)
        {
            tidyKeyPoints(points1,v1,points2,v2,outPs1,outVs1);
            outTrackPoint.push_back(outPs1);
            outTrackVector.push_back(outVs1);
        }
        if(points3.size()!=0&&points4.size()!=0)
        {
            tidyKeyPoints(points3,v3,points4,v4,outPs2,outVs2);
            outTrackPoint.push_back(outPs2);
            outTrackVector.push_back(outVs2);
        }
        if(points5.size()!=0&&points6.size()!=0)
        {
            tidyKeyPoints(points5,v5,points6,v6,outPs3,outVs3);
            outTrackPoint.push_back(outPs3);
            outTrackVector.push_back(outVs3);
        }

    }

    else
    {
        return -1;
    }

    return 0;


}

/**********************************************
* @projectName   ThiVision::findWordIndex
* @brief         在铸造字索引列表中寻找一个索引，并得到在该列表的索引位置
* @inParam       index: 输入的索引列表
*                i: 需要寻找的索引
* @outParam
*                indexIndex: 该列表的索引位置
* @return        找到返回 true，否则返回 false
* @author        xs(1874020422@qq.com)
* @date          2025-01-15
**********************************************/
bool ThiVision::findWordIndex(std::vector<unsigned long> index, unsigned long i, unsigned long &indexIndex)
{
    for(unsigned long j=0;j<index.size();j++)
    {
        if(i==index[j])
        {
            indexIndex=j;
            return true;
        }
    }
    return false;
}

/**********************************************
* @projectName   ThiVision::resetAvoidUp
* @brief         重新设置下半件上避障长度(主要是由于k小于0的耳板分界线的避障长度歧义问题)
* @inParam       pEar1: 耳板分界线第一个点
*                pEar2: 耳板分界线第二个点
*                plane: 投影的立面
*                vAixs: 轴向量
*                AvoidIn: 输入的避障长度(这个一般是上避障长度)
*                kIsPlus: 斜率k是否为正数
* @outParam      AvoidOut: 输出的避障长度
* @return        成功返回0 平面参数错误返回-1
* @author        xs(1874020422@qq.com)
* @date          2025-01-15
**********************************************/
int ThiVision::resetAvoidUp(PointT pEar1, PointT pEar2, const pcl::ModelCoefficients::Ptr plane, Eigen::Vector3f vAixs, float &AvoidIn, float &AvoidOut,bool kIsPlus)
{

    if(plane->values.size()!=4)
        return -1;
    if(!kIsPlus)
    {
        Eigen::Vector3f np(plane->values[0],plane->values[1],plane->values[2]);
        Eigen::Vector3f vAixsPro=(vAixs-((vAixs.dot(np))*np.normalized())).normalized();
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcEarLine(new pcl::PointCloud<pcl::PointXYZ>);
        pcEarLine->push_back(pEar1);
        pcEarLine->push_back(pEar2);
        planarProjectionPointCloud(pcEarLine,pcEarLine,plane);

        Eigen::Vector3f lnPro(pcEarLine->points[0].x-pcEarLine->points[1].x,pcEarLine->points[0].y-pcEarLine->points[1].y,pcEarLine->points[0].z-pcEarLine->points[1].z);
        lnPro=lnPro.normalized();

        float cosAthe = vAixsPro.dot(lnPro);
        float sinAthe = std::abs(std::sqrt(1-cosAthe*cosAthe));
        float dp12;
        getPointPointDistance(pcEarLine->points[0],pcEarLine->points[1],dp12);

        AvoidOut = AvoidIn+dp12*sinAthe;
    }
    else

    {
        AvoidOut = AvoidIn;
    }
    return 0;

}

/**********************************************
* @projectName   ThiVision::resetAvoidDown
* @brief         重新设置下半件下避障长度(输入的下避障距离为距离耳板距离，等价换算为距离承压台交界线距离)
* @inParam       C: 侧面曲线结构体
*                width: 道宽
*                plane: 承压台平面
*                AvoidIn: 输入的避障长度(下避障长度)
*                isUse: 是否使用该函数重设避障长度
* @outParam
*                AvoidOut: 输出的避障长度
* @return        成功返回0 平面参数错误返回-1
* @author        xs(1874020422@qq.com)
* @date          2025-01-20
**********************************************/
int ThiVision::resetAvoidDown(Curve C,float width, const pcl::ModelCoefficients::Ptr plane, float &AvoidIn, float &AvoidOut,bool isUse)
{

    if(isUse)
    {
        if(plane->values.size()!=4)
            return -1;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for(unsigned long i=0;i<C.points.size();i++)
        {
            cloud->push_back(C.points[i]);
        }

        planarProjectionPointCloud(cloud,cloud,plane);

        float curD=0;
        unsigned long avoidDownIndex=cloud->size()-1;
        PointT pLast = cloud->points[cloud->size()-1];
        for (unsigned long i=1;i<cloud->size();i++)
        {
            PointT pCur = cloud->points[cloud->size()-i-1];
            getPointPointDistance(pLast,pCur,curD);
            if(curD>=AvoidIn)
            {
                avoidDownIndex=cloud->size()-i-1;
                break;
            }
        }

        AvoidOut = avoidDownIndex*width;
    }
    else
    {
        AvoidOut= AvoidIn;
    }

    return 0;
}

/**********************************************
* @projectName   ThiVision::getEarCutLineKeyPoints
* @brief         得到耳板分界线的关键点的投影和平面投影线(只对斜率大于0的有效)
* @inParam       pEar1: 耳板分界线第一个点
*                pEar2: 耳板分界线第二个点
*                plane: 投影的立面
*                vAixs: 轴向量
*                width: 道宽(函数里面会根据角度变换的)
*                lineUpLeft: 左上边界线
*                lineUpRight: 右上边界线
* @outParam
*                keyCloud: 得到的关键分割点的投影
*                lineCutEarPro: 得到的平面投影耳板分割线
* @return        平面参数错误返回-1，成功返回0
* @author        xs(1874020422@qq.com)
* @date          2025-01-15
**********************************************/
int ThiVision::getEarCutLineKeyPoints(PointT pEar1,PointT pEar2,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vAixs,float width,pcl::ModelCoefficients::Ptr lineUpLeft,pcl::ModelCoefficients::Ptr lineUpRight,pcl::PointCloud<PointT>::Ptr &keyCloud,pcl::ModelCoefficients::Ptr &lineCutEarPro)
{
    if(plane->values.size()!=4)
        return -1;


    float dRL;
    PointT pUpLeft,pUpRight;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcEarLine(new pcl::PointCloud<pcl::PointXYZ>);
    pcEarLine->push_back(pEar1);
    pcEarLine->push_back(pEar2);
    planarProjectionPointCloud(pcEarLine,pcEarLine,plane);

    Eigen::Vector3f np(plane->values[0],plane->values[1],plane->values[2]);
    Eigen::Vector3f vAixsPro=(vAixs-((vAixs.dot(np))*np.normalized())).normalized();

    Eigen::Vector3f lnPro(pcEarLine->points[0].x-pcEarLine->points[1].x,pcEarLine->points[0].y-pcEarLine->points[1].y,pcEarLine->points[0].z-pcEarLine->points[1].z);
    lnPro=lnPro.normalized();

    float cosAthe = vAixsPro.dot(lnPro);
    float sinAthe = std::abs(std::sqrt(1-cosAthe*cosAthe));
    width = width/sinAthe;

    lineCutEarPro->values.resize(6);
    lineCutEarPro->values[0]=pcEarLine->points[0].x;
    lineCutEarPro->values[1]=pcEarLine->points[0].y;
    lineCutEarPro->values[2]=pcEarLine->points[0].z;
    lineCutEarPro->values[3]=lnPro(0);
    lineCutEarPro->values[4]=lnPro(1);
    lineCutEarPro->values[5]=lnPro(2);

    getLineLineIntersectPoint(lineCutEarPro,lineUpLeft,pUpLeft);
    getLineLineIntersectPoint(lineCutEarPro,lineUpRight,pUpRight);

    lnPro.x()=pUpRight.x-pUpLeft.x;
    lnPro.y()=pUpRight.y-pUpLeft.y;
    lnPro.z()=pUpRight.z-pUpLeft.z;
    lnPro= lnPro.normalized();

    getPointPointDistance(pUpLeft,pUpRight,dRL);


    unsigned long num = static_cast<unsigned long>(dRL/width);

    for (unsigned long i=0;i<num+1;i++)
    {
        PointT p(pUpLeft.x+lnPro(0)*width*(i+1),pUpLeft.y+lnPro(1)*width*(i+1),pUpLeft.z+lnPro(2)*width*(i+1));
        temp->push_back(p);
    }

    keyCloud->clear();
    *keyCloud=*temp;
    return 0;
}

/**********************************************
* @projectName   ThiVision::getEarCutBasePoints
* @brief         通过投影点得到真实的关键点
* @inParam       cloudIn: 输入的投影关键点
*                plane:  投影的立面
*                vPos:   统一避障姿态向量
*                lineEarCut: 耳板分割线
* @outParam
*                cloudOut:   输出的基础关键点
*                cloudVector: 输出的基础点姿态
* @return        成功返回0
* @author        xs(1874020422@qq.com)
* @date          2025-01-15
**********************************************/
int ThiVision::getEarCutBasePoints(pcl::PointCloud<PointT>::Ptr &cloudIn,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vPos,pcl::ModelCoefficients::Ptr lineEarCut,pcl::PointCloud<PointT>::Ptr &cloudOut,pcl::PointCloud<PointT>::Ptr &cloudVector)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr V(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcEarKeyOR(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ModelCoefficients::Ptr lAixs(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr lPlane(new pcl::ModelCoefficients);

    Eigen::Vector3f np(plane->values[0],plane->values[1],plane->values[2]);

    lAixs->values.resize(6);
    lPlane->values.resize(6);

    for (unsigned long i=0;i<cloudIn->size();i++)
    {
        np = np.normalized();
        lPlane->values[0] = cloudIn->points[i].x;
        lPlane->values[1] = cloudIn->points[i].y;
        lPlane->values[2] = cloudIn->points[i].z;
        lPlane->values[3] = np(0);
        lPlane->values[4] = np(1);
        lPlane->values[5] = np(2);

        PointT pBase;
        getLineLineIntersectPoint(lPlane,lineEarCut,pBase);
        pcEarKeyOR->push_back(pBase);
        V->push_back(PointT(vPos(0),vPos(1),vPos(2)));
    }

    cloudOut->clear();
    cloudVector->clear();

    *cloudOut = *pcEarKeyOR;
    *cloudVector = *V;
    return 0;
}

/**********************************************
* @projectName   ThiVision::fuseEarBaseAndCurve
* @brief         将曲线结构体和耳板分割线的基础点合成为一个新的并得到合成的边界索引(这个还可以用于合成姿态、投影点集)
* @inParam       cloudCurve: 曲线结构体点云
*                cloudEarKey: 耳板分割线点云
* @outParam
*                cloudOut: 输出合成的点云
*                boundaryIndex: 输出分割的边界索引
* @return        成功返回0
* @author        xs(1874020422@qq.com)
* @date          2025-01-15
**********************************************/
int ThiVision::fuseEarBaseAndCurve(pcl::PointCloud<PointT>::Ptr &cloudCurve,pcl::PointCloud<PointT>::Ptr &cloudEarKey,pcl::PointCloud<PointT>::Ptr &cloudOut,unsigned long &boundaryIndex)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    boundaryIndex = cloudCurve->size();

    for(unsigned long i=0;i<cloudCurve->size();i++)
    {
        temp->push_back(cloudCurve->points[i]);
    }

    for(unsigned long i=0;i<cloudEarKey->size();i++)
    {
        temp->push_back(cloudEarKey->points[i]);
    }
    cloudOut->clear();
    *cloudOut = *temp;
    return 0;
}




/**********************************************
* @projectName   ThiVision::createCylinder
* @brief         绘制圆柱面模型
* @inParam
*                coefficients: 圆柱模型参数(一共7个值)
*                massCenter: 圆柱质心的位置
*                numSides: 圆柱的梭边数(越大越精细)
*                scale: 圆柱的高
* @outParam      void
* @return        成功返回绘制的模型，失败返回nullptr
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
vtkSmartPointer<vtkPolyData> ThiVision::createCylinder(const pcl::ModelCoefficients &coefficients, Eigen::Vector3f massCenter, int numSides, float scale)
{
    if (coefficients.values.size() != 7)
    {
        PCL_WARN("[addCylinder] Coefficients size does not match expected size (expected 7).\n");
        return nullptr;
    }

    float P0[3], P2[3];
    P0[0] = coefficients.values[0];
    P0[1] = coefficients.values[1];
    P0[2] = coefficients.values[2];
    float dlambda = (massCenter[0] - P0[0])*coefficients.values[3] + (massCenter[1] - P0[1])*coefficients.values[4] + (massCenter[2] - P0[2])*coefficients.values[5];
    P2[0] = P0[0] + dlambda * coefficients.values[3];
    P2[1] = P0[1] + dlambda * coefficients.values[4];
    P2[2] = P0[2] + dlambda * coefficients.values[5];

    float pt[3];
    pt[0] = coefficients.values[3] * scale;
    pt[1] = coefficients.values[4] * scale;
    pt[2] = coefficients.values[5] * scale;
    vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
    line->SetPoint1(static_cast<double>(P2[0] - pt[0]), static_cast<double>(P2[1] - pt[1]), static_cast<double>(P2[2] - pt[2]));
    line->SetPoint2(static_cast<double>(P2[0] + pt[0]), static_cast<double>(P2[1] + pt[1]), static_cast<double>(P2[2] + pt[2]));

    vtkSmartPointer<vtkTubeFilter> tuber = vtkSmartPointer<vtkTubeFilter>::New();
    tuber->SetInputConnection(line->GetOutputPort());
    tuber->SetRadius(static_cast<double>(coefficients.values[6]));
    tuber->SetNumberOfSides(numSides);
    tuber->Update();

    return (tuber->GetOutput());
}

/**********************************************
* @projectName   ThiVision::createPlane
* @brief         创建要显示的平面
* @inParam
*                coefficients: 存储平面的平面方程
*                centralPoint: 平面的中心位置
*                w: 平面的宽度(默认100)
*                h: 平面的高度(默认100)
* @outParam      void
* @return        可以被addModelFromPolyData函数绘制的一个平面
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
vtkSmartPointer<vtkPolyData> ThiVision::createPlane(const pcl::ModelCoefficients &coefficients, PointT centralPoint, float w,float h)
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

/**********************************************
* @projectName   ThiVision::createLinePoint
* @brief         创建直线模型上一条线段的两个端点
* @inParam
*                coefficients: 直线模型
*                length: 线段长度
*                moveCentreLength: 线段中心沿着方向向量方向平移长度(默认为0)
* @outParam
*                p1: 输出的线段第一个点
*                p2: 输出的线段第二个点
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-09
**********************************************/
void ThiVision::createLinePoint(const pcl::ModelCoefficients::Ptr &coefficients, PointT &p1, PointT &p2, float length, float moveCentreLength)
{
    p1.x=coefficients->values[0]+(moveCentreLength+length/2)*coefficients->values[3];
    p1.y=coefficients->values[1]+(moveCentreLength+length/2)*coefficients->values[4];
    p1.z=coefficients->values[2]+(moveCentreLength+length/2)*coefficients->values[5];

    p2.x=coefficients->values[0]+(moveCentreLength-length/2)*coefficients->values[3];
    p2.y=coefficients->values[1]+(moveCentreLength-length/2)*coefficients->values[4];
    p2.z=coefficients->values[2]+(moveCentreLength-length/2)*coefficients->values[5];
}

/**********************************************
* @projectName   ThiVision::getVectorEndPoint
* @brief         已知一向量和起点求一定长度的终点
* @inParam       Vector: 该向量
*                origin: 起点
*                length: 向量长度
* @outParam
*                target: 终点坐标
* @return        void
* @author        xs(1874020422@qq.com)
* @date          2024-08-09
**********************************************/
void ThiVision::getVectorEndPoint(const PointT Vector,const PointT origin,float length,PointT &target)
{
    target.x = length*Vector.x+origin.x;  // 法向量的 x 分量
    target.y = length*Vector.y+origin.y;  // 法向量的 y 分量
    target.z = length*Vector.z+origin.z;  // 法向量的 z 分量
}



/**********************************************
* @projectName   ThiVision::floatIsEqual
* @brief         判断两个浮点数是否相等
* @inParam
*                a: 输入的一个浮点数
*                b: 输入的另一个浮点数
* @outParam      void
* @return        相等返回true,否则返回false
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
bool ThiVision::floatIsEqual(float a, float b)
{
    return (std::fabs(a-b)<1e-10f)?true:false;
}

/**********************************************
* @projectName   ThiVision::readTxtPointCloud
* @brief         从txt文件中读取点云,将点云文件转化为pcl::PointCloud<PointT>类
* @inParam
*                fileName: 点云文件路径
* @outParam      cloud: (输出)读取出来点云存储内存
* @return        成功返回0,否则返回-1
* @author        xs(1874020422@qq.com)
* @date          2024-08-08
**********************************************/
int ThiVision::readTxtPointCloud(const char* fileName, pcl::PointCloud<PointT>& cloud)
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
