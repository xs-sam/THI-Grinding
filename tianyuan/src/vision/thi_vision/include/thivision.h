#ifndef THIVISION_H
#define THIVISION_H

#include <string>
#include <iostream>

//common
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>

//IO
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

//滤波
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

//分割
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_line.h>

//特征
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>

//表面
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>

//VTK
#include <vtkRenderWindow.h>
#include <vtkPlaneSource.h>
#include <vtkTubeFilter.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;

enum class FILE_TYPE
{
    PCD=0,
    PLY=1,
    TXT=2
};

enum class READ_STATE
{
    READ_SUCCEED=0,
    READ_FAIL=-1,
    FILE_INEXIST=-2,
    FILETYPE_ERROR=-3
};

struct WordArea
{
    char type;
    unsigned long index[4]={0,0,0,0};
};

struct TrackPoint
{
    std::vector<PointT> point;
    std::vector<PointT> vector;
};

struct LineTrimmer
{
    unsigned long curIndex;
    bool isAcross;
    TrackPoint curTrack;
    TrackPoint beforTrack;
};

//下半曲线
struct Curve
{
    std::vector<PointT> points; //曲线(折线)的关键点
    std::vector<Eigen::Vector3f> vectorLine; //每条折线的方向向量
    std::vector<Eigen::Vector3f> vectorNormal; //每条折线的法相
};


class ThiVision
{
public:
    ThiVision();
    READ_STATE readPointCloudFile(std::string fileName,FILE_TYPE fileType,pcl::PointCloud<PointT>::Ptr& cloud);
    void voxelPointCloud(pcl::PointCloud<PointT>::Ptr& cloudIn,pcl::PointCloud<PointT>::Ptr& cloudOut,Eigen::Vector3f size);
    void blowUpPointCloud(pcl::PointCloud<PointT>::Ptr& cloudIn,pcl::PointCloud<PointT>::Ptr& cloudOut,int num);
    void getPointCloudMinMax3D(pcl::PointCloud<PointT>::Ptr& cloudIn,PointT& minPoint,PointT& maxPoint);
    void cutPointCloud(pcl::PointCloud<PointT>::Ptr& cloudIn,pcl::PointCloud<PointT>::Ptr& cloudOut,PointT& minPoint,PointT& maxPoint);
    void statisticalPointCloud(pcl::PointCloud<PointT>::Ptr& cloudIn,pcl::PointCloud<PointT>::Ptr& cloudOut,int K, double stdThreshold=1.0);
    void copyPointCloud(pcl::PointCloud<PointT>::Ptr& cloudIn,pcl::PointCloud<PointT>::Ptr& cloudOut);
    void segmentationCylinder(pcl::PointCloud<PointT>::Ptr& cloudIn,int K,int maxIterations,double normalWeight,double distanceThreshold,double rMin,double rMax, pcl::PointIndices::Ptr &inliers,pcl::ModelCoefficients::Ptr &cylinderCoefficients);
    void segmentationPlane(pcl::PointCloud<PointT>::Ptr& cloudIn,int maxIterations,double distanceThreshold,pcl::PointIndices::Ptr &inliers,pcl::ModelCoefficients::Ptr &planeCoefficients);
    void segmentationLine(pcl::PointCloud<PointT>::Ptr& cloudIn,int maxIterations,double distanceThreshold,pcl::PointIndices::Ptr &inliers,pcl::ModelCoefficients::Ptr &lineCoefficients);
    void leaveIndexesPoint(pcl::PointCloud<PointT>::Ptr& cloudIn,pcl::PointCloud<PointT>::Ptr &cloudOut,pcl::PointIndices::Ptr inliers);
    void removeIndexesPoint(pcl::PointCloud<PointT>::Ptr& cloudIn,pcl::PointCloud<PointT>::Ptr &cloudOut,pcl::PointIndices::Ptr inliers);
    void planarProjectionPointCloud(pcl::PointCloud<PointT>::Ptr& cloudIn,pcl::PointCloud<PointT>::Ptr& cloudOut,pcl::ModelCoefficients::Ptr coefficients);
    void getPlanarConcaveHull(pcl::PointCloud<PointT>::Ptr& cloudIn,pcl::PointCloud<PointT>::Ptr& cloudOut,double alpha);
    void createSectionPlane(pcl::ModelCoefficients::Ptr& lineCoefficients,pcl::ModelCoefficients::Ptr& cylinderCoefficients,float length,pcl::ModelCoefficients::Ptr& planeCoefficients);
    void createSectionPlane(pcl::ModelCoefficients::Ptr& lineCoefficients,Eigen::Vector3f &vAixs,pcl::ModelCoefficients::Ptr& planeCoefficients);
    int getPlanePlaneLine(pcl::ModelCoefficients::Ptr plane1,pcl::ModelCoefficients::Ptr plane2,pcl::ModelCoefficients::Ptr &line);
    int getPlaneCylinderLine(pcl::ModelCoefficients::Ptr plane,pcl::ModelCoefficients::Ptr cylinder,std::vector<pcl::ModelCoefficients::Ptr> &line);
    int getArcCenterAngle(PointT pLeft, PointT pRight,pcl::ModelCoefficients::Ptr cylinder,PointT &center,float &angle);
    int getLineLineIntersectPoint(pcl::ModelCoefficients::Ptr &line1,pcl::ModelCoefficients::Ptr &line2,PointT &point);
    int getArcDividePoint(PointT pLeft, PointT pRight,pcl::ModelCoefficients::Ptr cylinder,float width,float avoidLeft,float avoidRight,Eigen::Vector3f avoidVectorLeft,Eigen::Vector3f avoidVectorRight,std::vector<PointT> &points,std::vector<PointT> &vector);
    int movePointsAxial(float length,pcl::ModelCoefficients::Ptr cylinder,std::vector<PointT> inPoints,std::vector<PointT> inVector,std::vector<PointT> &outPoints,std::vector<PointT> &outVector);
    int getPointLineDistance(const PointT point,pcl::ModelCoefficients::Ptr line,float &distance);
    int getPointPointDistance(const PointT point1,const PointT point2,float &distance);
    Eigen::Vector3f getRotatedVector(const Eigen::Vector3f& original, const Eigen::Vector3f& axis, double angle);
    int sortAreaPointCloud(pcl::PointCloud<PointT>::Ptr &cloudIn);
    int getTrackKeyPoints(std::vector<pcl::PointCloud<PointT>::Ptr> wordAreaPoints,pcl::ModelCoefficients::Ptr cylinder,pcl::ModelCoefficients::Ptr line,float length,std::vector<PointT> points,std::vector<PointT> vectors,std::vector<std::vector<PointT>> &listOutPoints,std::vector<std::vector<PointT>> &listOutVectors);
    int tidyKeyPoints(std::vector<PointT> points1,std::vector<PointT> vectors1,std::vector<PointT> points2,std::vector<PointT> vectors2,std::vector<PointT> &outPoints,std::vector<PointT> &outVectors);
    int pressTrackPoints(std::vector<PointT> points,std::vector<PointT> vectors,float press,std::vector<PointT> &outPoints);

    //
    float getPoint2PlaneDistance(const pcl::ModelCoefficients::Ptr coefficients, const PointT point);
    void passThroughForPlane(const pcl::ModelCoefficients::Ptr coefficients,const pcl::PointCloud<PointT>::Ptr &cloudIn,float minDistance,float maxDistance,pcl::PointCloud<PointT>::Ptr &cloudOut);
    int getPlaneUpOrDownPointCloud(pcl::ModelCoefficients::Ptr &plane,pcl::PointCloud<PointT>::Ptr &cloudIn,pcl::PointCloud<PointT>::Ptr &cloudOut,Eigen::Vector3f vAix,bool isUp);
    int movePlaneByAix(pcl::ModelCoefficients::Ptr &planeIn,pcl::ModelCoefficients::Ptr &planeOut,Eigen::Vector3f vAix,float length);
    int getSurfacePointCloud(pcl::PointCloud<PointT>::Ptr &cloudIn,pcl::PointCloud<PointT>::Ptr &cloudOut,pcl::ModelCoefficients::Ptr lineCutBase,Eigen::Vector3f vCut,float ratio,bool isRight);
    int getCurveInfo(pcl::PointCloud<PointT>::Ptr &cloudIn,float minDstance,Eigen::Vector3f vSection,Curve &C);
    int retainValidCurvePointCloud(pcl::PointCloud<PointT>::Ptr &cloudIn,const pcl::ModelCoefficients::Ptr plane,PointT pEar1,PointT pEar2,pcl::PointCloud<PointT>::Ptr &cloudOut,pcl::ModelCoefficients::Ptr &lineCutEar);
    int getCurveCutPoints(Curve &C,float width,Eigen::Vector3f vAixs,float avoidDown,float avoidUp,Eigen::Vector3f avoidVectorDown,Eigen::Vector3f avoidVectorUp,pcl::PointCloud<PointT>::Ptr &cloudOut,pcl::PointCloud<PointT>::Ptr &cloudVector);
    int retainDownWorkProjection(pcl::PointCloud<PointT>::Ptr &cloudIn,pcl::PointCloud<PointT>::Ptr &keyPointsIn,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vAixs,pcl::PointCloud<PointT>::Ptr &cloudOut);
    int getTopLeftAndRightLine(pcl::PointCloud<PointT>::Ptr &cloudIn,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vAixs,pcl::ModelCoefficients::Ptr &lineLeft,pcl::ModelCoefficients::Ptr &lineRight);
    int getDownLeftAndRightLine(pcl::PointCloud<PointT>::Ptr &cloudIn,float length,float grooveWidth,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vAixs,pcl::ModelCoefficients::Ptr &lineLeft,pcl::ModelCoefficients::Ptr &lineRight);
    int getXZSurfacePointCloud(pcl::PointCloud<PointT>::Ptr &cloudIn,pcl::PointCloud<PointT>::Ptr &cloudOut,Eigen::Vector3f vCutLine,Eigen::Vector3f vCutPlane,float ratio,bool isRight);
    int getBorderWithBaseOffset(pcl::PointCloud<PointT>::Ptr &cloudIn,unsigned long boundaryIndex,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vAixs,
                                pcl::ModelCoefficients::Ptr &lineEarCutPro,pcl::ModelCoefficients::Ptr &lineUpLeft,pcl::ModelCoefficients::Ptr &lineUpRight,
                                pcl::ModelCoefficients::Ptr &lineDownLeft,pcl::ModelCoefficients::Ptr &lineDownRight,std::vector<float> &distanceLeft,std::vector<float> &distanceRight,bool kIsPlus);
    int getWordsWithBaseOffset(pcl::PointCloud<PointT>::Ptr &cloudIn,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vAixs,std::vector<pcl::PointCloud<PointT>::Ptr> wordAreaPoints,
                               std::vector<std::vector<float>> &leftDistances,std::vector<std::vector<float>> &rightDistances,std::vector<std::vector<unsigned long>> &leftIndexs,std::vector<std::vector<unsigned long>> &rightIndexs);

    int getDownTrackPoints(pcl::PointCloud<PointT>::Ptr &basePoints,pcl::PointCloud<PointT>::Ptr &baseVector,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vAixs,
                           std::vector<std::vector<float>> &leftWordsDistances,std::vector<std::vector<float>> &rightWordsDistances,
                           std::vector<std::vector<unsigned long>> &leftWordsIndexs,std::vector<std::vector<unsigned long>> &rightWordsIndexs,
                           std::vector<float> &distanceLineLeft,std::vector<float> &distanceLineRight,std::vector<std::vector<PointT>> &outTrackPoint,std::vector<std::vector<PointT>> &outTrackVector);

    bool findWordIndex(std::vector<unsigned long> index,unsigned long i,unsigned long &indexIndex);
    int resetAvoidUp(PointT pEar1, PointT pEar2, const pcl::ModelCoefficients::Ptr plane, Eigen::Vector3f vAixs, float &AvoidIn, float &AvoidOut,bool kIsPlus);
    int resetAvoidDown(Curve C,float width,const pcl::ModelCoefficients::Ptr plane, float &AvoidIn, float &AvoidOut,bool isUse);
    int getEarCutLineKeyPoints(PointT pEar1,PointT pEar2,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vAixs,float width,pcl::ModelCoefficients::Ptr lineUpLeft,pcl::ModelCoefficients::Ptr lineUpRight,pcl::PointCloud<PointT>::Ptr &keyCloud,pcl::ModelCoefficients::Ptr &lineCutEarPro);
    int getEarCutBasePoints(pcl::PointCloud<PointT>::Ptr &cloudIn,const pcl::ModelCoefficients::Ptr plane,Eigen::Vector3f vPos,pcl::ModelCoefficients::Ptr lineEarCut,pcl::PointCloud<PointT>::Ptr &cloudOut,pcl::PointCloud<PointT>::Ptr &cloudVector);


    int fuseEarBaseAndCurve(pcl::PointCloud<PointT>::Ptr &cloudCurve,pcl::PointCloud<PointT>::Ptr &cloudEarKey,pcl::PointCloud<PointT>::Ptr &cloudOut,unsigned long &boundaryIndex);



    //

    vtkSmartPointer<vtkPolyData> createCylinder(const pcl::ModelCoefficients &coefficients, Eigen::Vector3f massCenter, int numSides, float scale);
    vtkSmartPointer<vtkPolyData> createPlane(const pcl::ModelCoefficients &coefficients, PointT centralPoint, float w=100,float h=100);
    void createLinePoint(const pcl::ModelCoefficients::Ptr &coefficients,PointT &p1,PointT &p2,float length,float moveCentreLength=0.0);
    void getVectorEndPoint(const PointT Vector,const PointT origin,float length,PointT &target);

private:
    bool floatIsEqual(float a, float b);
    int readTxtPointCloud(const char* fileName, pcl::PointCloud<PointT>& cloud);

private:
    WordArea forwardWordArea,reverseWordArea;
    LineTrimmer lineTrimmer;
};
#endif // THIVISION_H
