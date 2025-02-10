#ifndef SURFACEHD100_H
#define SURFACEHD100_H

#include "3DCamera.hpp"
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>


typedef enum CAMERA_CODE
{
   CAMERA_SUCCESS = 0,
   CAMERA_CONNECT = -1,
   CAMERA_INFO = -2,
   CAMERA_DEPTH_INFO = -3,
   CAMERA_DEPTH_START = -4,
   CAMERA_DEPTH_INTRINSIC = -5,
   CAMERA_DEPTH_GETFRAME = -6,
   CAMERA_DEPTH_SCALE = -7,
   CAMERA_STOP = -8,
   CAMERA_RGB_INFO = -9,
   CAMERA_RGB_START = -10,
   CAMERA_RGB_GETFRAME = -11,
   CAMERA_STOP_DEPTH = -12,
   CAMERA_STOP_RGB = -13
}CAMERA_CODE;

class SurfaceHD100
{
public:
    SurfaceHD100();
    CAMERA_CODE CameraInit();
    CAMERA_CODE StartDepth();
    CAMERA_CODE StartRgb();
    void SetCameraParam();
    CAMERA_CODE GetPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    CAMERA_CODE GetRgb(cv::Mat &img);
    CAMERA_CODE Rgb2Point(int x,int y,pcl::PointXYZ point);
    CAMERA_CODE StopDepth();
    CAMERA_CODE stopRgb();
    CAMERA_CODE StopCamera();

private:
    cs::ICameraPtr camera;
    CameraInfo cameraInfo;
    std::vector<StreamInfo> streamInfosDepth;
    std::vector<StreamInfo> streamInfosRgb;
    Intrinsics intrinsicParamDepth;
    unsigned short* depth;

    bool IsEqual(float a, float b);
};

#endif // SURFACEHD100_H
