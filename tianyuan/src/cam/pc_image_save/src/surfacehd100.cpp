#include "surfacehd100.h"

SurfaceHD100::SurfaceHD100()
{
    camera = cs::getCameraPtr();
}

CAMERA_CODE SurfaceHD100::CameraInit()
{
    ERROR_CODE ret;

    ret = camera->connect();
    if(ret != SUCCESS)
      return CAMERA_CONNECT;

    ret = camera->getInfo(cameraInfo);
    if(ret != SUCCESS)
    {
      ret = camera->disconnect();
      if(ret != SUCCESS)
        return CAMERA_STOP;
      return CAMERA_INFO;
    }

    return CAMERA_SUCCESS;
}

CAMERA_CODE SurfaceHD100::StartDepth()
{
    ERROR_CODE ret;
    ret = camera->getStreamInfos(STREAM_TYPE_DEPTH,  streamInfosDepth);
    if(ret != SUCCESS)
    {
      ret = camera->disconnect();
      if(ret != SUCCESS)
        return CAMERA_STOP;
      return CAMERA_DEPTH_INFO;
    }


    for (auto streamInfo : streamInfosDepth)
    {
      if (streamInfo.format == STREAM_FORMAT_Z16)
      {
        ERROR_CODE ret = camera->startStream(STREAM_TYPE_DEPTH, streamInfo);
        if (ret != SUCCESS)
        {
          ret = camera->disconnect();
          if(ret != SUCCESS)
            return CAMERA_STOP;
          return CAMERA_DEPTH_START;
        }
        break;
      }
    }


    ret= camera->getIntrinsics(STREAM_TYPE_DEPTH, intrinsicParamDepth);
    if(ret != SUCCESS)
    {
      ret = camera->disconnect();
      if(ret != SUCCESS)
          return CAMERA_STOP;
      return CAMERA_DEPTH_INTRINSIC;
    }

    return CAMERA_SUCCESS;
}

CAMERA_CODE SurfaceHD100::StartRgb()
{
    ERROR_CODE ret;
    ret = camera->getStreamInfos(STREAM_TYPE_RGB,  streamInfosRgb);
    if(ret != SUCCESS)
    {
        ret = camera->disconnect();
        if(ret != SUCCESS)
          return CAMERA_STOP;
        return CAMERA_RGB_INFO;
    }


    for (auto streamInfo : streamInfosRgb)
    {
        if (streamInfo.format == STREAM_FORMAT_RGB8)
        {
            ERROR_CODE ret = camera->startStream(STREAM_TYPE_RGB, streamInfo);
            if (ret != SUCCESS)
            {
                ret = camera->disconnect();
                if(ret != SUCCESS)
                  return CAMERA_STOP;
                return CAMERA_RGB_START;
            }
            break;
        }
    }

    return CAMERA_SUCCESS;
}

void SurfaceHD100::SetCameraParam()
{
    PropertyExtension value;
    value.depthRange.min = 50;
    value.depthRange.max = 2000;
    value.algorithmContrast = 5;
    value.autoExposureMode = AUTO_EXPOSURE_MODE::AUTO_EXPOSURE_MODE_HIGH_QUALITY;

    camera->setPropertyExtension(PROPERTY_EXT_DEPTH_RANGE, value);
    camera->setPropertyExtension(PROPERTY_EXT_CONTRAST_MIN, value);
    camera->setProperty(STREAM_TYPE_DEPTH,PROPERTY_ENABLE_AUTO_EXPOSURE,1);
    camera->setPropertyExtension(PROPERTY_EXT_AUTO_EXPOSURE_MODE,value);
}

CAMERA_CODE SurfaceHD100::GetPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    cs::Pointcloud pc;
    cs::IFramePtr frameDepth;

    float scale = 0.1f;
    PropertyExtension value;

    if (SUCCESS != camera->getFrame(STREAM_TYPE_DEPTH, frameDepth))
    {
        ERROR_CODE ret = camera->disconnect();
        if(ret != SUCCESS)
            return CAMERA_STOP;
        return CAMERA_DEPTH_GETFRAME;
    }

    if (SUCCESS == camera->getPropertyExtension(PROPERTY_EXT_DEPTH_SCALE, value))
    {
        scale = value.depthScale;
    }
    else
    {
        ERROR_CODE ret = camera->disconnect();
        if(ret != SUCCESS)
            return CAMERA_STOP;
        return  CAMERA_DEPTH_SCALE;
    }

    depth = reinterpret_cast<unsigned short*>(const_cast<char*>(frameDepth->getData()));
    pc.generatePoints(depth, frameDepth->getWidth(), frameDepth->getHeight(), scale, &intrinsicParamDepth, nullptr, nullptr);

    const auto vertices = pc.getVertices();

    for (unsigned long i = 0; i < static_cast<unsigned long>(pc.size()); ++i)
    {
        if(!(IsEqual(vertices[i].x,0)&&IsEqual(vertices[i].y,0)&&IsEqual(vertices[i].z,0)))
        {
          pcl::PointXYZ point{vertices[i].x,vertices[i].y,vertices[i].z};
          cloud->push_back(point);
        }
    }
    return CAMERA_SUCCESS;
}

CAMERA_CODE SurfaceHD100::GetRgb(cv::Mat &img)
{
    cs::IFramePtr frameRgb;
    if (SUCCESS != camera->getFrame(STREAM_TYPE_RGB, frameRgb))
    {
        ERROR_CODE ret = camera->disconnect();
        if(ret != SUCCESS)
          return CAMERA_STOP;
        return CAMERA_RGB_GETFRAME;
    }
    cv::Mat image(frameRgb->getHeight(), frameRgb->getWidth(), CV_8UC3, static_cast<void*>(const_cast<char*>(frameRgb->getData())));
    img = image.clone();
    return CAMERA_SUCCESS;
}

CAMERA_CODE SurfaceHD100::Rgb2Point(int x, int y, pcl::PointXYZ point)
{
    int dx,dy;
    cs::float3 p;
    cs::float2 m;
    cs::Pointcloud pc;
    float scale = 0.1f;
    PropertyExtension value;
    cs::IFramePtr frameRgb;
    cs::IFramePtr frameDepth;
    std::map<int,int> rgbTodepth;

    if (SUCCESS != camera->getFrame(STREAM_TYPE_DEPTH, frameDepth))
        return CAMERA_DEPTH_GETFRAME;
    if (SUCCESS != camera->getFrame(STREAM_TYPE_RGB, frameRgb))
        return CAMERA_DEPTH_GETFRAME;

    if (SUCCESS == camera->getPropertyExtension(PROPERTY_EXT_DEPTH_SCALE, value))
    {
        scale = value.depthScale;
    }
    else
    {
        return  CAMERA_DEPTH_SCALE;
    }

    pc.generateTextureToDepthMap(frameDepth->getWidth(),frameDepth->getHeight(),rgbTodepth);

    pc.getDepthCoordFromMap(rgbTodepth,x,y,frameRgb->getWidth(),frameRgb->getHeight(),frameDepth->getWidth(),frameDepth->getHeight(),dx,dy);


    float z = static_cast<float>(*(depth +(dy*frameDepth->getWidth()+dx)));

    pc.generatePoint(p,m,dx,dy,z, frameDepth->getWidth(), frameDepth->getHeight(), scale, &intrinsicParamDepth, nullptr, nullptr);

    point.x=p.x;
    point.y=p.y;
    point.z=p.z;

    return CAMERA_SUCCESS;
}

CAMERA_CODE SurfaceHD100::StopDepth()
{
    ERROR_CODE ret = camera->stopStream(STREAM_TYPE_DEPTH);
    if(ret != SUCCESS)
        return CAMERA_STOP_DEPTH;
    return  CAMERA_SUCCESS;
}

CAMERA_CODE SurfaceHD100::stopRgb()
{
    ERROR_CODE ret = camera->stopStream(STREAM_TYPE_RGB);
    if(ret != SUCCESS)
        return CAMERA_STOP_RGB;
    return  CAMERA_SUCCESS;
}

CAMERA_CODE SurfaceHD100::StopCamera()
{
    ERROR_CODE ret = camera->disconnect();
    if(ret != SUCCESS)
        return CAMERA_STOP;
    return  CAMERA_SUCCESS;
}

bool SurfaceHD100::IsEqual(float a, float b)
{
   return (std::fabs(a-b)<1e-10f)?true:false;
}
