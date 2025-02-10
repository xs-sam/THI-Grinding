#include "kukatcp.h"

/**********************************************
* @projectName   KukaTcp
* @brief         KUKA机器人TCP通讯构造函数
* @param         ip: 机器人服务器的ip地址
*                port: 机器人服务器的端口
* @return        void
* @author        xs:1874020422@qq.com
* @date          2024-06-18
**********************************************/
KukaTcp::KukaTcp(std::string ip, std::string port):ip(ip),port(port)
{
  if(!ip.empty()&&!port.empty())
  {
    tcp=new xmlTcp(ip,port);
  }
}

/**********************************************
* @projectName   connectTcp
* @brief         连接机器人服务器
* @param         void
* @return        -1: ip或port为空
*                -2: 超过最大tcp连接数
*                -3: 超过最大机器人连接数
*                 0: 连接成功
* @author        xs:1874020422@qq.com
* @date          2024-06-17
**********************************************/
int KukaTcp::connectTcp()
{
  if(ip.empty()||port.empty())
  {
    ROS_ERROR("ip or port is empty !!!");
    return -1;
  }

  getParams();
  int connectNum=0;
  while (tcp->connectTcp()<0)
  {
    ROS_WARN("reconnection tcp with %s:%s",ip.c_str(),port.c_str());
    sleep(1);
    if(connectNum>tcpMaxConnect)
    {
      ROS_ERROR("connect %s:%s is failed!!!",ip.c_str(),port.c_str());
      return -2;
    }
    connectNum++;
    ros::spinOnce();
  }

  if(!tcp->isConnectRobot(robotConnectOkKey,"1"))
  {
    tcp->disConnectTcp();
    return -3;
  }
  tcp->getParams(curX,curY,curZ,curA,curB,curC);
  std::thread readData(&xmlTcp::readReceiveMap,tcp);
  readData.detach();

  ROS_INFO_STREAM("\033[32m connect robot succeed \033[0m");
  return 0;

}

/**********************************************
* @projectName   getParams
* @brief         获得参数表各个通讯变量参数
* @param         void
* @return        void
* @author        xs:1874020422@qq.com
* @date          2024-06-18
**********************************************/
void KukaTcp::getParams()
{
  ros::NodeHandle n;

  n.getParam("tcp/maxConnect",tcpMaxConnect);
  n.getParam("robot/maxConnect",robotMaxConnect);

  n.getParam("robot/receive/connectOk",robotConnectOkKey);
  n.getParam("robot/receive/startNewRound",startNewRound); 
  n.getParam("robot/receive/X",curX);
  n.getParam("robot/receive/Y",curY);
  n.getParam("robot/receive/Z",curZ);
  n.getParam("robot/receive/A",curA);
  n.getParam("robot/receive/B",curB);
  n.getParam("robot/receive/C",curC);

  n.getParam("robot/send/X",X);
  n.getParam("robot/send/Y",Y);
  n.getParam("robot/send/Z",Z);
  n.getParam("robot/send/A",A);
  n.getParam("robot/send/B",B);
  n.getParam("robot/send/C",C);
  n.getParam("robot/send/pointNum",pointNum);
  n.getParam("robot/send/moveMode",moveMode);
  n.getParam("robot/send/robotSpeed",robotSpeed);
  n.getParam("robot/send/sigStart",sigStart);
  n.getParam("robot/send/positionStart",positionStart);
}

/**********************************************
* @projectName   compareData
* @brief         阻塞读取key对应的值，并和data比较 相同返回true
* @param         key: 对应的信号键
*                data: 需要判断是否相等的值
* @return        读取到值并和data相等返回true，超过最大读取次数还未读到返回false(开始发送信号会一直等待)
* @author        xs:1874020422@qq.com
* @date          2024-06-18
**********************************************/
bool KukaTcp::compareData(std::string key,std::string data)
{
    bool ret=tcp->isDataWithSignal(key,data);
    if(key==startNewRound)
    {
      while (!ret)
      {
        ret=tcp->isDataWithSignal(key,data);
      }
    }
  return ret;
}

/*********************************************
* @projectName   sendTrack
* @brief         服务呼叫的传输路径函数，输入一段子路径发送给机器人
* @param         req: 自定义服务结构体，一段子路径(包含了点数、速度、移动方式和路径点)
* @return        发完返回0，失败返回-1
* @author        xs:1874020422@qq.com
* @date          2024-06-18
**********************************************/
int KukaTcp::sendTrack(kuka_tcp::kukaTrackRequest &req)
{
    if(!compareData(startNewRound,"1"))
        return -1;
    tcp->writeXmlTcp(pointNum,std::to_string(req.track.size()));
    tcp->writeXmlTcp(robotSpeed,std::to_string(req.speed));
    tcp->writeXmlTcp(moveMode,std::to_string(req.mod));
    tcp->writeXmlTcp(sigStart,"true");
    for(unsigned long i=0;i<req.track.size();i++)
    {
      tcp->writeXmlTcp(X+std::to_string(i),std::to_string(req.track[i].x));
      tcp->writeXmlTcp(Y+std::to_string(i),std::to_string(req.track[i].y));
      tcp->writeXmlTcp(Z+std::to_string(i),std::to_string(req.track[i].z));
      tcp->writeXmlTcp(A+std::to_string(i),std::to_string(req.track[i].a));
      tcp->writeXmlTcp(B+std::to_string(i),std::to_string(req.track[i].b));
      tcp->writeXmlTcp(C+std::to_string(i),std::to_string(req.track[i].c));
    }
    tcp->writeXmlTcp(positionStart,"true");
    return 0;
}

/**********************************************
* @projectName   getPose
* @brief         供主函数调用接口，得到机器人的实际位置
* @param         pos: (输出)得到一个6位vector包含(x、y、z、a、b、c)
* @return        有数据成功读到返回0，否则返回-1(数据位置就全为9999.9)
* @author        xs:1874020422@qq.com
* @date          2024-06-21
**********************************************/
int KukaTcp::getPose(std::vector<float> &pos)
{
    return tcp->getPose(pos);
}

/**********************************************
* @projectName   disconnect
* @brief         断开tcp连接，清空标识符
* @return        成功返回0
* @author        xs(1874020422@qq.com)
* @date          2024-11-23
**********************************************/
int KukaTcp::disconnect()
{
 return tcp->disConnectTcp();
}

