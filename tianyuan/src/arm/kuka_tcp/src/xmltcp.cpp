#include "xmltcp.h"

/**********************************************
* @projectName   xmlTcp
* @brief         对象构造函数，传入一个ip地址和端口号
* @param         ip: 服务器的ip地址
*                port: 服务器的端口号
* @return        void
* @author        xs:1874020422@qq.com
* @date          2024-06-13
**********************************************/
xmlTcp::xmlTcp(std::string ip, std::string port)
{
    socketFd=-1;
    bzero(&servAddr,sizeof (servAddr));

    servAddr.sin_family=AF_INET;
    servAddr.sin_addr.s_addr=inet_addr(ip.c_str());
    servAddr.sin_port=htons(static_cast<uint16_t>(atoi(port.c_str())));
}

/**********************************************
* @projectName   connectTcp
* @brief         初始化socket并连接tcp通讯
* @param         void
* @return        -1: socket初始化失败
*                -2: tcp连接失败
* @author        xs:1874020422@qq.com
* @date          2024-06-13
**********************************************/
int xmlTcp::connectTcp()
{
    if((socketFd=socket(AF_INET,SOCK_STREAM,0))<0)
    {
        return -1;
    }
    if(connect(socketFd,reinterpret_cast<struct sockaddr*>(&servAddr),sizeof (servAddr))<0)
    {
        close(socketFd);
        return -2;
    }

    // 设置套接字为非阻塞模式
    if (setNonBlockingMode(socketFd) < 0)
    {
        close(socketFd);
        return -1;
    }
    return 0;
}

/**********************************************
* @projectName   disConnectTcp
* @brief         断开tcp连接
* @param         void
* @return        成功返回0，失败返回-1
* @author        xs:1874020422@qq.com
* @date          2024-06-15
**********************************************/
int xmlTcp::disConnectTcp()
{
    ROS_WARN("close TCP Connect");
    return close(socketFd);
}

/**********************************************
* @projectName   isConnectRobot
* @brief         是否连接上机器人
* @param         key: 机器人连接信号
*                data: 该信号对应的值
*                timeOut: 重连接次数
* @return        连接成功返回true，失败返回false
* @author        xs:1874020422@qq.com
* @date          2024-06-21
**********************************************/
bool xmlTcp::isConnectRobot(std::string key, std::string data, int timeOut)
{
    char* ip=inet_ntoa(servAddr.sin_addr);
    unsigned short port = ntohs(servAddr.sin_port);
    std::string portStr = std::to_string(port);
    for (int j =0;j<timeOut;j++)
    {
        std::string xml=readXmlTcp();
//        std::cout<<xml<<std::endl;
        if(!xml.empty())
        {
            std::map<std::string,std::string> map;
            tinyxml2::XMLDocument doc;
            doc.Parse(xml.c_str());
            tinyxml2::XMLElement* root=doc.FirstChildElement();
            map=getReceiveMap(root);
            mapReceiveXml.clear();
            for (std::map<std::string,std::string>::iterator it=map.begin();it!=map.end();++it)
            {
                if(it->first==key&&it->second==data)
                {
                    return true;
                }
            }

        }
        else
        {
            ROS_WARN("read is empty,reconnection robot...");
            disConnectTcp();
        }
        sleep(1);

        for(int j=0;j<10;j++)
        {
            if (connectTcp()<0)
            {
                ROS_WARN("reconnection tcp with %s:%s",ip,portStr.c_str());
                sleep(1);
            }
            else
            {
                break;
            }
        }

    }
    ROS_ERROR("connect %s:%s is failed!!!",ip,portStr.c_str());
    return false;
}

/**********************************************
* @projectName   getParams
* @brief         得到x、y、z、a、b、c的key，便于分离读到的数据
* @param         keyX: 当前位置X对应的key
*                keyY: 当前位置y对应的key
*                keyZ: 当前位置z对应的key
*                keyA: 当前位置a对应的key
*                keyB: 当前位置b对应的key
*                keyC: 当前位置c对应的key
* @return        void
* @author        xs:1874020422@qq.com
* @date          2024-06-21
**********************************************/
void xmlTcp::getParams(const std::string &keyX, const std::string &keyY, const std::string &keyZ, const std::string &keyA, const std::string &keyB, const std::string &keyC)
{
    curX=keyX;
    curY=keyY;
    curZ=keyZ;
    curA=keyA;
    curB=keyB;
    curC=keyC;
}

/**********************************************
* @projectName   writeXmlTcp
* @brief         给某一个key写对应的值data
* @param         key: tcp通讯对应的键
*                data: tcp写下去的值
* @return        如果key或者data为空返回-1，写入失败返回-2，成功返回0
* @author        xs:1874020422@qq.com
* @date          2024-06-20
**********************************************/
int xmlTcp::writeXmlTcp(std::string key, std::string data)
{
    std::string token;
    std::vector<std::string> vectorKey;
    std::istringstream tokenStream(key);
    if(key.empty()||data.empty())
    {
        std::cout<<"xml key or data is empty"<<std::endl;
        return -1;
    }


    while (std::getline(tokenStream, token, '/'))
    {
        vectorKey.push_back(token);
    }

    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement* root;
    tinyxml2::XMLElement* child;
    for(unsigned long i=0;i<vectorKey.size();++i)
    {
        child=doc.NewElement(vectorKey[i].c_str());
        if(i==0)
        {
            root=child;
            doc.InsertEndChild(root);
        }
        else if(i==vectorKey.size()-1)
        {
            child->SetText(data.c_str());
            root->InsertEndChild(child);
        }
        else
        {
            root->InsertEndChild(child);
            root=child;
        }
    }

    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    std::string xml(removeExtraString(printer.CStr()));
    if(0>send(socketFd,xml.c_str(),xml.size(),0))
        return -2;
    return 0;
}

/**********************************************
* @projectName   readReceiveMap
* @brief         多线程调用该函数，一直读取缓冲区得到两个map(信号和当前位置)
* @param         void
* @return        void
* @author        xs:1874020422@qq.com
* @date          2024-06-14
**********************************************/
void xmlTcp::readReceiveMap()

{
    while(ros::ok())
    {
        std::string xml=readXmlTcp();
        if(!xml.empty())
        {
            std::map<std::string,std::string> map;
            tinyxml2::XMLDocument doc;
            doc.Parse(xml.c_str());
            tinyxml2::XMLElement* root=doc.FirstChildElement();
            map=getReceiveMap(root);
            mapReceiveXml.clear();
            for (std::map<std::string,std::string>::iterator it=map.begin();it!=map.end();++it)
            {
                if(it->first==curX||it->first==curY||it->first==curZ||it->first==curA||it->first==curB||it->first==curC)
                {
                    mapPos[it->first]=it->second;
                }
                else
                {
                    std::map<std::string,std::string> signal;
                    signal[it->first]=it->second;
                    mapSignal.clear();
                    mapSignal=signal;
                }
            }
        }
        ros::spinOnce();
    }
}

/**********************************************
* @projectName   isDataWithSignal
* @brief         判断mapSignal的键值是否为key和data
* @param         key: 需要判断的key
*                data: 需要判断的data
*                timeOut: 阻塞判断的超时数，一般为秒
* @return        存在key且data匹配返回true，超时返回false
* @author        xs:1874020422@qq.com
* @date          2024-06-20
**********************************************/
bool xmlTcp::isDataWithSignal(std::string key, std::string data, int timeOut)
{
    for (int i=0;i<timeOut;i++)
    {
        for (std::map<std::string,std::string>::iterator it=mapSignal.begin();it!=mapSignal.end();++it)
        {
            if(it->first==key&&it->second==data)
            {
                mapSignal[key].clear();
                return true;
            }
        }
        sleep(1);
    }
    return false;
}

/**********************************************
* @projectName   getPose
* @brief         得到机器人的实际位置
* @param         pos: (输出)得到一个6位vector包含(x、y、z、a、b、c)
* @return        有数据成功读到返回0，否则返回-1(数据位置就全为9999.9)
* @author        xs:1874020422@qq.com
* @date          2024-06-21
**********************************************/
int  xmlTcp::getPose(std::vector<float> &pos)
{
    if(mapPos.size()!=0)
    {
        for (std::map<std::string,std::string>::iterator it=mapPos.begin();it!=mapPos.end();++it)
        {
            if(it->first==curX)
            {
                pos.at(0)=std::stof(it->second);
            }
            else if(it->first==curY)
            {
                pos.at(1)=std::stof(it->second);
            }
            else if(it->first==curZ)
            {
                pos.at(2)=std::stof(it->second);
            }
            else if(it->first==curA)
            {
                pos.at(3)=std::stof(it->second);
            }
            else if(it->first==curB)
            {
                pos.at(4)=std::stof(it->second);
            }
            else if(it->first==curC)
            {
                pos.at(5)=std::stof(it->second);
            }
            else
            {
                for (unsigned long i=0;i<6;i++)
                {
                    pos.at(i)=static_cast<float>(9999.9);
                }
                return -1;
            }
        }
    }
    else
    {
        for (unsigned long i=0;i<6;i++)
        {
            pos.at(i)=static_cast<float>(9999.9);
        }
        return -1;
    }
    return 0;
}


/**********************************************
  私有函数段:
**********************************************/

/**********************************************
* @projectName   readXmlTcp
* @brief         从tcp通讯中读取当前缓冲区数据
* @param         void
* @return        读到的xml数据
* @author        xs:1874020422@qq.com
* @date          2024-06-14
**********************************************/
std::string xmlTcp::readXmlTcp()
{
    FD_ZERO(&read_fds);
    FD_SET(socketFd, &read_fds);

    timeout.tv_sec = 3;
    timeout.tv_usec = 0;

    int selret = select(socketFd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (selret <=0)
    {
        return "";
    }
    else
    {
        char buff[XMLLEN]={0};
        if(0>recv(socketFd,buff,XMLLEN,0))
        {
            return "";
        }

        std::string xml(buff);
        return xml;
    }
}

/**********************************************
* @projectName   setNonBlockingMode
* @brief         设置某tcp的sockFd为非阻塞读取模式
* @param         sock: tcp创建的sockFd号
* @return        失败返回-1，成功返回0
* @author        xs:1874020422@qq.com
* @date          2024-06-20
**********************************************/
int xmlTcp::setNonBlockingMode(int sock)
{
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags == -1)
    {
        return -1;
    }
    return fcntl(sock, F_SETFL, flags | O_NONBLOCK);
}

/**********************************************
* @projectName   getReceiveMap
* @brief         私有重载函数，递归获得map
* @param         root: tinyxml2分得的树的根节点
*                path: 传入上一层级的路径
* @return        一张xml信息map映射表
* @author        xs:1874020422@qq.com
* @date          2024-06-14
**********************************************/
std::map<std::string,std::string> xmlTcp::getReceiveMap(tinyxml2::XMLElement* root,std::string path)
{
    for(tinyxml2::XMLElement* r=root;r!=nullptr;r=r->NextSiblingElement())
    {
        std::string p=path;

        if(r->FirstChildElement()!=nullptr)
        {
            p=p+r->Value()+"/";
            getReceiveMap(r->FirstChildElement(),p);
        }
        else
        {
            p=p+r->Value();
            mapReceiveXml[p]=r->GetText();
        }
    }
    return  mapReceiveXml;
}

/**********************************************
* @projectName   removeExtraString
* @brief         移除字符串中多余的字符
* @param         str: 待移除的字符串
* @return        移除后的字符串
* @author        xs:1874020422@qq.com
* @date          2024-06-14
**********************************************/
std::string xmlTcp::removeExtraString(std::string str)
{
    std::string res;
    for(char c:str)
    {
        if(c!=10&&c!=32)
            res+=c;
    }
    return  res;
}

