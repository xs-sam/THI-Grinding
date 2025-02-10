#ifndef XMLTCP_H
#define XMLTCP_H

#define XMLLEN 1024

#include <map>
#include <vector>
#include <sstream>
#include <string>
#include <algorithm>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include "tinyxml2.h"
#include <fcntl.h>
#include <ros/ros.h>

class xmlTcp
{
public:
    xmlTcp(std::string ip,std::string port);

    int connectTcp();
    int disConnectTcp();
    bool isConnectRobot(std::string key,std::string data,int timeOut=10);

    void getParams(const std::string &keyX,const std::string &keyY,const std::string &keyZ,
                   const std::string &keyA,const std::string &keyB,const std::string &keyC);
    int writeXmlTcp(std::string key,std::string data);
    void readReceiveMap();
    bool isDataWithSignal(std::string key,std::string data,int timeOut=10);
    int  getPose(std::vector<float> &pos);

private:
    std::string readXmlTcp();
    int setNonBlockingMode(int sock);
    std::map<std::string,std::string> getReceiveMap(tinyxml2::XMLElement* root,std::string path="");
    std::string removeExtraString(std::string str);

    int socketFd;
    struct sockaddr_in servAddr;
    fd_set read_fds;
    struct timeval timeout;
    std::map<std::string,std::string> mapReceiveXml;
    std::map<std::string,std::string> mapPos;
    std::map<std::string,std::string> mapSignal;
    std::string curX,curY,curZ,curA,curB,curC;;

};

#endif // XMLTCP_H
