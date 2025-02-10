#ifndef KUKATCP_H
#define KUKATCP_H

#include <xmltcp.h>
#include <thread>
#include <kuka_tcp/kukaTrack.h>



class KukaTcp
{
public:
    KukaTcp(std::string ip="",std::string port="");
    int connectTcp();
    int sendTrack(kuka_tcp::kukaTrackRequest &req);
    int getPose(std::vector<float> &pos);
    int disconnect();

private:
    void getParams();
    bool compareData(std::string key,std::string data);
    xmlTcp *tcp;
    std::string ip;
    std::string port;
    int tcpMaxConnect,robotMaxConnect;

    //receive
    std::string robotConnectOkKey,startNewRound,curX,curY,curZ,curA,curB,curC;
    //send
    std::string pointNum,robotSpeed,moveMode,sigStart,positionStart,X,Y,Z,A,B,C;

};

#endif // KUKATCP_H
