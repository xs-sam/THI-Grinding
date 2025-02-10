#include <kukatcp.h>

static KukaTcp *tcp;
static int chunkPointNum;
static ros::Publisher pubPos;

bool track_arm(kuka_tcp::kukaTrackRequest &req,kuka_tcp::kukaTrackResponse &res)
{
  if(!(static_cast<int>(req.track.size())<chunkPointNum))
  {
    int chunkNum=static_cast<int>(req.track.size()/static_cast<unsigned long>(chunkPointNum));
    int remainder=static_cast<int>(req.track.size())%chunkPointNum;
    for (int i=0;i<chunkNum+1;i++)
    {
      kuka_tcp::kukaTrackRequest reqSon;
      reqSon.speed=req.speed;
      reqSon.mod=req.mod;
      reqSon.track.clear();
      if(i!=chunkNum)
      {
        for (int j=0;j<chunkPointNum;j++)
        {
          reqSon.track.push_back(req.track.at(static_cast<unsigned long>(j+i*chunkPointNum)));
        }
      }
      else
      {
        for (int j=0;j<remainder;j++)
        {
          reqSon.track.push_back(req.track.at(static_cast<unsigned long>(j+i*chunkPointNum)));
        }
      }
      if(reqSon.track.size()!=0)
      {
        if(tcp->sendTrack(reqSon)<0)
        {
            res.ok=false;
            return true;
        }
      }
    }
  }
  else
  {
    if(req.track.size()!=0)
    {
      if(tcp->sendTrack(req)<0)
      {
          res.ok=false;
          return true;
      }
    }
  }
    ROS_INFO_STREAM("\033[32m send track succeed \033[0m");
    res.ok=true;
    return true;
}

void getpos()
{
  ros::Rate loop(10);
  while (ros::ok())
  {
    kuka_tcp::kukaPoint msgPos;
    std::vector<float> pos(6);
    tcp->getPose(pos);
    msgPos.x=pos[0];
    msgPos.y=pos[1];
    msgPos.z=pos[2];
    msgPos.a=pos[3];
    msgPos.b=pos[4];
    msgPos.c=pos[5];

    pubPos.publish(msgPos);

    loop.sleep();
    ros::spinOnce();
  }
}

int main(int argv,char** argc)
{
    ros::init(argv,argc,"kuka_tcp");
    ros::NodeHandle n;
    std::string ip;
    int port;
    n.getParam("tcp/ip",ip);
    n.getParam("tcp/port",port);
    n.getParam("robot/chunkPointNum",chunkPointNum);


    tcp = new KukaTcp (ip,std::to_string(port));
    if(tcp->connectTcp()<0)
        return -1;

    pubPos=n.advertise<kuka_tcp::kukaPoint>("/kuka/link6/pose",1);
    ros::ServiceServer serSend=n.advertiseService("/kuka/track",track_arm);
    std::thread getPos(getpos);
    getPos.detach();
    ros::spin();
    return tcp->disconnect();

}
