#ifndef TEAMCOMRECV_H
#define TEAMCOMRECV_H

#include "kernel/Template.h"

#include "representations/rcss/FieldDimensions.h"
#include "representations/perception/FrameInfo.h"
#include "representations/perception/PlayerInfo.h"
#include "representations/modeling/RobotPose.h"
#include "representations/modeling/OtherRobots.h"
#include "representations/perception/HearMessage.h"
#include "representations/teamcom/TeamComData.h"
#include "representations/teamcom/TeamComInfo.h"

#include "MsgEncoder.h"

MODULE(TeamComRecv)
  REQUIRES(FieldDimensions)
  REQUIRES(FrameInfo)
  REQUIRES(PlayerInfo)
  REQUIRES(HearMessage)
  USES(RobotPose) //for position consistency check and drawing
  PROVIDES(TeamComInfo)
  PROVIDES(TeamComDataIn)
END_MODULE


class TeamComRecv : public TeamComRecvBase
{
  private:
    enum { MAX_PLAYERS = OtherRobotsData::MAX_UNUM };
    

    //message encoding/decoding
    MsgEncoder msgEncoder;
    bool processHeardMessage(const HearMessageData& msg);

    void draw(const Vector2<double> &pose, bool red, double direction);

    //TeamComInfo
    bool messageReceived;
    int senderUnum;
    Vector2<double> senderPos;
    double poseInconsistency;
    
    //received data
    std::vector<BitBuffer*> data;
    
  public:
  
    TeamComRecv();
    ~TeamComRecv();

    void init();
    void execute();
    
    void update(TeamComInfo& theTeamComInfo);
    void update(TeamComDataIn& theTeamComDataIn);
    
};


#endif

