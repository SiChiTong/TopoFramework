#ifndef TEAMCOMSEND_H
#define TEAMCOMSEND_H

#include "kernel/Template.h"

#include "representations/rcss/FieldDimensions.h"
#include "representations/perception/FrameInfo.h"
#include "representations/perception/PlayerInfo.h"
#include "representations/modeling/RobotPose.h"
#include "representations/modeling/OtherRobots.h"
#include "representations/teamcom/TeamComInfo.h"
#include "representations/teamcom/TeamComData.h"
#include "representations/rcss/SayMessage.h"

#include "MsgEncoder.h"

MODULE(TeamComSend)

  REQUIRES(FieldDimensions)
  REQUIRES(FrameInfo)
  REQUIRES(PlayerInfo)
  REQUIRES(RobotPose)
  REQUIRES(TeamComInfo)
  REQUIRES(TeamComDataOut)

  PROVIDES(SayMessage)
  
END_MODULE



class TeamComSend : public TeamComSendBase
{
  private:
    enum { MAX_PLAYERS = OtherRobotsData::MAX_UNUM };
  
    //send order
    long lastReceiveTime[MAX_PLAYERS+1];
    int lastReceivedId;
    int prevId;
    long timeout;
    void updateSendOrder(int id);
    
    //message encoding/decoding
    MsgEncoder msgEncoder;
    void createOutputMessage();      
    
  public:
  
    TeamComSend();
    ~TeamComSend();

    void init();

    void execute();
    void update(SayMessage& theSayMessage);
    
};


#endif

