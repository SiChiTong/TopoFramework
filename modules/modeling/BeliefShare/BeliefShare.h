#ifndef BELIEFSHARE_H
#define BELIEFSHARE_H

#include "kernel/Template.h"

#include "representations/rcss/FieldDimensions.h"
#include "representations/perception/FrameInfo.h"
#include "representations/perception/PlayerInfo.h"
#include "representations/perception/Gamestate.h"
#include "representations/modeling/RobotPose.h"
#include "representations/modeling/FallState.h"
#include "representations/modeling/BallPos.h"
#include "representations/modeling/OtherRobots.h"
#include "representations/teamcom/TeamComInfo.h"
#include "representations/teamcom/TeamComData.h"

#include <vector>

MODULE(BeliefShare)
  REQUIRES(FieldDimensions)
  REQUIRES(FrameInfo)
  REQUIRES(PlayerInfo)
  REQUIRES(Gamestate)
  REQUIRES(FallState)
  REQUIRES(LocalRobotPose)
  REQUIRES(LocalBallPos)
  REQUIRES(LocalOtherRobots)
  REQUIRES(TeamComInfo)
  REQUIRES(TC_BeliefShareIn)

  PROVIDES(TC_BeliefShareOut)
  PROVIDES(TeamRobotPose)
  PROVIDES(TeamBallPos)
  PROVIDES(TeamOtherRobots)  
  PROVIDES(GoalieBallPos)
END_MODULE



class BeliefShare : public BeliefShareBase
{
  private:
    enum { MAX_PLAYERS = OtherRobotsData::MAX_UNUM,
           MAX_OTHER_ROBOT_STATES = 3 };
      
    //global model
    RobotPoseData teamRobotPose;
    BallPosData teamBallPos;
    OtherRobotsData::State teamTeammates[MAX_PLAYERS+1];
    OtherRobotsData::State teamOpponents[MAX_PLAYERS+1];
    OtherRobotsData::State ownState;
    int goalieUnum;
    BallPosData goalieBallPos;
    int goalieBallPosTimeReceived;
    
    //send buffers
    bool buffersInitialized;
    BitBuffer* ballBuffer;
    BitBuffer* otherRobotBuffer[MAX_OTHER_ROBOT_STATES];

    //create BallPosData to send
    void createBallPos(BallPosData &ballpos);
    //select robot states with highest confidence difference
    OtherRobotsData::State emptyState;
    bool getStateToSend(const OtherRobotsData::State* &state, double &prevGain);

    //processing of received information
    void processReceivedBall(const BallPosData &pos);
    void processReceivedRobotState(bool ownTeam, 
                                   const OtherRobotsData::State &state);
    
    int bitsPoseT;
    int bitsPoseR;
    
  public:
  
    BeliefShare();
    ~BeliefShare();

    void init();

    void execute();
    void update(TC_BeliefShareOut& theTC_BeliefShareOut);

    void update(TeamRobotPose& theTeamRobotPose);
    void update(TeamBallPos& theTeamBallPos);
    void update(GoalieBallPos& theTeamBallPos);
    void update(TeamOtherRobots& theTeamOtherRobots);
    
};


#endif

