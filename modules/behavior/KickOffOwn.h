#ifndef KICKOFFOWN_H
#define KICKOFFOWN_H


#include "kernel/Template.h"

#include "BehaviorTools.h"

#include "representations/modeling/RobotPose.h"
#include "representations/modeling/FallState.h"
#include "representations/behavior/PlayerRole.h"
#include "representations/perception/Gamestate.h"
#include "representations/behavior/BehaviorOutputs.h"


MODULE(KickOffOwn)
  REQUIRES(RobotPose)
  REQUIRES(FallState)
  REQUIRES(PlayerRole)
  REQUIRES(Gamestate)
  PROVIDES(KickOffOwnOutput)
END_MODULE


class KickOffOwn : public KickOffOwnBase
{
  public:
  
    void update(KickOffOwnOutput& output);
  
  private:
  
    BehaviorTools tools;
 
};


#endif

