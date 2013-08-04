#ifndef GOALKICKOPP_H
#define GOALKICKOPP_H


#include "kernel/Template.h"

#include "BehaviorTools.h"

#include "representations/rcss/FieldDimensions.h"
#include "representations/perception/PlayerInfo.h"
#include "representations/behavior/PlayerRole.h"
#include "representations/perception/Gamestate.h"
#include "representations/modeling/RobotPose.h"
#include "representations/modeling/FallState.h"
#include "representations/behavior/BehaviorOutputs.h"


MODULE(GoalkickOpp)
  REQUIRES(FieldDimensions)
  REQUIRES(Gamestate)
  REQUIRES(PlayerInfo)
  REQUIRES(PlayerRole)
  REQUIRES(RobotPose)
  REQUIRES(FallState)
  PROVIDES(GoalkickOppOutput)
END_MODULE


class GoalkickOpp : public GoalkickOppBase
{
  public:
  
    void update(GoalkickOppOutput& output);
  
 private:
 
    BehaviorTools tools;
    
};


#endif

