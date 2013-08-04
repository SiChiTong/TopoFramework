#ifndef BEFOREKICKOFF_H
#define BEFOREKICKOFF_H


#include "kernel/Template.h"

#include "representations/modeling/FallState.h"
#include "representations/rcss/FieldDimensions.h"
#include "representations/perception/PlayerInfo.h"
#include "representations/perception/Gamestate.h"
#include "representations/modeling/RobotPose.h"
#include "representations/behavior/BehaviorOutputs.h"

#include "BehaviorTools.h"

MODULE(BeforeKickOff)
  REQUIRES(FallState)
  REQUIRES(FieldDimensions)
  REQUIRES(Gamestate)
  REQUIRES(PlayerInfo)
  REQUIRES(RobotPose)
  PROVIDES(BeforeKickOffOutput)
END_MODULE


class BeforeKickOff : public BeforeKickOffBase
{
  public:
  
    void update(BeforeKickOffOutput& output);
  
 private:
 
    BehaviorTools tools;
    
};


#endif

