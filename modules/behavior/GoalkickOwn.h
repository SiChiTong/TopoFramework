#ifndef GOALKICKOPP_H
#define GOALKICKOPP_H


#include "kernel/Template.h"

#include "BehaviorTools.h"

#include "representations/rcss/FieldDimensions.h"
#include "representations/behavior/PlayerRole.h"
#include "representations/perception/Gamestate.h"
#include "representations/modeling/RobotPose.h"
#include "representations/modeling/FallState.h"
#include "representations/behavior/BehaviorOutputs.h"


MODULE(GoalkickOwn)
  REQUIRES(FieldDimensions)
  REQUIRES(Gamestate)
  REQUIRES(PlayerRole)
  REQUIRES(RobotPose)
  REQUIRES(FallState)
  PROVIDES(GoalkickOwnOutput)
END_MODULE


class GoalkickOwn : public GoalkickOwnBase
{
  public:
  
    void update(GoalkickOwnOutput& output);
  
 private:
 
    BehaviorTools tools;
    
};


#endif

