#ifndef FREEKICKOPP_H
#define FREEKICKOPP_H


#include "kernel/Template.h"

#include "BehaviorTools.h"

#include "representations/rcss/FieldDimensions.h"
#include "representations/perception/Gamestate.h"
#include "representations/behavior/PlayerRole.h"
#include "representations/modeling/BallPos.h"
#include "representations/modeling/FallState.h"
#include "representations/behavior/BehaviorOutputs.h"


MODULE(FreekickOpp)
  REQUIRES(FieldDimensions)
  REQUIRES(Gamestate)
  REQUIRES(PlayerRole)
  REQUIRES(BallPos)
  REQUIRES(FallState)
  PROVIDES(FreekickOppOutput)
END_MODULE


class FreekickOpp : public FreekickOppBase
{
  public:
  
    void update(FreekickOppOutput& output);
  
 private:
 
    BehaviorTools tools;
     
};


#endif

