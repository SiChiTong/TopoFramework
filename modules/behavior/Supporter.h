#ifndef SUPPORTER_H
#define SUPPORTER_H


#include "kernel/Template.h"

#include "BehaviorTools.h"

#include "representations/modeling/FallState.h"
#include "representations/behavior/PlayerRole.h"
#include "representations/behavior/BehaviorOutputs.h"


MODULE(Supporter)
  REQUIRES(FallState)
  REQUIRES(PlayerRole)
  PROVIDES(SupporterOutput)
END_MODULE


class Supporter : public SupporterBase
{
  public:
  
    void update(SupporterOutput& output);
  
  private:
  
    BehaviorTools tools;
 
};


#endif

