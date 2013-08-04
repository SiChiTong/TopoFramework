#ifndef BEHAVIORCOMBINATOR_H
#define BEHAVIORCOMBINATOR_H

using namespace std;

#include "kernel/Template.h"

#include "representations/behavior/BehaviorOutputs.h"


MODULE(BehaviorCombinator)
  REQUIRES(BeforeKickOffOutput)
  REQUIRES(KickOffOwnOutput)
  REQUIRES(FreekickOppOutput)
  REQUIRES(GoalkickOppOutput)
  REQUIRES(GoalkickOwnOutput)
  REQUIRES(GoalieOutput)
  REQUIRES(StrikerOutput)
  REQUIRES(SupporterOutput)
  PROVIDES(SkillRequest)
  PROVIDES(BeamRequest)
END_MODULE


class BehaviorCombinator : public BehaviorCombinatorBase
{
  public:
  
    BehaviorCombinator() : output(NULL) {}

    void execute();
    void update(SkillRequest& theSkillRequest);
    void update(BeamRequest& theBeamRequest);
  
  private:
  
    const BehaviorOutput *output;
  
};


#endif

