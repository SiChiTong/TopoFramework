#ifndef SKILLCOMBINATOR_H
#define SKILLCOMBINATOR_H

#include "kernel/Template.h"

#include "representations/skills/SkillRequest.h"
#include "representations/skills/SkillOutputs.h"
#include "representations/motion/MotionRequest.h"


MODULE(SkillCombinator)
  REQUIRES(SkillRequest)
  REQUIRES(SkillMoveToPosOutput)
  REQUIRES(SkillGetBallOutput)
  REQUIRES(SkillKickOutput)
  REQUIRES(SkillDribbleOutput)
  PROVIDES(MotionRequest)
END_MODULE


class SkillCombinator : public SkillCombinatorBase
{
  public:
    
    void update(MotionRequest& theMotionRequest);
  
};


#endif

