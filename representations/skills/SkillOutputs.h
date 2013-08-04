#ifndef SKILLOUTPUTS_H
#define SKILLOUTPUTS_H

#include "kernel/Template.h"

#include "representations/skills/MoveToRequest.h"
#include "representations/motion/MotionRequest.h"

class SkillOutput
{
  public:
    MoveToRequest moveToRequest;
    MotionRequest motionRequest;
    bool active;
};


REPRESENTATION(SkillMoveToPosOutput)
class SkillMoveToPosOutput 
    : public SkillMoveToPosOutputBase, public SkillOutput {};

REPRESENTATION(SkillGetBallOutput)
class SkillGetBallOutput   
    : public SkillGetBallOutputBase,   public SkillOutput {};

REPRESENTATION(SkillKickOutput)
class SkillKickOutput      
    : public SkillKickOutputBase,      public SkillOutput {};

REPRESENTATION(SkillDribbleOutput)
class SkillDribbleOutput   
    : public SkillDribbleOutputBase,   public SkillOutput {};

#endif

