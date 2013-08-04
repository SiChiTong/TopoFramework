/**
 * @file SkillDribble.h
 *
 * Contains the skill module Dribble.
 *
 * @author Andreas Seekircher <aseek@cs.miami.edu>
 */
#ifndef SKILLDRIBBLE_H
#define SKILLDRIBBLE_H

#include "kernel/Template.h"

#include "representations/perception/BallPercept.h"
#include "representations/modeling/RobotPose.h"
#include "representations/modeling/OtherRobots.h"
#include "representations/modeling/BallPos.h"
#include "representations/skills/ActiveFoot.h"
#include "representations/skills/SkillRequest.h"
#include "representations/skills/SkillOutputs.h"


MODULE(SkillDribble)
  REQUIRES(RobotPose)
  REQUIRES(OtherRobots)
  REQUIRES(BallPercept)
  REQUIRES(BallPos)
  REQUIRES(LocalBallPos)
  REQUIRES(ActiveFoot)
  REQUIRES(SkillRequest)
  PROVIDES(SkillDribbleOutput)
END_MODULE

/**
 * @class SkillDribble
 */
class SkillDribble : public SkillDribbleBase
{
  public:
    
    void init();
    
    void update(SkillDribbleOutput& theSkillDribbleOutput);


  private:
  
    double ball_visibility;
    bool dribbling;
};


#endif

