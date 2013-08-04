/**
 * @file SkillGetBall.h
 *
 * Contains the skill module GetBall.
 *
 * @author Andreas Seekircher <aseek@cs.miami.edu>
 */
#ifndef SKILLGETBALL_H
#define SKILLGETBALL_H

#include "kernel/Template.h"

#include "representations/modeling/RobotPose.h"
#include "representations/modeling/BallPos.h"
#include "representations/skills/ActiveFoot.h"
#include "representations/skills/SkillRequest.h"
#include "representations/skills/SkillOutputs.h"


MODULE(SkillGetBall)
  REQUIRES(SkillRequest)
  REQUIRES(RobotPose)
  REQUIRES(BallPos)
  REQUIRES(ActiveFoot)
  PROVIDES(SkillGetBallOutput)
END_MODULE

/**
 * @class SkillGetBall
 */
class SkillGetBall : public SkillGetBallBase
{
  public:
    
    void update(SkillGetBallOutput& theSkillGetBallOutput);


  private:
  

};


#endif

