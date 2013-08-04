/**
 * @file SkillMoveToPos.h
 *
 * Contains the skill module MoveToPos
 *
 * @author Andreas Seekircher <aseek@cs.miami.edu>
 */
#ifndef SKILLMOVETOPOS_H
#define SKILLMOVETOPOS_H

#include "kernel/Template.h"

#include "representations/perception/ForceData.h"
#include "representations/modeling/RobotPose.h"
#include "representations/modeling/BallPos.h"
#include "representations/skills/SafeWalkDirection.h"
#include "representations/skills/SkillRequest.h"
#include "representations/skills/SkillOutputs.h"
#include "representations/motion/WalkingEngineOutput.h"

MODULE(SkillMoveToPos)
  REQUIRES(RobotPose)
  REQUIRES(ForceData)
  REQUIRES(SafeWalkDirection)
  USES(WalkingEngineOutput)
  PROVIDES(SkillMoveToPosOutput)
END_MODULE

/**
 * @class SkillMoveToPos
 */
class SkillMoveToPos : public SkillMoveToPosBase
{
  public:
    
    void init();
    
    void update(SkillMoveToPosOutput& theSkillMoveToPosOutput);


  private:

    bool destination_turn;
    bool walk;
    bool walk_stopped;

    bool backwards;
    
    //from config
    bool keepForwardOrientation;

};


#endif

