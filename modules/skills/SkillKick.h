/**
 * @file SkillKick.h
 *
 * Contains the skill module GetBall.
 *
 * @author Andreas Seekircher <aseek@cs.miami.edu>
 */
#ifndef SKILLKICK_H
#define SKILLKICK_H

#include "kernel/Template.h"

#include "representations/skills/SkillRequest.h"
#include "representations/perception/FrameInfo.h"
#include "representations/perception/BallPercept.h"
#include "representations/modeling/BallPos.h"
#include "representations/modeling/RobotPose.h"
#include "representations/skills/ActiveFoot.h"
#include "representations/skills/SkillKickParameters.h"
#include "representations/skills/SkillOutputs.h"


MODULE(SkillKick)
  REQUIRES(SkillRequest)
  REQUIRES(FrameInfo)
  REQUIRES(BallPercept)
  REQUIRES(BallPos)
  REQUIRES(LocalBallPos)
  REQUIRES(RobotPose)
  REQUIRES(ActiveFoot)
  REQUIRES(SkillKickParameters)
  PROVIDES(SkillKickOutput)
END_MODULE

/**
 * @class SkillKick
 */
class SkillKick : public SkillKickBase
{
  public:

    
    void init();
    void update(SkillKickOutput& theSkillKickOutput);


  private:

    MotionRequest::Motions kickType;
    const SkillKickParameters::Params *params;
  
    long time_kick_started;
    
    void selectKick(const Vector2<double> &target_point, double maxAngleError);
    bool kickPossible(const Vector2<double> &target, double maxAngleError);
    Pose2D positioning(const Vector2<double> &target_point,
                       double maxAngleError);

    int step_left_frames;
    int step_right_frames;
    
    double ball_visibility;

};


#endif

