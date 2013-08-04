#include "BehaviorTools.h"

#include "representations/modeling/RobotPose.h"
#include "representations/modeling/BallPos.h"
#include "representations/modeling/FallState.h"
#include "representations/behavior/BehaviorOutputs.h"

bool BehaviorTools::standUp(const FallState *theFallState, SkillRequest* skillRequest)
{

  MotionRequest* theMotionRequest = &(skillRequest->motionRequest);
  skillRequest->skill = SkillRequest::NONE;
  if (theFallState->orientation == ORIENTATION_LYING_BACK)
  {
    theMotionRequest->motion = MotionRequest::SPECIAL_ACTION;
    theMotionRequest->specialActionRequest = MotionRequest::STAND_UP_BACK;
  }
  else if (theFallState->orientation == ORIENTATION_LYING_FORWARD)
  {
    theMotionRequest->motion = MotionRequest::SPECIAL_ACTION;
    theMotionRequest->specialActionRequest = MotionRequest::STAND_UP_FRONT;
  }
  else if (theFallState->orientation == ORIENTATION_LYING_SIDEWAYS)
  {
    theMotionRequest->motion = MotionRequest::STAND;
  }
  else
    return false;
  return true;
}

bool BehaviorTools::searchBall(const BallPos *ballPos, const RobotPose *robotPose,
    SkillRequest* skillRequest)
{
  if (ballPos->confidence < 0.01)
  {
    Pose2D movetarget = robotPose->pose;
    skillRequest->skill = SkillRequest::MOVETOPOS;
    skillRequest->moveToRequest.setValues(movetarget.rotate(3.1), -1, true, 0.1);
    return true;
  }
  return false;
}
