#include "SkillCombinator.h"
#include <cstdio>

MAKE_MODULE(SkillCombinator)

void SkillCombinator::update(MotionRequest& theMotionRequest)
{
  //No skill, direct motion request  
  if (theSkillRequest->skill == SkillRequest::NONE)
  {
    //log << "skill NONE" << std::endl;
    theMotionRequest.updateMotionRequest(theSkillRequest->motionRequest);
  }
  //MoveToPos
  else if (theSkillMoveToPosOutput->active)  // MoveToPos can be used by other skills
  {
    //log << "skill MOVETOPOS" << std::endl;
    theMotionRequest.updateMotionRequest(theSkillMoveToPosOutput->motionRequest);
  }
  //GetBall
  else if (theSkillRequest->skill == SkillRequest::GETBALL && theSkillGetBallOutput->active)
  {
    //log << "skill GETBALL" << std::endl;
    theMotionRequest.updateMotionRequest(theSkillGetBallOutput->motionRequest);
  }
  //Kick
  else if (theSkillRequest->skill == SkillRequest::KICK && theSkillKickOutput->active)
  {
    //log << "skill KICK" << std::endl;
    theMotionRequest.updateMotionRequest(theSkillKickOutput->motionRequest);
  }
  //Dribble
  else if (theSkillRequest->skill == SkillRequest::DRIBBLE && theSkillDribbleOutput->active)
  {
    //log << "skill DRIBBLE" << std::endl;
    theMotionRequest.updateMotionRequest(theSkillDribbleOutput->motionRequest);
  }
  //error
  else
  {
    //log << "Error: No skill running and SkillRequest not NONE!" << std::endl;
  }
  //theMotionRequest->motion = MotionRequest::STAND;

  //HACK set walk mode and copy speed Pose2D into complexWalkRequest
  theMotionRequest.complexWalkRequest.mode = WalkRequest::speedMode;
  theMotionRequest.complexWalkRequest.speed = theMotionRequest.walkRequest;
}

