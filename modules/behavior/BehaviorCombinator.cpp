#include "BehaviorCombinator.h"

MAKE_MODULE(BehaviorCombinator)

void BehaviorCombinator::execute()
{
  if (theBeforeKickOffOutput->active)
  {
    output = theBeforeKickOffOutput;
    //log << "active behavior: BeforeKickOff" << std::endl;
  }
  else if (theKickOffOwnOutput->active)
  {
    output = theKickOffOwnOutput;
    //log << "active behavior: KickOffOwn" << std::endl;
  }
  else if (theFreekickOppOutput->active)
  {
    output = theFreekickOppOutput;
    //log << "active behavior: FreekickOpp" << std::endl;
  }
  else if (theGoalkickOppOutput->active)
  {
    output = theGoalkickOppOutput;
    //log << "active behavior: GoalkickOpp" << std::endl;
  }
  else if (theGoalkickOwnOutput->active)
  {
    output = theGoalkickOwnOutput;
    //log << "active behavior: GoalkickOwn" << std::endl;
  }
  else if (theStrikerOutput->active)
  {
    output = theStrikerOutput;
    //log << "active behavior: Striker" << std::endl;
  }
  else if (theSupporterOutput->active)
  {
    output = theSupporterOutput;
    //log << "active behavior: Supporter" << std::endl;
  }
  else if (theGoalieOutput->active)
  {
    output = theGoalieOutput;
    //log << "active behavior: Goalie" << std::endl;
  }
  //error
  else
  {
    output = NULL;
    //log << "Error: No behavior running!" << std::endl;
  }
}

void BehaviorCombinator::update(SkillRequest& theSkillRequest)
{
  if (output != NULL)
    theSkillRequest.updateSkillRequest(output->skillRequest);
  else
  {
    //STAND
    theSkillRequest.skill = SkillRequest::NONE;
    theSkillRequest.motionRequest.motion = MotionRequest::SPECIAL_ACTION;
    theSkillRequest.motionRequest.specialActionRequest = MotionRequest::SPECIAL_STAND;
  }
}

void BehaviorCombinator::update(BeamRequest& theBeamRequest)
{
  if (output != NULL)
  {
    theBeamRequest.active = output->beamRequest.active;
    theBeamRequest.pose = output->beamRequest.pose;
  }
  else
    theBeamRequest.active = false;
}

