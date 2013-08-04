#include "BeforeKickOff.h"

MAKE_MODULE(BeforeKickOff)

void BeforeKickOff::update(BeforeKickOffOutput& output)
{
  output.active = (theGamestate->playmode == Gamestate::BEFORE_KICKOFF
      || theGamestate->playmode == Gamestate::KICKOFF_OPP
      || theGamestate->playmode == Gamestate::GOAL_OWN
      || theGamestate->playmode == Gamestate::GOAL_OPP);
  if (!output.active)
    return;

  //stand up, if fallen (if not in BEFORE_KICKOFF and beam is allowed)
  if (theGamestate->playmode != Gamestate::BEFORE_KICKOFF
      && tools.standUp(theFallState, &output.skillRequest))
    return;

  //get kickoff position
  //log << "using unum " << thePlayerInfo->unum << std::endl;
  char xName[32];
  char yName[32];
  int unum = std::min(11, std::max(0, thePlayerInfo->unum));
  snprintf(xName, 31, "kickOffPos_%d_x", unum);
  snprintf(yName, 31, "kickOffPos_%d_y", unum);
  double x = -0.9 + thePlayerInfo->unum / 20.0;
  Pose2D kickOffPos(0, config.getValue(xName, x), config.getValue(yName, 0.0));
  //log << "position from config: " << xName << "=" << kickOffPos.translation.x << "  " << yName
  //    << "=" << kickOffPos.translation.y << "  " << std::endl;
  kickOffPos.translation.x *= theFieldDimensions->halfLength;
  kickOffPos.translation.y *= theFieldDimensions->halfWidth;

  //calculate error to kick off position
  double dist = (theRobotPose->pose.translation - kickOffPos.translation).abs();
  double angle = normalize(theRobotPose->pose.rotation - kickOffPos.rotation);

  bool alreadyInPosition = (dist < 0.1 && fabs(angle) < 0.2);

  if (theGamestate->playmode == Gamestate::KICKOFF_OPP && !alreadyInPosition)
  {
    //walk
    output.skillRequest.skill = SkillRequest::MOVETOPOS;
    output.skillRequest.moveToRequest.setValues(kickOffPos, -1, true, 0.2);
  }
  else
  {
    //stand
    output.skillRequest.skill = SkillRequest::NONE;
    output.skillRequest.motionRequest.motion = MotionRequest::STAND;
  }

  //beam
  output.beamRequest.active = (!alreadyInPosition
      && (theGamestate->playmode != Gamestate::KICKOFF_OPP));
  if (output.beamRequest.active)
    output.beamRequest.pose = kickOffPos;
}

