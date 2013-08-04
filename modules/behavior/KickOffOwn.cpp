#include "KickOffOwn.h"

MAKE_MODULE(KickOffOwn)

void KickOffOwn::update(KickOffOwnOutput& output)
{
  //check if active
  output.active = (thePlayerRole->role == PlayerRole::STRIKER
      && theGamestate->playmode == Gamestate::KICKOFF_OWN);
  if (!output.active)
    return;

  //stand up, if fallen
  if (tools.standUp(theFallState, &output.skillRequest))
    return;

  Pose2D pose(config.getValue("kickpose.rotation", 0), config.getValue("kickpose.x", -0.05),
      config.getValue("kickpose.y", -0.2));

  if ((pose.translation - theRobotPose->pose.translation).abs()
      < config.getValue("theshold_translation", 0.01)
      && fabs(normalize(pose.rotation - theRobotPose->pose.rotation))
          < config.getValue("theshold_rotation", 0.01))
  {
    output.skillRequest.skill = SkillRequest::NONE;
    output.skillRequest.motionRequest.motion = MotionRequest::SPECIAL_ACTION;
    output.skillRequest.motionRequest.specialActionRequest =
        (MotionRequest::SpecialAction) config.getValue("kick_special_action",
            MotionRequest::SPECIAL_KICK_RIGHT);

  }
  else
  {
    output.skillRequest.skill = SkillRequest::MOVETOPOS;
    output.skillRequest.moveToRequest.setValues(pose, -1, false, 0);
  }

}

