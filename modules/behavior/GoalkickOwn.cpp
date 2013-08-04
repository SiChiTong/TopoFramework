#include "GoalkickOwn.h"

MAKE_MODULE(GoalkickOwn)

void GoalkickOwn::update(GoalkickOwnOutput& output)
{
  output.active = (thePlayerRole->role == PlayerRole::SUPPORTER)
      && (theGamestate->playmode == Gamestate::GOALKICK_OWN);
  if (!output.active)
    return;

  //stand up, if fallen
  if (tools.standUp(theFallState, &output.skillRequest))
    return;

  output.skillRequest.skill = SkillRequest::MOVETOPOS;
  output.skillRequest.moveToRequest.setValues(theRobotPose->pose);

  //go out of penalty area
  output.skillRequest.moveToRequest.target.translation.x = -theFieldDimensions->halfLength + 2.2;
  output.skillRequest.moveToRequest.target.rotation = 0;

}

