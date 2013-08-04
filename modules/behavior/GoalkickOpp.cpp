#include "GoalkickOpp.h"

MAKE_MODULE(GoalkickOpp)

void GoalkickOpp::update(GoalkickOppOutput& output)
{
  output.active = (thePlayerRole->role == PlayerRole::SUPPORTER)
      && (theGamestate->playmode == Gamestate::GOALKICK_OPP);
  if (!output.active)
    return;

  //stand up, if fallen
  if (tools.standUp(theFallState, &output.skillRequest))
    return;

  output.skillRequest.skill = SkillRequest::MOVETOPOS;
  output.skillRequest.moveToRequest.setValues(thePlayerRole->pose);

  //go to supporter positions, but 2m shifted to the left
  if (output.skillRequest.moveToRequest.target.translation.x > 0)
    output.skillRequest.moveToRequest.target.translation.x -= 3.0;

}

