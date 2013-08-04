#include "Supporter.h"

MAKE_MODULE(Supporter)

void Supporter::update(SupporterOutput& output)
{
  //check if active
  output.active = (thePlayerRole->role == PlayerRole::SUPPORTER);
  if (!output.active)
    return;

  //stand up, if fallen
  if (tools.standUp(theFallState, &output.skillRequest))
    return;

  output.skillRequest.skill = SkillRequest::MOVETOPOS;
  output.skillRequest.moveToRequest.setValues(thePlayerRole->pose, 0.3, true, 0.3);

}

