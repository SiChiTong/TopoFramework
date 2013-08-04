#include "FreekickOpp.h"

MAKE_MODULE(FreekickOpp)

void FreekickOpp::update(FreekickOppOutput& output)
{
  output.active = thePlayerRole->role == PlayerRole::STRIKER
      && (theGamestate->playmode == Gamestate::FREEKICK_OPP
          || theGamestate->playmode == Gamestate::CORNERKICK_OPP
          || theGamestate->playmode == Gamestate::KICKIN_OPP
          || theGamestate->playmode == Gamestate::GOALKICK_OPP);
  if (!output.active)
    return;

  //stand up, if fallen
  if (tools.standUp(theFallState, &output.skillRequest))
    return;

  const Vector2<double> &ball_position = theBallPos->absPos;
  Vector2<double> rel_ball_to_goal_mid_point(
      (-theFieldDimensions->halfLength + 0.3) - ball_position.x, -ball_position.y);
  rel_ball_to_goal_mid_point.normalize(1.7);

  Vector2<double> point_to_move(ball_position + rel_ball_to_goal_mid_point);
  float ang_to_ball = normalize(pi + rel_ball_to_goal_mid_point.angle());
  Pose2D temp(ang_to_ball, point_to_move);

  output.skillRequest.skill = SkillRequest::MOVETOPOS;
  output.skillRequest.moveToRequest.setValues(temp);
}

