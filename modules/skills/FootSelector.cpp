#include "FootSelector.h"

MAKE_MODULE(FootSelector)

void FootSelector::update(ActiveFoot& theActiveFoot)
{
  //closest opp
  const Vector2<double> &ball_position = theBallPos->absPos;
  const Vector2<double> &my_position = theRobotPose->pose.translation;
  double my_direction = theRobotPose->pose.rotation;
  float distance_to_opponent = 100.0f;
  float distance_opp_ball = 100.0f;
  float angle_to_opponent_deg = 0;
  Vector2<double> opp(100, 0);
  std::vector<OtherRobots::State>::const_iterator oppIter;
  for (oppIter = theLocalOtherRobots->opponents.begin();
      oppIter != theLocalOtherRobots->opponents.end(); ++oppIter)
  {
    const Vector2<double> &pos_2d = oppIter->pose.translation;
    float angle = (pos_2d - ball_position).rotate(-my_direction).angle();
    angle = normalize(angle);
    angle *= 180.0 / pi;
    float dist = (pos_2d - ball_position).abs();
    if (dist < distance_opp_ball)
    {
      distance_opp_ball = dist;
      distance_to_opponent = (pos_2d - my_position).abs();
      angle_to_opponent_deg = (angle <= 180 ? angle : angle - 360);
      opp = pos_2d;
    }
  }

  if (!footFromOpp && distance_to_opponent < 1.5)
    footFromOpp = true;
  if (footFromOpp && distance_to_opponent > 2.0)
    footFromOpp = false;

  double ballDist = theBallPos->relativePos.abs();

  if (footFromOpp)
  {
    //left or right foot from opponent
    Vector2<double> toTarget = theSkillRequest->target.translation - theBallPos->absPos;
    double oppAngle = normalize((opp - ball_position).angle() - toTarget.angle());
    oppAngle = oppAngle / pi * 180.0;

    if (left && oppAngle > 20 && ballDist > 0.3)
      left = false;
    if (!left && oppAngle < -20 && ballDist > 0.3)
      left = true;
  }
  else
  {
    //otherwise select foot using direction to target

    const Vector2<double> &target_point = theSkillRequest->target.translation;
    const Vector2<double> &ball_position = theBallPos->absPos;

    double angleDiff = (target_point - ball_position).rotate(-theRobotPose->pose.rotation).angle();
    while (angleDiff > M_PI)
      angleDiff -= M_PI * 2.0;
    while (angleDiff < -M_PI)
      angleDiff += M_PI * 2.0;
    if (left && angleDiff < -0.2 && ballDist > 0.15)
      left = false;
    if (!left && angleDiff > 0.2 && ballDist > 0.15)
      left = true;
  }

  //foot distance values
  double dist_x = 0.115;
  double dist_y = 0.06;
  if (!left)
    dist_y *= -1;

  theActiveFoot.offset = Vector2<double>(dist_x, dist_y);
  theActiveFoot.left = left;

  /** @todo move drawing to ActiveFoot representation, needs local coord. */
  Vector2<double> drawFoot = theRobotPose->pose * theActiveFoot.offset;
  drawing.circle("FootSelector.foot", drawFoot.x, drawFoot.y, 0.03, 2, 255, 255, 200);

}

