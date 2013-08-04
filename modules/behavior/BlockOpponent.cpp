#include "BlockOpponent.h"

MAKE_MODULE(BlockOpponent)

bool BlockOpponent::inOppPenaltyArea(const Vector2<double> &v)
{
  const double maxX = theFieldDimensions->halfLength - theFieldDimensions->penaltyAreaDepth;
  const double maxY = theFieldDimensions->halfPenaltyAreaWidth;
  return (fabs(v.y) < maxY && v.x > maxX);
}

void BlockOpponent::update(BlockPosition& theBlockPosition)
{
  //try to find opponent to cover
  float min_dist = -1.0f;
  Vector2<double> closest_opp;
  std::vector<OtherRobots::State>::const_iterator oppIter;
  for (oppIter = theOtherRobots->opponents.begin(); oppIter != theOtherRobots->opponents.end();
      ++oppIter)
  {
    const Vector2<double> &opp = oppIter->pose.translation;
//      if (opp.x < -4.0)
    if (opp.x < theBallPos->absPos.x + 3.0 && !inOppPenaltyArea(opp))
    {
//        if((opp-ball_position).abs() > 2.0)
      if ((opp - theBallPos->absPos).abs() > 2.0) // && (opp-ball_position).abs() < 6.0)
      {
        float my_dist = (theRobotPose->pose.translation - opp).abs();
        bool already_covered = false;
        std::vector<OtherRobots::State>::const_iterator ownIter;
        for (ownIter = theOtherRobots->teammates.begin();
            ownIter != theOtherRobots->teammates.end(); ++ownIter)
        {
          const Vector2<double> &own = ownIter->pose.translation;
          float other_dist = (own - opp).abs();
          if (other_dist < my_dist && other_dist < 1.5) //&& fabs((own-opp).angle_to_vector_deg(ball_position-opp))<90)
            already_covered = true;
        }
        if (!already_covered && (my_dist < min_dist || min_dist < 0))
        {
          min_dist = my_dist;
          closest_opp = opp;
        }
      }
    }
  }

  if (min_dist > 0 && min_dist < 6)
  {
    theBlockPosition.pos = closest_opp
        + (theBallPos->absPos - closest_opp).normalize(std::min(min_dist, 0.6f));
    theBlockPosition.valid = true;
  }
  else
    theBlockPosition.valid = false;
}

