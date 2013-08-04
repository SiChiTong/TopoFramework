#include "Striker.h"

MAKE_MODULE(Striker)

void Striker::update(StrikerOutput& output)
{
  //check if active
  output.active = (thePlayerRole->role == PlayerRole::STRIKER);
  if (!output.active)
    return;

  //stand up
  if (tools.standUp(theFallState, &output.skillRequest))
    return;

  //------- striker behavior based on striker from old agent

  // ball and myself position
  const Vector2<double> &ball_position = theBallPos->absPos;
  const Vector2<double> &my_position = theRobotPose->pose.translation;
  float my_direction = theRobotPose->pose.rotation;

  //target point
  Vector2<double> target_point;
  float half_goal_width = theFieldDimensions->halfGoalWidth;
  float opp_goal_x = theFieldDimensions->halfLength;
  double target_x_offset = (fabs(ball_position.y) < half_goal_width ? 0.3 : -0.1);
  //near own goal target is only straight away from goal
  if ((my_position - Vector2<double>(-opp_goal_x, 0)).abs() < 5.0)
  {
    target_point = (ball_position - Vector2<double>(-opp_goal_x - 0.3, 0)).normalize(10.0);
  }
  else
  {
    target_point = Vector2<double>(opp_goal_x + target_x_offset, ball_position.y);
    target_point.y = std::min(half_goal_width - 0.2,
        std::max(-half_goal_width + 0.2, target_point.y));
  }

  //find closest opponent
  float distance_to_opponent = 100.0f;
  float distance_opp_ball = 100.0f;
  float angle_to_opponent = 0, angle_to_opponent_deg = 0;
  float opp_x = 20, opp_y = 20;
  bool oppFallen = false;
  std::vector<LocalOtherRobots::State>::const_iterator oppIter;
  for (oppIter = theLocalOtherRobots->opponents.begin();
      oppIter != theLocalOtherRobots->opponents.end(); ++oppIter)
  {
    const Vector2<double> &pos_2d = oppIter->pose.translation;
    float angle = (pos_2d - ball_position).angle() - my_direction;
    angle = normalize(angle);
    float dist = (pos_2d - ball_position).abs();
    if (dist < distance_opp_ball && fabs(angle) < 70.0 * pi / 180.0)
    {
      distance_opp_ball = dist;
      distance_to_opponent = (pos_2d - my_position).abs();
      angle_to_opponent = angle;
      angle_to_opponent_deg = angle * 180.0 / pi;
      opp_x = pos_2d.x;
      opp_y = pos_2d.y;
      oppFallen = oppIter->fallen();
    }
  }
  Vector2<double> opp(opp_x, opp_y);
  //calculate some values to decide what to do                
  Vector2<double> vec_orientation = Vector2<double>(1, 0).rotate(my_direction);
  float opp_shift = sin(fabs(angle_to_opponent)) * distance_to_opponent;
  if (distance_to_opponent > 10.0 || fabs(angle_to_opponent_deg) > 90)
    opp_shift = 100.0;

  drawing.circle("Striker.debugdebug", target_point.x, target_point.y, 0.5, 2, 255, 0, 0);

  float dist_to_target = (ball_position - target_point).abs();
  float angle_to_target_point_deg = (target_point - my_position).angleToVector(vec_orientation);
  angle_to_target_point_deg = normalize(angle_to_target_point_deg) * 180.0 / pi;

  drawing.line("Striker.dribble_target", ball_position.x, ball_position.y, 0, target_point.x,
      target_point.y, 0, 255, 255, 255, 1);

  //check if kick possible/neccessary
  if (!state_kick /*&& fabs(angle_to_ball_and_target_deg) < 50*/
  && /*distance_to_opponent < 2.0  opp_shift < 2.0 &&*/
  (dist_to_target < 4.0 || ball_position.x > opp_goal_x - 2.0)
  /*&& (opp_shift > 0.1 || distance_to_opponent>0.5)*/)
    state_kick = true;
  if (state_kick && (/*fabs(angle_to_ball_and_target_deg) > 60
   ||*//*distance_to_opponent > 3.5 || opp_shift > 3.5 ||*/
  (dist_to_target > 4.5 && ball_position.x < opp_goal_x - 2.5)
  /*|| (opp_shift < 0.05 && distance_to_opponent < 0.4)*/))
    state_kick = false;

  //value like opp_shift, but relative to direction to dribble target
  float angleToOppToTarget = (opp - ball_position).angleToVector(target_point - ball_position);
  float oppShiftToTarget = sin(fabs(angleToOppToTarget)) * distance_to_opponent;
  if (distance_to_opponent > 10.0 || fabs(angleToOppToTarget) > 90)
    oppShiftToTarget = 100.0;

  //check if pass would be good
  //opp close, but not completely blocking, in pass direction a teammate, pass towards opp goal
  Vector2<double> toOppGoal = target_point - ball_position;
  //Vector2<double> toOppGoal = Vector2<double>(opp_goal_x, 0)-ball_position;  
  float pass_direction_deg = normalize((ball_position - my_position).angleToVector(toOppGoal))
      * 180.0 / pi;

  if (state_kick)
  {
    Vector2<double> leftPost(theFieldDimensions->halfLength, theFieldDimensions->halfGoalWidth);
    Vector2<double> rightPost(theFieldDimensions->halfLength, -theFieldDimensions->halfGoalWidth);
    Vector2<double> goalCenter(theFieldDimensions->halfLength, 0);
    double maxAngleError = std::min(
        fabs(normalize((goalCenter - ball_position).angleToVector(leftPost - ball_position))),
        fabs(normalize((goalCenter - ball_position).angleToVector(rightPost - ball_position))));

    drawing.line("Striker.goalkick", ball_position.x, ball_position.y, 0, goalCenter.x,
        goalCenter.y, 0, 255, 0, 0, 8);
    drawing.line("Striker.goalkick", ball_position.x, ball_position.y, 0, leftPost.x, leftPost.y, 0,
        255, 0, 0, 3);
    drawing.line("Striker.goalkick", ball_position.x, ball_position.y, 0, rightPost.x, rightPost.y,
        0, 255, 0, 0, 3);
    output.skillRequest.skill = SkillRequest::KICK;
    output.skillRequest.target = goalCenter;
    output.skillRequest.kickAccuracy = std::max(0.2, maxAngleError * 0.8);
  }
  else
  {
    //if opponent is close, try to kick, when ball is in the right place
    float angle_ball_opp = (ball_position - my_position).angleToVector(opp - my_position);
    angle_ball_opp = normalize(angle_ball_opp) * 180.0 / pi;
    static bool panic_kick = false;
    if (!panic_kick
        && (oppShiftToTarget < 2.0 && distance_opp_ball < 2.5 && fabs(pass_direction_deg) < 80)) //&& fabs(angle_to_target_point_deg) < 60))
      panic_kick = true;
    if (panic_kick
        && (oppShiftToTarget > 2.5 || distance_opp_ball > 3.0 || fabs(pass_direction_deg) > 100)) //|| fabs(angle_to_target_point_deg) > 80))
      panic_kick = false;
    if (panic_kick)
    {
      static int panicOppState = 0;
      Vector2<double> toTarget = target_point - ball_position;
      double oppAngle = normalize((opp - ball_position).angle() - toTarget.angle());

      //set target to opponent to block if opponent close to ball
      const double oppTargetAngle = normalize(
          (opp - ball_position).angle()
              - (ball_position - Vector2<double>(-opp_goal_x, 0)).angle());
      if (theGamestate->playmode == Gamestate::PLAYON && distance_opp_ball < 0.5) //&& fabs(oppTargetAngle) < 0.7*pi)
      {
        toTarget = opp - ball_position;
        oppAngle = 0;
      }

      //direction of panic kick depending on opponent (middle, left, right) 
      if (panicOppState == -1 && oppAngle > 0.6)
        panicOppState = 0;
      if (panicOppState == -1 && oppAngle < -0.2)
        panicOppState = 1;
      if (panicOppState == 1 && oppAngle < -0.6)
        panicOppState = 0;
      if (panicOppState == 1 && oppAngle > 0.2)
        panicOppState = -1;
      if (panicOppState == 0 && oppAngle > 0 && oppAngle < 0.5)
        panicOppState = -1;
      if (panicOppState == 0 && oppAngle < 0 && oppAngle > -0.5)
        panicOppState = 1;
      target_point = theBallPos->absPos + toTarget.rotate(panicOppState * 0.4).normalize(5.0);

      drawing.line("Striker.panicKick", ball_position.x, ball_position.y, 0, target_point.x,
          target_point.y, 0, 150, 0, 0, 8);
      output.skillRequest.skill = SkillRequest::KICK;
      output.skillRequest.target = target_point;
      output.skillRequest.kickAccuracy = 0.3;
    }
    //if not kicked, continue dribbling
    else //if(!kicked)
    {
      drawing.line("Striker.dribble", ball_position.x, ball_position.y, 0, target_point.x,
          target_point.y, 0, 0, 150, 0, 8);
      output.skillRequest.skill = SkillRequest::DRIBBLE;
      output.skillRequest.target = target_point;
    }
  }
}

