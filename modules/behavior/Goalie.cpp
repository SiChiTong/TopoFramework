#include "Goalie.h"

MAKE_MODULE(Goalie)

void Goalie::init()
{
  penaltyMode = config.getValue("penaltymode", false);
}

Pose2D& Goalie::clipToPenaltyArea(Pose2D &pose)
{
  const double maxX = -theFieldDimensions->halfLength + theFieldDimensions->penaltyAreaDepth - 0.3;
  const double maxY = theFieldDimensions->halfPenaltyAreaWidth - 0.3;
  if (pose.translation.x > maxX)
    pose.translation.x = maxX;
  if (pose.translation.y < -maxY)
    pose.translation.y = -maxY;
  if (pose.translation.y > maxY)
    pose.translation.y = maxY;
  return pose;
}

bool Goalie::inPenaltyArea(const Vector2<double> &v)
{
  const double maxX = -theFieldDimensions->halfLength + theFieldDimensions->penaltyAreaDepth;
  const double maxY = theFieldDimensions->halfPenaltyAreaWidth;
  return (fabs(v.y) < maxY && v.x < maxX);
}

void Goalie::update(GoalieOutput& output)
{

  //check if active
  output.active = (thePlayerRole->role == PlayerRole::GOALIE);
  if (!output.active)
    return;

  //stand up, if fallen
  if (tools.standUp(theFallState, &output.skillRequest))
    return;

  // first check goal shot in every behavior cycle
  bool goal_shot = false;
#ifndef TARGET_NAO
  //TODO Check how this works and if it actually happens and is useful.
  bool left_side = false;
  goal_shot = isBallApproachingGoal(left_side);
#endif

  // extract ball and myself position
  const Vector2<double> &ball_position = theBallPos->absPos;
  const Vector2<double> &my_position = theRobotPose->pose.translation;
  Vector2<double> ownGoalCenter(-theFieldDimensions->halfLength, 0);

  // am I responsible?
  if ((!penaltyMode
      && ((thePlayerRole->closestToBall
          && (ball_position - ownGoalCenter).abs() < config.getValue("alwaysGotoBallDistance", 5.0))
          || inPenaltyArea(ball_position))) || (penaltyMode && inPenaltyArea(ball_position)))
  {
    //log << "closest player" << std::endl;
    //target point
    Vector2<double> own_goal_point(-theFieldDimensions->halfLength, 0);
    Vector2<double> target_point = ball_position + (ball_position - own_goal_point).normalize(6.0);

    drawing.circle("Goalie.target_point", target_point.x, target_point.y, 0.1, 255, 255, 255, 2);

    //check if kick possible
    static bool state_kick = false;
    //if(!state_kick && (distance_to_opponent < 3.0 || dist_to_target < 3.0))
    state_kick = true;
    //if(state_kick && (distance_to_opponent > 3.5 && dist_to_target > 4.0))
    //  state_kick = false;

    //state_kick = true;
    if (state_kick || penaltyMode)
    {
      //if(((dist_to_target > 5.0 && fabs(angle_to_ball_deg) < 70) || fabs(angle_to_ball_deg) < 30))
      //{
      output.skillRequest.skill = SkillRequest::KICK;
      output.skillRequest.kickAccuracy =
          (ball_position - own_goal_point).abs() > config.getValue("halfToleranceDistance", 2.0) ?
              pi_4 : pi_4 * 0.5;
      //}
      //else
      //  output->skillRequest.skill = SkillRequest::GETBALL;
      output.skillRequest.target = target_point;
    }
    else
    {
      //DRIBBLE
      output.skillRequest.skill = SkillRequest::DRIBBLE;
      output.skillRequest.target = target_point;
    }
  }
  else if (goal_shot)
  {
    // handle a goal shot by walking between goal line and ball_position
    Vector2<double> handle_point = ball_position + ball_speed.normalize(2.0);
    handle_point.x = (std::max(handle_point.x, -17.0));

    float ball_speed_direction = this->ball_speed.angle();
    Pose2D temp_pose(ball_speed_direction, handle_point.x, handle_point.y);

    output.skillRequest.skill = SkillRequest::MOVETOPOS;
    output.skillRequest.moveToRequest.setValues(temp_pose, -1, true);

  }
  else
  {
    // repositioning
    // new position distance factor between 0 ... 1 (1 means nearest to ball) 
    float pos_dist_factor = (penaltyMode ? 0.9 : 0.33);
    Vector2<double> mid_point_goal(-theFieldDimensions->halfLength, 0.0);
    Vector2<double> center_goal_to_ball = ball_position - mid_point_goal;
    float ang_goal_mid_point_to_ball = atan2(center_goal_to_ball.y, center_goal_to_ball.x);

    float dist = std::min(config.getValue("maxDistanceToGoal", 3.0),
        center_goal_to_ball.abs() * pos_dist_factor);
    Vector2<double> temp = mid_point_goal + center_goal_to_ball.normalize(dist);

    //HACK: keep walk target close, to avoid turning of agent 
    double maxWalkTargetDist = config.getValue("maxWalkTargetDistance", 1.5);
    if ((my_position - temp).abs() > maxWalkTargetDist)
      temp = my_position + (temp - my_position).normalize(maxWalkTargetDist);

    Pose2D temp_pose(ang_goal_mid_point_to_ball, temp.x, temp.y);
    if (penaltyMode)
      clipToPenaltyArea(temp_pose);
    // execute skill move
    output.skillRequest.skill = SkillRequest::MOVETOPOS;
    output.skillRequest.moveToRequest.setValues(temp_pose, -1, true, 0.2);
  }
}

bool Goalie::isBallApproachingGoal(bool &left_side)
{
  bool goal_shot = false;
  const Vector2<double> &ball_position = theBallPos->absPos;
  const Vector2<double> &my_position = theRobotPose->pose.translation;

  if (ball_position.x > 0.0)
  {
    // no danger
    goal_shot = false;
  }
  else
  {
    // get ball speed from current and last ball position
    this->ball_speed = ball_position - ball_previous_position;
    float ball_speed_length = (this->ball_speed.abs() > 0.1) ? this->ball_speed.abs() : 0.0f;
    float ball_speed_direction = atan2(this->ball_speed.y, this->ball_speed.x) * (180.0 / M_PI)
        + 180.0;
    if (ball_speed_direction < -180)
      ball_speed_direction += 360;
    if (ball_speed_direction > 180)
      ball_speed_direction -= 360;

    // calculate angles from ball to goal posts
    Vector2<double> ball_to_right_post(-theFieldDimensions->halfLength,
        theFieldDimensions->halfGoalWidth);
    ball_to_right_post -= ball_position;
    Vector2<double> ball_to_left_post(-theFieldDimensions->halfLength,
        -theFieldDimensions->halfGoalWidth);
    ball_to_left_post -= ball_position;

    float ang_ball_to_right_post(
        atan2(ball_to_right_post.y, ball_to_right_post.x) * (180.0 / M_PI));
    float ang_ball_to_left_post(atan2(ball_to_left_post.y, ball_to_left_post.x) * (180.0 / M_PI));

    ang_ball_to_right_post += 180.0;
    if (ang_ball_to_right_post < -180)
      ang_ball_to_right_post += 360;
    if (ang_ball_to_right_post > 180)
      ang_ball_to_right_post -= 360;

    ang_ball_to_left_post += 180.0;
    if (ang_ball_to_left_post < -180)
      ang_ball_to_left_post += 360;
    if (ang_ball_to_left_post > 180)
      ang_ball_to_left_post -= 360;

    // goal shot is true if the current ball_speed_direction faces the goal and the ball is moving
    bool goal_shot = ((ang_ball_to_right_post > ball_speed_direction)
        && (ang_ball_to_left_post < ball_speed_direction) && ball_speed_length > 0.1);

    // in case of a goal shot choose the side to stop the ball
    if (goal_shot)
    {
      Vector2<double> ball_to_myself = my_position - ball_position;
      float ang_ball_to_myself(atan2(ball_to_myself.y, ball_to_myself.x) * (180.0 / M_PI));

      left_side = ang_ball_to_myself < ball_speed_direction;
    }
  }

  // always updating ball previous position  
  ball_previous_position = ball_position;

  return goal_shot;
}

