#include "Formation.h"

MAKE_MODULE(Formation)

void Formation::init()
{
  confNumBlocker = config.getValue("numberOfBlocker", 2);
  confNumStrikerSupporter = config.getValue("numberOfStrikerSupporter", 3);
  nBlocker = confNumBlocker;
}

void Formation::execute()
{
  if (theGamestate->playmode == Gamestate::KICKOFF_OWN)
    timeLastKickoff = theFrameInfo->time_ms;
}

void Formation::update(PlayerRole& thePlayerRole)
{
  int skip = -1;
  /*int kickoffWaitTime = theFrameInfo->time_ms - timeLastKickoff;
   if(kickoffWaitTime < 4000 && theGamestate->playmode != Gamestate::KICKOFF_OWN)
   {
   printf("waittime %d\n", kickoffWaitTime);
   skip = 2; //skip striker
   }*/

  thePlayerRole.closestToBall = closest_to_ball(skip);
  if (thePlayerInfo->unum == config.getValue("goalieUnum", 1))
    thePlayerRole.role = PlayerRole::GOALIE;
  else if (thePlayerRole.closestToBall || theGamestate->penaltyShootout)
    thePlayerRole.role = PlayerRole::STRIKER;
  else
  {
    thePlayerRole.role = PlayerRole::SUPPORTER;

    //set number of blockers
    if (theGamestate->playmode != Gamestate::PLAYON)
      nBlocker = confNumBlocker;
    //  near opponent goal
    if (theBallPos->absPos.x > theFieldDimensions->halfLength - 3.0)
      nBlocker = 0;
    if (theBallPos->absPos.x < theFieldDimensions->halfLength - 4.0)
      nBlocker = confNumBlocker;
    //  near own goal
    //if(theBallPos->absPos.x < -theFieldDimensions->halfWidth + 2.0)
    //  nBlocker = 0;
    //if(theBallPos->absPos.x > -theFieldDimensions->halfWidth + 3.0)
    //  nBlocker = 2;

    //get position
    thePlayerRole.pose = supporterAndDefendersSimple();
  }
}

bool Formation::closest_to_ball(int skip)
{
  //Vector2d ball_position( extract_perception->get_cartesian_position( VO_BALL )->get_x(),
  //		  extract_perception->get_cartesian_position( VO_BALL )->get_y() );
  //return this->closest_to_point( ball_position, (ball_position-opp).scale(1.0));

  if (thePlayerInfo->unum == skip)
    return false;

  const Vector2<double> &ball_position = theBallPos->absPos;
  const Vector2<double> &my_position = theRobotPose->pose.translation;

  Vector2<double> opp(theFieldDimensions->halfLength * 1.2, 0);
  float distance_to_opponent = 100.0f;
  std::vector<OtherRobots::State>::const_iterator oppIter;
  for (oppIter = theOtherRobots->opponents.begin(); oppIter != theOtherRobots->opponents.end();
      ++oppIter)
  {
    const Vector2<double> &pos_2d = oppIter->pose.translation;
    float dist = (pos_2d - ball_position).abs();
    if (dist < distance_to_opponent)
    {
      distance_to_opponent = dist;
      opp = pos_2d;
    }
  }

  Vector2<double> point_speed = (ball_position - opp).normalize(1.0);
  Vector2<double> point_with_speed = get_intercept_pos(my_position, opp, point_speed);
  if ((point_with_speed - opp).abs() < (ball_position - opp).abs())
    point_with_speed = ball_position;

  float my_value = get_walk_cost(my_position, point_with_speed);

  int n = 0;
  bool lyingRobotNearBall = false;
  std::vector<OtherRobots::State>::const_iterator ownIter;
  for (ownIter = theOtherRobots->teammates.begin(); ownIter != theOtherRobots->teammates.end();
      ++ownIter)
  {
    const Vector2<double> &pos_2d = ownIter->pose.translation;

    point_with_speed = get_intercept_pos(pos_2d, opp, point_speed);
    if ((point_with_speed - opp).abs() < (ball_position - opp).abs())
      point_with_speed = ball_position;

    float player_value = get_walk_cost(pos_2d, point_with_speed);

    //extra distance value for lying robots
    const float radius_for_second_robot = 0.6;
    const float radius_for_third_robot = 1.6;
    if (ownIter->fallen() && (pos_2d - my_position).abs() < 5
        && !((pos_2d - ball_position).abs() < radius_for_second_robot)) //&& !(pos_2d.distance_to(ball_position) < radius_for_third_robot
                                                                        //   && lyingRobotNearBall)
    {
      player_value += 5.0;
      if ((pos_2d - ball_position).abs() <= radius_for_third_robot)
        lyingRobotNearBall = true;
      drawing.circle("Formation.fallenPlayer", pos_2d.x, pos_2d.y, 0.35, 2, 255, 200, 200);
    }

    //if(ownIter->unum != 1 || 
    //    (point_with_speed-Vector2<double>(-theFieldDimensions->halfLength,0)).abs() < 4.0)
    if (player_value < my_value)
    {
      //if(ownIter->unum != skip)
      n++;
      //else
      //  printf("player %d skipped\n", skip);
      drawing.circle("Formation.closerPlayer", pos_2d.x, pos_2d.y, 0.3, 2, 255, 255, 255);
    }
  }
  return (n == 0);
}

float Formation::get_walk_cost(Vector2<double> robot, Vector2<double> dest)
{
  float gw = theFieldDimensions->halfGoalWidth;
  float goalY = std::min(std::max((float) theBallPos->absPos.y, -gw), gw);
  float angle_factor = 0.7;
  float dist = (dest - robot).abs();
  float angle_to_point = (dest - robot).angleToVector(
      Vector2<double>(theFieldDimensions->halfLength + 1.0, goalY) - dest);
  angle_to_point = fabs(normalize(angle_to_point));
  return dist * (1.0 + angle_factor * angle_to_point);
}

Vector2<double> Formation::get_intercept_pos(Vector2<double> robot, Vector2<double> point,
    Vector2<double> point_speed)
{
  //modify point using point speed
  Vector2<double> point_with_speed = point;
  Vector2<double> robot_to_point = point - robot;
  if (point_speed.abs() > 0.1 && robot_to_point.abs() > 0.3)
  {
    double intercept_dist = 1.0;
    double angle = (robot - point).angleToVector(point_speed);
    if (angle > M_PI)
      angle -= 2.0 * M_PI;
    if (fabs(angle) < M_PI_2)
    {
      const double d = robot_to_point.abs();
      const double f = point_speed.abs();
      double p = -(d * f * cos(angle)) / (f * f - 1);
      double q = (d * d) / (f * f - 1);
      double a1 = -p + sqrt(p * p - q);
      double a2 = -p - sqrt(p * p - q);
      if (a1 >= 0 && a1 < 10)
        intercept_dist = a1;
      else
        intercept_dist = a2;
      /*Vector2<double> t = point + Vector2<double>(point_speed).normalize(intercept_dist);
       D_DRAW_LINE("behavior.point_speed", point.get_x(), point.get_y(),0,
       robot.get_x(), robot.get_y(),0, 1,0.6,1, 1);
       D_DRAW_LINE("behavior.point_speed", robot.get_x(), robot.get_y(),0,
       t.get_x(), t.get_y(),0, 1,0.6,1, 1);
       D_DRAW_LINE("behavior.point_speed", t.get_x(), t.get_y(),0,
       point.get_x(), point.get_y(),0, 1,0.3,1, 1);
       */
    }
    if (intercept_dist > 5.0)
      intercept_dist = 5.0;
    if (intercept_dist > 0)
      point_with_speed = point + point_speed.normalize(intercept_dist);
    ;
  }
  return point_with_speed;
}

Pose2D Formation::supporterAndDefendersSimple()
{
  // extract ball and myself position
  const Vector2<double> &ball_position = theBallPos->absPos;
  const Vector2<double> &my_position = theRobotPose->pose.translation;

  // calc dist from ball to opponent goal mid point
  Vector2<double> own_goal_mid_point(-theFieldDimensions->halfLength, 0.0);
  Vector2<double> opp_goal_mid_point(theFieldDimensions->halfLength, 0.0);
  //Vector2<double> ball_to_opp_goal_mid_point = opp_goal_mid_point - ball_position;

  //get number of other players closer to the opponent goal
  int others_closer = ownPlayersCloserToPoint(opp_goal_mid_point);
  //printf("unum = %d  others_closer = %d\n", thePlayerInfo->unum, others_closer);

  Pose2D supporter_pose;
  bool covering = false;
//  if(ball_position.x < -3.0 && others_closer >= 3)
  if (others_closer < nBlocker || (ball_position.x < -6.0 && others_closer < 5))
  {
    if (theBlockPosition->valid)
    {
      supporter_pose = Pose2D(0, theBlockPosition->pos);
      covering = true;
    }
  }

  const int numStrikerSupporter = confNumStrikerSupporter;

  if (!covering && others_closer < nBlocker + 1 + numStrikerSupporter)
  {
    // striker supporter
    //others_closer is 1 to 3 (if nBlocker==0), 
    //but may change fast, so positioning independend from this number
    Vector2<double> temp_position = getSupporterPosition(ball_position);

    drawing.line("Formation", my_position.x, my_position.y, 0, temp_position.x, temp_position.y, 0,
        0, 0, 0, 1);

    //Vector2d look_direction = ((opp_goal_mid_point-my_position)+(ball_position-my_position));
    //float temp_ang = atan2(look_direction.get_y(), 
    //                       look_direction.get_x())*(180.0/M_PI);
    float temp_ang = 0;
    supporter_pose = Pose2D(temp_ang, temp_position.x, temp_position.y);
  }

  if (!covering && others_closer >= nBlocker + 1 + numStrikerSupporter)
  {
    float dist = config.getValue("defenders_startBallDist", 3.0);
    float minDistToGoal = 1.1 * theFieldDimensions->penaltyAreaDepth;
    float supporter_ball_dist = std::max(0.0f,
        std::min((float) (own_goal_mid_point - ball_position).abs() - minDistToGoal, dist));
    Vector2<double> temp_position = ball_position
        + (own_goal_mid_point - ball_position).normalize(supporter_ball_dist);

    int def_index = others_closer - (numStrikerSupporter + nBlocker);
    double ballgoaldist = (own_goal_mid_point - ball_position).abs();
    if (def_index > 0)
    {
      if ((def_index & 1) > 0)
        temp_position.y = (temp_position.y + def_index * 0.03 * ballgoaldist);
      else
        temp_position.y = (temp_position.y - def_index * 0.03 * ballgoaldist);
      temp_position.x = (temp_position.x - def_index * 0.08 * ballgoaldist);
    }

    double min_x = -theFieldDimensions->halfLength + config.getValue("defenders_borderDistX", 0.5);
    if (temp_position.x < min_x)
      temp_position.x = min_x;
    double max_y = theFieldDimensions->halfLength - config.getValue("defenders_borderDistY", 2.0);
    if (temp_position.y > max_y)
      temp_position.y = max_y;
    if (temp_position.y < -max_y)
      temp_position.y = -max_y;

    //if temp_position in penalty area, avoid collisions
    if (inPenaltyArea(temp_position))
    {
      double upperY = theFieldDimensions->halfGoalWidth * 2.0;
      double lowerY = -theFieldDimensions->halfGoalWidth * 2.0;
      std::vector<OtherRobots::State>::const_iterator ownIter;
      for (ownIter = theOtherRobots->teammates.begin(); ownIter != theOtherRobots->teammates.end();
          ++ownIter)
      {
        const Vector2<double> &pos = ownIter->pose.translation;
        if (inPenaltyArea(pos))
        {
          if (pos.y > my_position.y && pos.y < upperY)
            upperY = pos.y;
          if (pos.y < my_position.y && pos.y > lowerY)
            lowerY = pos.y;
        }
      }
      temp_position.y = (upperY + lowerY) / 2.0;
    }

    Vector2<double> temp_position_to_ball = ball_position - temp_position;
    float temp_ang = temp_position_to_ball.angle();

    supporter_pose = Pose2D(temp_ang, temp_position.x, temp_position.y);

  }

  return supporter_pose;
}

bool Formation::inPenaltyArea(const Vector2<double> &v)
{
  const double maxX = -theFieldDimensions->halfLength + theFieldDimensions->penaltyAreaDepth;
  const double maxY = theFieldDimensions->halfPenaltyAreaWidth;
  return (fabs(v.y) < maxY && v.x < maxX);
}

Vector2<double> Formation::getSupporterPosition(Vector2<double> ball_position)
{
  const int nSupporter = confNumStrikerSupporter;
  const Vector2<double> &my_position = theRobotPose->pose.translation;
  Vector2<double> opp_goal_mid_point = Vector2<double>(theFieldDimensions->halfLength, 0.0);

  //smooth thresholds for ball left/right/middle
  static int ballPos = 0;
  if (nSupporter <= 1)
    ballPos = -1;
  else if (nSupporter == 2)
  {
    if (ballPos == 0 && ball_position.y < -theFieldDimensions->halfWidth * 0.2)
      ballPos = -1;
    if (ballPos == -1 && ball_position.y > theFieldDimensions->halfWidth * 0.2)
      ballPos = 0;
  }
  else
  {
    if (ballPos == 0 && ball_position.y < -theFieldDimensions->halfWidth * 0.45)
      ballPos = -1;
    if (ballPos == 0 && ball_position.y > theFieldDimensions->halfWidth * 0.45)
      ballPos = 1;
    if (ballPos == -1 && ball_position.y > -theFieldDimensions->halfWidth * 0.3)
      ballPos = 0;
    if (ballPos == 1 && ball_position.y < theFieldDimensions->halfWidth * 0.3)
      ballPos = 0;
  }
  //calculate three positions based on ball position
  double oppGoalMinDist = config.getValue("supporter_oppGoalMinDist", 1.5);
  double centerPosOffsetX = config.getValue("supporter_centerPosOffsetX", -1.5);
  double sidePosDistY = config.getValue("supporter_sidePosDistY", 2.0);
  Vector2<double> pos[nSupporter];
  for (int i = 0; i < nSupporter; i++)
  {
    pos[i] = ball_position
        + Vector2<double>((i == ballPos + 1 ? centerPosOffsetX : 0),
            sidePosDistY * (i - ballPos - 1));
    pos[i].x = std::min(opp_goal_mid_point.x - oppGoalMinDist, pos[i].x);
  }
  /*for(int i=0; i<3; i++)
   D_DRAW_CIRCLE("behavior.supporter", pos[i].get_x(),pos[i].get_y(),0.1, 0,0,0, 4);*/

  //find the nBlocker+1+nRobots front robots
  const int nFrontRobots = nBlocker + 1 + nSupporter;
  //VLA(int, id, nFrontRobots);  // blocker + striker + strikerSupporter
  int id[nFrontRobots];
  for (int i = 0; i < nFrontRobots; i++)
    id[i] = -1;
  //VLA(Vector2<double>, robot_pos, nFrontRobots);
  Vector2<double> robot_pos[nFrontRobots];
  int own_id = thePlayerInfo->unum;
  std::vector<OtherRobots::State>::const_iterator ownIter;
  bool selfDone = false;
  ownIter = theOtherRobots->teammates.begin();
  while (ownIter != theOtherRobots->teammates.end())
  {
    Vector2<double> p = ownIter->pose.translation;
    int current_id = ownIter->unum;
    if (!selfDone)
    {
      p = theRobotPose->pose.translation;
      current_id = thePlayerInfo->unum;
    }

    int insert = nFrontRobots;
    while (insert > 0 && (id[insert - 1] == -1 || p.x > robot_pos[insert - 1].x))
      insert--;
    if (insert < nFrontRobots)
    {
      for (int i = nFrontRobots - 2; i >= insert; i--)
      {
        robot_pos[i + 1] = robot_pos[i];
        id[i + 1] = id[i];
      }
      robot_pos[insert] = p;
      id[insert] = current_id;
    }

    if (selfDone)
      ownIter++;
    selfDone = true;
  }
  //select own position in y direction, ignoring the nBlocker robots in front
  int y_order = 0;
  for (int i = nBlocker; i < nFrontRobots; i++)
    if (id[i] != own_id && robot_pos[i].y < my_position.y)
      y_order++;
  //log << "supporter  y_order=" << y_order << std::endl;
  int posIndex = y_order;
  if (posIndex >= ballPos + 2)
    posIndex--;
  return pos[posIndex];
}

int Formation::ownPlayersCloserToPoint(Vector2<double> point)
{
  const Vector2<double> &my_position = theRobotPose->pose.translation;

  float angle_factor = 0.0;

  float my_dist = (point - my_position).abs();
  float my_angle_to_point = (point - my_position).angleToVector(
      Vector2<double>(theFieldDimensions->halfLength + 1.0, 0) - point);
  if (my_angle_to_point > M_PI)
    my_angle_to_point -= M_PI * 2.0;
  float my_value = my_dist * (1.0 + angle_factor * fabs(my_angle_to_point));

  int n = 0;
  //bool lyingRobotNearBall = false;
  std::vector<OtherRobots::State>::const_iterator ownIter;
  for (ownIter = theOtherRobots->teammates.begin(); ownIter != theOtherRobots->teammates.end();
      ++ownIter)
  {
    const Vector2<double> &pos_2d = ownIter->pose.translation;
    float player_dist_to_point = (pos_2d - point).abs();
    float player_angle_to_point = (point - pos_2d).angleToVector(
        Vector2<double>(theFieldDimensions->halfLength + 1.0, 0) - point);
    if (player_angle_to_point > M_PI)
      player_angle_to_point -= M_PI * 2.0;
    float player_value = player_dist_to_point * (1.0 + angle_factor * fabs(player_angle_to_point));

    //extra distance value for lying robots
    /*const float radius_for_second_robot = 0.5;
     const float radius_for_third_robot = 1.5;
     if(pos->get_z() < 0.2
     && pos_2d.distance_to(my_position) < 5
     && !(pos_2d.distance_to(ball_position) < radius_for_second_robot)
     )//&& !(pos_2d.distance_to(ball_position) < radius_for_third_robot
     //   && lyingRobotNearBall))
     {
     player_value += 4.0;
     if(pos_2d.distance_to(ball_position) <= radius_for_third_robot)
     lyingRobotNearBall = true;
     }*/

    //if(ownIter->unum != 1 || (point-Vector2<double>(-theFieldDimensions->halfLength,0)).abs() < 4.0)
    if (player_value < my_value)
      n++;
  }
  return n;
}

