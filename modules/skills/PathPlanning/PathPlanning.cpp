#include "PathPlanning.h"
#include "RRT.h"

MAKE_MODULE(PathPlanning)

PathPlanning::PathPlanning()
{
  rrt = NULL;
}
void PathPlanning::init()
{
  rrt = new RRT::RRT(theFieldDimensions->length, theFieldDimensions->width, drawing);
}
PathPlanning::~PathPlanning()
{
  if (rrt != NULL)
    delete rrt;
}

bool PathPlanning::getRequest()
{
  const int nRequests = 4;
  const MoveToRequest* requests[nRequests] =
  //--- the SkillRequest can directly contain a MoveToRequest:
      { (theSkillRequest->skill == SkillRequest::MOVETOPOS ?
          &(theSkillRequest->moveToRequest) : NULL),
      //--- or other skills outputs:
          (theSkillGetBallOutput->active ? &theSkillGetBallOutput->moveToRequest : NULL), (
              theSkillKickOutput->active ? &theSkillKickOutput->moveToRequest : NULL), (
              theSkillDribbleOutput->active ? &theSkillDribbleOutput->moveToRequest : NULL) };

  //get the active request
  request = NULL;
  for (int i = 0; i < nRequests; i++)
    if (requests[i] != NULL && requests[i]->active)
      request = requests[i];
  return !(request == NULL);
}

RRT::Obstacle*
PathPlanning::createPlayerObstacle(const OtherRobots::State &player, double radius)
{
  double obs_radius = radius; //0.35;
  /** @todo use angle of lying robots, line obstacle from head to feet*/
  if (player.fallen())
    obs_radius = 0.6;
  RRT::Obstacle* newObs = new RRT::Round_Obstacle(player.pose.translation.x,
      player.pose.translation.y, obs_radius);
  newObs->avoid_collision_with_pose(theRobotPose->pose.translation);

  drawing.circle("PathPlanning.obstacles", player.pose.translation.x, player.pose.translation.y,
      obs_radius, 2, 100, 100, 100);

  return newObs;
}

void PathPlanning::createObstacles(std::vector<RRT::Obstacle*> &obstacles)
{
  bool striker = true;
  bool goalie = false;
  if (!thePlayerRole.isNull())
  {
    striker = (thePlayerRole->role == PlayerRole::STRIKER);
    goalie = (thePlayerRole->role == PlayerRole::GOALIE);
  }
  const float radius_for_second_robot = 0.7;
  const float radius_for_third_robot = 1.6;
  const Vector2<double> &mypos = theRobotPose->pose.translation;
  //teammate obstacles
  // keep distances according to rules, even without ball
  std::vector<OtherRobots::State>::const_iterator playerIter;
  for (playerIter = theOtherRobots->teammates.begin();
      playerIter != theOtherRobots->teammates.end(); ++playerIter)
  {
    bool twoAlreadyClose = false;
    if (!striker && !goalie)
    {
      float myDist = (mypos - playerIter->pose.translation).abs();
      std::vector<OtherRobots::State>::const_iterator iter;
      for (iter = theOtherRobots->teammates.begin();
          iter != theOtherRobots->teammates.end() && !twoAlreadyClose; ++iter)
      {
        const float secondPlayerDist =
            (iter->pose.translation - playerIter->pose.translation).abs();
        if (iter->unum != playerIter->unum && secondPlayerDist < radius_for_third_robot
            && secondPlayerDist < myDist)
          twoAlreadyClose = true;
      }
    }
    obstacles.push_back(
        createPlayerObstacle(*playerIter,
            (twoAlreadyClose ? radius_for_third_robot : radius_for_second_robot)));
  }
  //opponent obstacles
  for (playerIter = theOtherRobots->opponents.begin();
      playerIter != theOtherRobots->opponents.end(); ++playerIter)
  {
    obstacles.push_back(createPlayerObstacle(*playerIter, 0.35));
  }
  //ball
  if (request->avoidBallDistance > 0)
  {
    obstacles.push_back(
        new RRT::Round_Obstacle(theBallPos->absPos.x, theBallPos->absPos.y,
            request->avoidBallDistance));
    drawing.circle("PathPlanning.obstacles", theBallPos->absPos.x, theBallPos->absPos.y,
        request->avoidBallDistance, 2, 100, 100, 100);
  }
  //add goal obstacles
  double hl = theFieldDimensions->halfLength;
  double gw = theFieldDimensions->halfGoalWidth;
  double gd = theFieldDimensions->goalDepth;
  double goalOppDist = 0.2;
  obstacles.push_back(new RRT::Line_Obstacle(-hl, gw, -hl - gd, gw, goalOppDist));
  obstacles.push_back(new RRT::Line_Obstacle(-hl, -gw, -hl - gd, -gw, goalOppDist));
  obstacles.push_back(new RRT::Line_Obstacle(-hl - gd, gw, -hl - gd, -gw, goalOppDist));
  obstacles.push_back(new RRT::Line_Obstacle(hl, gw, hl + gd, gw, goalOppDist));
  obstacles.push_back(new RRT::Line_Obstacle(hl, -gw, hl + gd, -gw, goalOppDist));
  obstacles.push_back(new RRT::Line_Obstacle(hl + gd, gw, hl + gd, -gw, goalOppDist));
  //add areas where the robot would be beamed out
  if (theGamestate->playmode == Gamestate::PLAYON && !goalie)   // goalie ignores this
  {
    //in playOn count robots closer to ball
    double dist = (mypos - theBallPos->absPos).abs();
    int closerTeammates = 0;
    for (playerIter = theOtherRobots->teammates.begin();
        playerIter != theOtherRobots->teammates.end(); ++playerIter)
    {
      if ((playerIter->pose.translation - theBallPos->absPos).abs() < dist)
        closerTeammates++;
    }
    //if ==1 avoid small area
    if (closerTeammates == 1 || (closerTeammates > 1 && striker)) // for striker only smaller dist
      obstacles.push_back(
          new RRT::Round_Obstacle(theBallPos->absPos.x, theBallPos->absPos.y,
              radius_for_second_robot));
    //if  >1 avoid larger area
    else if (closerTeammates > 1)
      obstacles.push_back(
          new RRT::Round_Obstacle(theBallPos->absPos.x, theBallPos->absPos.y,
              radius_for_third_robot));
  }
  //not more than 3 other players in own penalty area
  if (!goalie)
  {
    int inPenaltyArea = 0;
    for (playerIter = theOtherRobots->teammates.begin();
        playerIter != theOtherRobots->teammates.end(); ++playerIter)
    {
      if (fabs(playerIter->pose.translation.y) < theFieldDimensions->halfPenaltyAreaWidth
          && playerIter->pose.translation.x
              < -theFieldDimensions->halfLength + theFieldDimensions->penaltyAreaDepth)
        inPenaltyArea++;
    }
    //own penalty area
    const Vector2<double> &target = request->target.translation;
    if (inPenaltyArea >= 3
        || (theGamestate->playmode == Gamestate::GOALKICK_OWN
            && target.x > -theFieldDimensions->halfLength + theFieldDimensions->penaltyAreaDepth))
    {
      double x1 = -theFieldDimensions->halfLength;
      double x2 = x1 + theFieldDimensions->penaltyAreaDepth;
      double y1 = theFieldDimensions->halfPenaltyAreaWidth;
      double y2 = -y1;
      double dist = 0.2;
      obstacles.push_back(new RRT::Line_Obstacle(x1, y1, x2, y1, dist));
      obstacles.push_back(new RRT::Line_Obstacle(x2, y1, x2, y2, dist));
      obstacles.push_back(new RRT::Line_Obstacle(x2, y2, x1, y2, dist));
      obstacles.push_back(new RRT::Line_Obstacle(x1, y2, x1, y1, dist));
    }
  }
  //opp goalkick, opp penalty area  
  if (theGamestate->playmode == Gamestate::GOALKICK_OPP)
  {
    double x1 = theFieldDimensions->halfLength;
    double x2 = x1 - theFieldDimensions->penaltyAreaDepth;
    double y1 = theFieldDimensions->halfPenaltyAreaWidth;
    double y2 = -y1;
    double dist = 0.2;
    obstacles.push_back(new RRT::Line_Obstacle(x1, y1, x2, y1, dist));
    obstacles.push_back(new RRT::Line_Obstacle(x2, y1, x2, y2, dist));
    obstacles.push_back(new RRT::Line_Obstacle(x2, y2, x1, y2, dist));
    obstacles.push_back(new RRT::Line_Obstacle(x1, y2, x1, y1, dist));
  }
  //various playmodes, ball distance
  if (theGamestate->playmode == Gamestate::FREEKICK_OPP
      || theGamestate->playmode == Gamestate::KICKOFF_OPP
      || theGamestate->playmode == Gamestate::KICKIN_OPP
      || theGamestate->playmode == Gamestate::CORNERKICK_OPP)
  {
    obstacles.push_back(
        new RRT::Round_Obstacle(theBallPos->absPos.x, theBallPos->absPos.y,
            theFieldDimensions->freeKickMoveDist + 0.2));
  }
  else if (!striker && !goalie)
  {
    obstacles.push_back(
        new RRT::Round_Obstacle(theBallPos->absPos.x, theBallPos->absPos.y,
            radius_for_second_robot + 0.1));
  }
}

void PathPlanning::execute()
{
  if (!getRequest())
    return;
  const Vector2<double> &mypos = theRobotPose->pose.translation;
  Vector2<double> target = request->target.translation;

  //limit distance to target position
  double maxDistToTarget = 8.0;
  if ((target - mypos).abs() > maxDistToTarget)
    target = mypos + (target - mypos).normalize(maxDistToTarget);

  //create obstacle vector
  std::vector<RRT::Obstacle*> obstacles;
  createObstacles(obstacles);

  //execute rrt
  Vector2<double> rrtResult = rrt->createPath(mypos, target, &obstacles);
  walkVec = rrtResult - mypos;

  //draw path
#ifdef DEBUG_MODE
  std::vector<Vector2<double> >::iterator path_iter = rrt->path.begin();
  for(; path_iter<rrt->path.end(); path_iter++)
  drawing.circle("PathPlanning.path",
      path_iter->x,path_iter->y,0.05, 2, 0,0,255);
#endif

  //cleanup obstacle vector
  std::vector<RRT::Obstacle*>::iterator iter = obstacles.begin();
  for (; iter < obstacles.end(); iter++)
    delete *iter;
  obstacles.clear();

}

void PathPlanning::update(SafeWalkDirection& theSafeWalkDirection)
{
  if (!(theSafeWalkDirection.active = (request != NULL)))
    return;

  const Vector2<double> &mypos = theRobotPose->pose.translation;
  Vector2<double> target = request->target.translation;

  //limit distance to target position
  double maxDistToTarget = 8.0;
  if ((target - mypos).abs() > maxDistToTarget)
    target = mypos + (target - mypos).normalize(maxDistToTarget);

  theSafeWalkDirection.moveToRequest = *request;
  theSafeWalkDirection.moveToRequest.target = target;
  theSafeWalkDirection.vec = walkVec;
}

