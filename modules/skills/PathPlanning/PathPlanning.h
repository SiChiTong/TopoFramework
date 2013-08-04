/**
 * @file PathPlanning.h
 *
 * Contains the module for RRT path planning.
 *
 * @author Andreas Seekircher <aseek@cs.miami.edu>
 */
#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include "kernel/Template.h"

#include "representations/rcss/FieldDimensions.h"
#include "representations/perception/Gamestate.h"
#include "representations/behavior/PlayerRole.h"
#include "representations/modeling/RobotPose.h"
#include "representations/modeling/BallPos.h"
#include "representations/modeling/OtherRobots.h"
#include "representations/skills/SkillRequest.h"
#include "representations/skills/SkillOutputs.h"
#include "representations/skills/SafeWalkDirection.h"

namespace RRT
{
  class RRT;
  class Obstacle;
};

MODULE(PathPlanning)
  //direct MOVETOPOSE request
  REQUIRES(SkillRequest)
  //skills that could use the MoveToPos skill
  REQUIRES(SkillGetBallOutput)
  REQUIRES(SkillKickOutput)
  REQUIRES(SkillDribbleOutput)

  REQUIRES(Gamestate)
  USES(PlayerRole)
  REQUIRES(RobotPose)
  REQUIRES(BallPos)
  REQUIRES(OtherRobots)
  REQUIRES(FieldDimensions)
  
  PROVIDES(SafeWalkDirection)
END_MODULE

/**
 * @class PathPlanning
 */
class PathPlanning : public PathPlanningBase
{
  public:
    PathPlanning();
    ~PathPlanning();
    void init();
    
    void execute();
    void update(SafeWalkDirection& theSafeWalkDirection);


  private:
  
    bool getRequest();
    const MoveToRequest *request;
    
    RRT::Obstacle* createPlayerObstacle(const OtherRobots::State &player, double radius);
    void createObstacles(std::vector<RRT::Obstacle*> &obstacles);
    
    RRT::RRT *rrt;
    Vector2<double> walkVec;
};


#endif

