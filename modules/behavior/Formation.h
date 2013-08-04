#ifndef FORMATION_H
#define FORMATION_H


#include "kernel/Template.h"

#include "representations/rcss/FieldDimensions.h"
#include "representations/perception/FrameInfo.h"
#include "representations/perception/Gamestate.h"
#include "representations/perception/PlayerInfo.h"
#include "representations/modeling/RobotPose.h"
#include "representations/modeling/BallPos.h"
#include "representations/modeling/OtherRobots.h"
#include "representations/behavior/PlayerRole.h"
#include "representations/behavior/BlockPosition.h"


MODULE(Formation)
  REQUIRES(FrameInfo)
  REQUIRES(FieldDimensions)
  REQUIRES(Gamestate)
  REQUIRES(PlayerInfo)
  REQUIRES(RobotPose)
  REQUIRES(BallPos)
  REQUIRES(OtherRobots)
  REQUIRES(BlockPosition)
  PROVIDES(PlayerRole)
END_MODULE


class Formation : public FormationBase
{
  public:
  
    void init();
    void execute();
    void update(PlayerRole& thePlayerRole);

  private:
    
    //which player is the striker
    bool closest_to_ball(int skip);
    float get_walk_cost(Vector2<double> robot, Vector2<double> dest);
    Vector2<double> get_intercept_pos(Vector2<double> robot,
                          Vector2<double> point, Vector2<double> point_speed);

    //positioning of others
    Pose2D supporterAndDefendersSimple();
    bool inPenaltyArea(const Vector2<double> &v);
    int ownPlayersCloserToPoint(Vector2<double> point);
    Vector2<double> getSupporterPosition(Vector2<double> ball_position);
    
    int nBlocker;
    
    int confNumBlocker;
    int confNumStrikerSupporter;
    
    
    int timeLastKickoff;
};


#endif

