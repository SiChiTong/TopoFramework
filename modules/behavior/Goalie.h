#ifndef GOALIE_H
#define GOALIE_H


#include "kernel/Template.h"

#include "BehaviorTools.h"

#include "representations/perception/FrameInfo.h"
#include "representations/perception/PlayerInfo.h"
#include "representations/perception/Gamestate.h"
#include "representations/modeling/FallState.h"
#include "representations/modeling/RobotPose.h"
#include "representations/modeling/BallPos.h"
#include "representations/modeling/OtherRobots.h"
#include "representations/rcss/FieldDimensions.h"
#include "representations/behavior/PlayerRole.h"
#include "representations/behavior/BehaviorOutputs.h"


MODULE(Goalie)
  REQUIRES(Gamestate)
  REQUIRES(FieldDimensions)
  REQUIRES(PlayerInfo)
  REQUIRES(PlayerRole)
  REQUIRES(FallState)
  REQUIRES(RobotPose)
  REQUIRES(BallPos)
  REQUIRES(OtherRobots)
  PROVIDES(GoalieOutput)
END_MODULE


class Goalie: public GoalieBase
{
  public:

    void init();
    void update(GoalieOutput& output);

  private:

    BehaviorTools tools;

    bool penaltyMode;
    Pose2D& clipToPenaltyArea(Pose2D &pose);
    bool inPenaltyArea(const Vector2<double> &v);

    bool isBallApproachingGoal(bool &left_side);
    Vector2<double> ball_speed;
    Vector2<double> ball_previous_position;
};


#endif

