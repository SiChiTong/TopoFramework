#ifndef STRIKER_H
#define STRIKER_H


#include "kernel/Template.h"

#include "BehaviorTools.h"

#include "representations/perception/Gamestate.h"
#include "representations/perception/FrameInfo.h"
#include "representations/modeling/FallState.h"
#include "representations/modeling/RobotPose.h"
#include "representations/modeling/BallPos.h"
#include "representations/modeling/OtherRobots.h"
#include "representations/rcss/FieldDimensions.h"
#include "representations/behavior/PlayerRole.h"
#include "representations/behavior/BehaviorOutputs.h"


MODULE(Striker)
  REQUIRES(FieldDimensions)
  REQUIRES(Gamestate)
  REQUIRES(PlayerRole)
  REQUIRES(FrameInfo)
  REQUIRES(FallState)
  REQUIRES(RobotPose)
  REQUIRES(BallPos)
  REQUIRES(LocalOtherRobots)
  PROVIDES(StrikerOutput)
END_MODULE


class Striker : public StrikerBase
{
  public:
  
    void update(StrikerOutput& output);
  
  private:
  
    BehaviorTools tools;
 
    bool state_kick;
    bool pass;
    long time_target_update;
    Vector2<double> old_dribble_vec;
    
    Vector2<double> current_pass_target;
};


#endif

