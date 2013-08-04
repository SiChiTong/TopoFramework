#ifndef BLOCKOPPONENT_H
#define BLOCKOPPONENT_H


#include "kernel/Template.h"

#include "representations/rcss/FieldDimensions.h"
#include "representations/modeling/RobotPose.h"
#include "representations/modeling/BallPos.h"
#include "representations/modeling/OtherRobots.h"
#include "representations/behavior/BlockPosition.h"


MODULE(BlockOpponent)
  REQUIRES(FieldDimensions)
  REQUIRES(RobotPose)
  REQUIRES(BallPos)
  REQUIRES(OtherRobots)
  PROVIDES(BlockPosition)
END_MODULE


class BlockOpponent : public BlockOpponentBase
{
  public:
  
    void update(BlockPosition& theBlockPosition);

  private:
  
    bool inOppPenaltyArea(const Vector2<double> &v);
    
};


#endif

