#ifndef BELIEFCOMBINATOR_H
#define BELIEFCOMBINATOR_H

#include "kernel/Template.h"

#include "representations/modeling/RobotPose.h"
#include "representations/modeling/BallPos.h"
#include "representations/modeling/OtherRobots.h"

#include <vector>

MODULE(BeliefCombinator)

  REQUIRES(LocalRobotPose)
  REQUIRES(LocalBallPos)
  REQUIRES(LocalOtherRobots)
  
  REQUIRES(TeamRobotPose)
  REQUIRES(TeamBallPos)
  REQUIRES(TeamOtherRobots)

  //PROVIDES(RobotPose)
  PROVIDES(BallPos)
  //PROVIDES(OtherRobots)

END_MODULE


class BeliefCombinator : public BeliefCombinatorBase
{
  public:
      
    void execute();

    void update(RobotPose& theRobotPose);
    void update(BallPos& theBallPos);
    void update(OtherRobots& theOtherRobots);
  
  private:
  
    const RobotPoseData *robotPose;
};


#endif

