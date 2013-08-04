#ifndef FOOTSELECTOR_H
#define FOOTSELECTOR_H

#include "kernel/Template.h"

#include "representations/modeling/RobotPose.h"
#include "representations/modeling/BallPos.h"
#include "representations/modeling/OtherRobots.h"
#include "representations/skills/SkillRequest.h"
#include "representations/skills/ActiveFoot.h"


MODULE(FootSelector)
  REQUIRES(RobotPose)
  REQUIRES(BallPos)
  REQUIRES(LocalOtherRobots)
  REQUIRES(SkillRequest)
  PROVIDES(ActiveFoot)
END_MODULE


class FootSelector : public FootSelectorBase
{
  public:
    
    void update(ActiveFoot& theActiveFoot);
  
  private:
  
    bool left;
    
    bool footFromOpp;
};


#endif

