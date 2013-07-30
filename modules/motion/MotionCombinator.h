#ifndef MOTIONCOMBINATOR_H
#define MOTIONCOMBINATOR_H

#include "kernel/Template.h"

#include "representations/motion/MotionSelection.h"
#include "representations/motion/SpecialActionsOutput.h"
#include "representations/motion/SpecialMotionsOutput.h"
#include "representations/motion/DeadMotionOutput.h"
#include "representations/motion/KickMotionOutput.h"
#include "representations/motion/WalkingEngineOutput.h"
#include "representations/motion/HeadMotionRequest.h"
#include "representations/motion/JointRequest.h"


MODULE(MotionCombinator)
  REQUIRES(MotionSelection)
  REQUIRES(SpecialActionsOutput)
  REQUIRES(SpecialMotionsOutput)
  REQUIRES(DeadMotionOutput)
  REQUIRES(KickMotionOutput)
  REQUIRES(WalkingEngineOutput)
  REQUIRES(HeadMotionRequest)
  PROVIDES(JointRequest)
END_MODULE


class MotionCombinator : public MotionCombinatorBase
{
  public:
    
    void update(JointRequest& theJointRequest);

  private:
  
    bool prevSpecialActionActive;
    bool prevSpecialMotionActive;
  
};


#endif

