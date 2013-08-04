#ifndef DEADMOTION_H
#define DEADMOTION_H

#include "kernel/Template.h"

#include "representations/perception/JointData.h"
#include "representations/motion/MotionRequest.h"
#include "representations/motion/DeadMotionOutput.h"

MODULE(DeadMotion)
  REQUIRES(JointData)  
  REQUIRES(MotionRequest)  
  PROVIDES(DeadMotionOutput)
END_MODULE


class DeadMotion : public DeadMotionBase
{
  public:
    
    void update(DeadMotionOutput& theDeadMotionOutput);
      
};


#endif

