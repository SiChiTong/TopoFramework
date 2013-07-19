#ifndef DEADMOTIONOUTPUT_H
#define DEADMOTIONOUTPUT_H

#include "kernel/Framework.h"
#include "common/JointValues.h"

REPRESENTATION(DeadMotionOutput)

class DeadMotionOutput: public DeadMotionOutputBase
{
  public:

    bool active;
    JointValues values;
};

#endif

