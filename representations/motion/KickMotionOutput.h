#ifndef KICKMOTIONOUTPUT_H
#define KICKMOTIONOUTPUT_H

#include "kernel/Template.h"
#include "common/JointValues.h"

REPRESENTATION(KickMotionOutput)

class KickMotionOutput: public KickMotionOutputBase
{
  public:

    bool active;
    JointValues values;

};

#endif

