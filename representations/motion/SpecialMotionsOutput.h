#ifndef SPECIALMOTIONSOUTPUT_H
#define SPECIALMOTIONSOUTPUT_H

#include "kernel/Template.h"
#include "common/JointValues.h"

REPRESENTATION(SpecialMotionsOutput)

class SpecialMotionsOutput: public SpecialMotionsOutputBase
{
  public:

    bool active;
    JointValues values;

};

#endif

