#ifndef SPECIALACTIONSOUTPUT_H
#define SPECIALACTIONSOUTPUT_H

#include "kernel/Template.h"
#include "common/JointValues.h"

REPRESENTATION(SpecialActionsOutput)

class SpecialActionsOutput: public SpecialActionsOutputBase
{
  public:

    bool active;
    JointValues values;

};

#endif

