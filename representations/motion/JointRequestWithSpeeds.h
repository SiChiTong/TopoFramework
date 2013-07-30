#ifndef JOINTREQUESTWITHSPEEDS_H
#define JOINTREQUESTWITHSPEEDS_H

#include "kernel/Template.h"
#include "common/JointValues.h"

REPRESENTATION(JointRequestWithSpeeds)

class JointRequestWithSpeeds: public JointRequestWithSpeedsBase
{
  public:

    JointValues values;

};

#endif

