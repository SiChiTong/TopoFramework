#ifndef GOALPERCEPT_H
#define GOALPERCEPT_H

#include "kernel/Template.h"
#include <map>
#include <vector>
#include "math/Polar.h"
#include "math/Vector3.h"

REPRESENTATION(GoalPercept)

enum GOALPOST_ID
{
  GOALPOST_FOE_LEFT, GOALPOST_FOE_RIGHT, GOALPOST_OWN_LEFT, GOALPOST_OWN_RIGHT
};

class GoalPercept: public GoalPerceptBase
{
  public:
    std::map<GOALPOST_ID, Polar> goalposts;
};

#endif

