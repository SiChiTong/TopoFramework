#ifndef FLAGPERCEPT_H
#define FLAGPERCEPT_H

#include "kernel/Template.h"
#include <map>
#include "math/Polar.h"

REPRESENTATION(FlagPercept)

enum FLAG_ID
{
  FLAG_FOE_LEFT, FLAG_FOE_RIGHT, FLAG_OWN_LEFT, FLAG_OWN_RIGHT
};

class FlagPercept: public FlagPerceptBase
{
  public:

    std::map<FLAG_ID, Polar> flags;

};

#endif

