#ifndef INTERCEPTPOSITION_H
#define INTERCEPTPOSITION_H

#include "kernel/Template.h"
#include "representations/modeling/OtherRobots.h"
#include "math/Vector2.h"

REPRESENTATION(InterceptPosition)
class InterceptPosition: public InterceptPositionBase
{
  public:

    Vector2<double> own;

    enum
    {
      OTHERS = OtherRobots::MAX_UNUM + 1
    };  // MAX_UNUM+1 for index 0 to 11
    Vector2<double> others[OTHERS];

};

#endif

