#ifndef ACTIVEFOOT_H
#define ACTIVEFOOT_H

#include "kernel/Template.h"
#include "math/Vector2.h"

REPRESENTATION(ActiveFoot)
class ActiveFoot: public ActiveFootBase
{
  public:

    bool left;
    Vector2<double> offset;

};

#endif

