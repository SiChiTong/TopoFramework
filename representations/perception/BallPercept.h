#ifndef BALLPERCEPT_H
#define BALLPERCEPT_H

#include "kernel/Template.h"

#include "math/Polar.h"
#include "math/Matrix.h"


REPRESENTATION(BallPercept)


class BallPercept : public BallPerceptBase
{
  public:

    bool updated;      
    Polar polar; // Not streamable
    Vector3<double> pos; // Position on the ground plane
};


#endif

