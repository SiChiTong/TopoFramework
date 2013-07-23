#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "kernel/Framework.h"
#include "math/Pose2D.h"

REPRESENTATION(Odometry)

class Odometry: public OdometryBase
{
  public:

    Odometry() :
        pose(0, 0, 0)
    {
    }

    Pose2D pose;

    /** Draws the 2d pose. */
    void draw() const
    {
      drawing.pose("Odometry", pose, 0.18, 1, 0,0,0);
    }

};

#endif

