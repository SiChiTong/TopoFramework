#ifndef MOVETOREQUEST_H
#define MOVETOREQUEST_H

#include "kernel/Template.h"
#include "math/Pose2D.h"

class MoveToRequest
{
  public:

    bool active;
    Pose2D target;

    float tolerance;
    bool avoidObstacles;
    float avoidBallDistance;

    MoveToRequest() :
        active(false), tolerance(0), avoidObstacles(false), avoidBallDistance(-1)
    {
    }

    void setValues(const Pose2D &target, const float avoidBallDistance = -1,
        const bool avoidObstacles = true, const float tolerance = 0.0f)
    {
      active = true;
      this->target = target;
      this->avoidBallDistance = avoidBallDistance;
      this->avoidObstacles = avoidObstacles;
      this->tolerance = tolerance;
    }
};

#endif

