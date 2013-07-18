#ifndef FALLSTATE_H
#define FALLSTATE_H

#include "kernel/Framework.h"

#include "math/Polar.h"

REPRESENTATION(FallState)

/**
 * This enum contains different orientations the robot can have.
 */
enum ORIENTATION
{
  ORIENTATION_STANDING,
  ORIENTATION_FALLING_BACK,
  ORIENTATION_LYING_BACK,
  ORIENTATION_FALLING_FORWARD,
  ORIENTATION_LYING_FORWARD,
  ORIENTATION_FALLING_SIDEWAYS,
  ORIENTATION_LYING_SIDEWAYS,
  ORIENTATION_SITTING,
  ORIENTATION_UNKNOWN,
  NUM_ORIENTATION
};

class FallState: public FallStateBase
{
  public:

    bool fallen;
    ORIENTATION orientation;

};

#endif

