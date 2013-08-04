#ifndef SAFEWALKDIRECTION_H
#define SAFEWALKDIRECTION_H

#include "kernel/Template.h"
#include "representations/skills/MoveToRequest.h"
#include "math/Vector2.h"

REPRESENTATION(SafeWalkDirection)

class SafeWalkDirection: public SafeWalkDirectionBase
{
  public:

    /** The current MoveToRequest. */
    MoveToRequest moveToRequest; //This is needed because different skills can set their own moveToRequest.

    /** A safe walk direction (orientation relative to field). */
    Vector2<double> vec;

    /** If true, the MoveToPos skill uses vec to create a MotionRequest. */
    bool active;

    SafeWalkDirection() : active(false) {}

    void draw() const
    {
      if (active)
        drawing.pose("SafeWalkDirection.moveToRequest.target", moveToRequest.target, 0.15, 2, 150,
            200, 150);
    }
};

#endif

