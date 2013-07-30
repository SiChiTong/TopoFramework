#ifndef MOTIONSELECTION_H
#define MOTIONSELECTION_H

#include "kernel/Template.h"
#include "representations/motion/MotionRequest.h"

REPRESENTATION(MotionSelection)

class MotionSelection: public MotionSelectionBase
{
  public:

    MotionRequest::Motions targetMotion; /**< The motion that is the destination of the current interpolation. */
    float ratios[MotionRequest::NUM_MOTIONS]; /**< The current ratio of each motion in the final joint request. */

};

#endif

