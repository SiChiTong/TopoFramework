#ifndef HEADMOTIONREQUEST_H
#define HEADMOTIONREQUEST_H

#include "kernel/Framework.h"

REPRESENTATION(HeadMotionRequest)

class HeadMotionRequest: public HeadMotionRequestBase
{
  public:
    double pan;
    double tilt;

    HeadMotionRequest() :
        pan(0), tilt(0)
    {
    }
};

#endif

