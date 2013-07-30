#ifndef BEAMREQUEST_H
#define BEAMREQUEST_H

#include "kernel/Template.h"
#include "math/Pose2D.h"

REPRESENTATION(BeamRequest)

class BeamRequest: public BeamRequestBase
{
  public:

    bool active;
    Pose2D pose;
    BeamRequest() :
        active(false)
    {
    }

};

#endif

