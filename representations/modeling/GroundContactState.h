/**
 * @file GroundContactState.h
 * Declaration of class GroundContactState.
 * @author Colin Graf
 */

#pragma once

#include "kernel/Template.h"

/**
 * @class GroundContactState
 * Describes whether we got contact with ground or not.
 */
REPRESENTATION(GroundContactState)
class GroundContactState: public GroundContactStateBase
{
  public:
    /** Default constructor. */
    GroundContactState() :
        contact(true)
    {
    }

    bool contact; /**< a foot of the robot touches the ground */

};
