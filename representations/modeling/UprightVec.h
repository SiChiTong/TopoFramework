/**
 * @file UprightVector.h
 * Contains the UprightVector representation.
 */
#ifndef UPRIGHTVEC_H
#define UPRIGHTVEC_H

#include "kernel/Framework.h"
#include "math/Vector3.h"

REPRESENTATION(UprightVec)

/** 
 * @class UprightVect
 * The representation that stores the upright vector.
 * 
 * @author Andreas Seekircher <aseek@cs.miami.edu>
 * @author Sam Abeyruwan <saminda@cs.miami.edu>
 */
class UprightVec: public UprightVecBase
{
  public:

    Vector3<double> vec; /**< The upright vector. */

    /** Draws the upright vector vec. */
    void draw() const
    {
      Vector3<double> lineUp = vec;
      lineUp.normalize(1.0);
      //debug.drawing.line("UprightVec",
      //        0,0,0,
      //        lineUp.x, lineUp.y, lineUp.z,
      //        20,20,20, 2,  Drawing::ROBOTPOSE);
    }

};

#endif

