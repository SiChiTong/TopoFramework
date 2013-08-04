/**
 * @file SimpleHeadMotion.h
 *
 * Contains the module SimpleHeadMotion.
 *
 * @author Andreas Seekircher <aseek@cs.miami.edu>
 */
#ifndef SIMPLEHEADMOTION_H
#define SIMPLEHEADMOTION_H

#include "kernel/Template.h"

#include "representations/perception/FrameInfo.h"
#include "representations/perception/BallPercept.h"
#include "representations/modeling/BallPos.h"
#include "representations/modeling/FallState.h"
#include "representations/motion/MotionRequest.h"
#include "representations/motion/HeadMotionRequest.h"


MODULE(SimpleHeadMotion)
  REQUIRES(FrameInfo)
  REQUIRES(BallPercept)
  REQUIRES(BallPos)
  REQUIRES(MotionRequest)
  PROVIDES(HeadMotionRequest)
END_MODULE

/**
 * @class SimpleHeadMotion 
 * 
 * Creates a simple head motion. Depending on the distance to the ball and some
 * hardcoded thresholds the robot looks sometimes to the ball or looks around
 * using sine curves.
 */
class SimpleHeadMotion : public SimpleHeadMotionBase
{
  public:
    
    void update(HeadMotionRequest& theHeadMotionRequest);


  private:
  
    double ball_visibility;

};


#endif

