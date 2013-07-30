/**
 * @file AccEvaluator.h
 *
 * Defines the module AccEvaluator.
 *
 * @author Andreas Seekircher <aseek@cs.miami.edu>
 * @author Sam Abeyruwan <saminda@cs.miami.edu>
 */

#ifndef ACCEVALUATOR_H
#define ACCEVALUATOR_H

#include "kernel/Template.h"

#include "representations/perception/FrameInfo.h"
#include "representations/perception/SensorData.h"
#include "representations/modeling/FKModel.h"
#include "representations/modeling/FallState.h"
#include "representations/modeling/UprightVec.h"
#include "representations/motion/WalkingEngineOutput.h"
#include "representations/modeling/Odometry.h"
#include "representations/modeling/TorsoPose.h"


MODULE(AccEvaluator)
  REQUIRES(FrameInfo)
  REQUIRES(SensorData)
  REQUIRES(FKModel)
  USES(WalkingEngineOutput)
  PROVIDES(FallState)
  PROVIDES(UprightVec)
  PROVIDES(Odometry)
  PROVIDES(TorsoPose)
END_MODULE

/**
 * @class AccEvaluator 
 * 
 * This module uses the accelerator to create the UprightVec and the 
 * FallState.
 */
class AccEvaluator : public AccEvaluatorBase
{
  public:
  
    void execute();
    
    void update(FallState& theFallState);   /**< update FallState */
    void update(UprightVec& theUprightVec); /**< update UprightVec */

    void update(Odometry& theOdometry);     /**< update Odometry */

    void update(TorsoPose& theTorsoPose);   /**< update TorsoPose */
  
  private:
  
    Vector3<double> vec;
    float angleX, angleY;
    
    ORIENTATION orientation;
    
    long timePrev;
    double odometryRotation;


    long timeLastFall;
};


#endif

