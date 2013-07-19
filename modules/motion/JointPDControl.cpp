#include "JointPDControl.h"
#include <cstdio>

MAKE_MODULE(JointPDControl)

void JointPDControl::update(JointRequestWithSpeeds& theJointRequestWithSpeeds)
{
  theJointRequestWithSpeeds.values = theJointRequest->values;
  if (theJointRequest->hasAlreadySpeeds)
    return;

  /** @todo check angle limits of JointRequest */

  // pd control factors
  double pFactor = config.getValue("p_factor", 0.26f);
  double dFactor = config.getValue("d_factor", 0.01f);
  /*p *= pMultiplier;*//**< @todo p factor multiplier, used in kick */

  //get time step
  double timeStep = theFrameInfo->time_ms - lastTimestamp;
  lastTimestamp = theFrameInfo->time_ms;

  //calculate speeds
  if (timeStep > 0.0)
    for (int i = 0; i < NUM_JOINT_ID; i++)
    {
      // get current joint error 
      double error = theJointRequest->values[i].angle - theJointData->values[i].angle;

      // calc control values to get angle velocity
      double p = pFactor * error;
      double d = (dFactor * (error - prevError[i])) / timeStep;
      prevError[i] = error;
      theJointRequestWithSpeeds.values[i].speed = p + d;
    }
}

