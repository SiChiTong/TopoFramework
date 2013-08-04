#include "DeadMotion.h"

MAKE_MODULE(DeadMotion)

void DeadMotion::update(DeadMotionOutput& theDeadMotionOutput)
{
  theDeadMotionOutput.active = (theMotionRequest->motion == MotionRequest::DEAD);
  if (!theDeadMotionOutput.active)
    return;

  for (int i = 0; i < NUM_JOINT_ID; i++)
  {
    theDeadMotionOutput.values[i].angle = theJointData->values[i].angle;
    theDeadMotionOutput.values[i].hardness = 0;
  }
}

