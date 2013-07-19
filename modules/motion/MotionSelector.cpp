#include "MotionSelector.h"

MAKE_MODULE(MotionSelector)

void MotionSelector::update(MotionSelection& theMotionSelection)
{
  //set all ratios to 0
  for (int i = 0; i < MotionRequest::NUM_MOTIONS; i++)
    theMotionSelection.ratios[i] = 0.0f;

  //if not calibrated, stand (unless standup is needed)
  /*if(theMotionRequest->motion != MotionRequest::DEAD
   && !theFallState->fallen && !theTorsoAngles->calibrated)
   {
   theMotionSelection->targetMotion = MotionRequest::STAND;
   theMotionSelection->ratios[MotionRequest::STAND] = 1.0f;
   return;
   }*/

  //select requested motion (no interpolation yet)  
  theMotionSelection.targetMotion = theMotionRequest->motion;
  theMotionSelection.ratios[theMotionRequest->motion] = 1.0f;

}

