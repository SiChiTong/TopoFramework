#include "MotionCombinator.h"
#include <cstdio>

MAKE_MODULE(MotionCombinator)

void MotionCombinator::update(JointRequest& theJointRequest)
{
  theJointRequest.hasAlreadySpeeds = false; //usually only angles are set

  bool specialActionActive = false;
  bool specialMotionActive = false;

  //TODO move priority checks (e.g. don't stop a running SpecialAction) into MotionSelector!

  //TODO interpolate using ratios from MotionSelection

  //use joint angles of DeadMotion
  if (theMotionSelection->targetMotion == MotionRequest::DEAD)
  {
    if (!theDeadMotionOutput->active)
      log << "ERROR DeadMotion selected, but output not active!\n";
    theJointRequest.values = theDeadMotionOutput->values;
    log << "motion from DeadMotion" << std::endl;
  }

  //use joint speeds of KickMotion
  //else if(theMotionSelection->targetMotion >= MotionRequest::KICK_LEFT
  //     && theMotionSelection->targetMotion <= MotionRequest::KICK_RIGHT_TO_LEFT)  
  else if (!prevSpecialActionActive && !prevSpecialMotionActive && theKickMotionOutput->active)
  {
    if (!theKickMotionOutput->active)
      log << "ERROR motion Kick selected, but output not active!\n";
    theJointRequest.values = theKickMotionOutput->values;
    theJointRequest.hasAlreadySpeeds = true; //skip joint PD-control
    log << "motion from KickMotion" << std::endl;
  }

  //use joint angles of SpecialActions
  else if (theSpecialActionsOutput->active && (!theSpecialMotionsOutput->active || !prevSpecialMotionActive))
  {
    specialActionActive = true;
    theJointRequest.values = theSpecialActionsOutput->values;
    log << "motion from SpecialActions" << std::endl;
  }

#ifndef TARGET_NAO
  //use joint speeds of SpecialMotions
  else if (theSpecialMotionsOutput->active)
  {
    specialMotionActive = true;
    theJointRequest.values = theSpecialMotionsOutput->values;
    theJointRequest.hasAlreadySpeeds = true; //skip joint PD-control
    log << "motion from SpecialMotions" << std::endl;
  }
#endif

  //angles from WalkingEngine
  else if ((theMotionSelection->targetMotion == MotionRequest::WALK
      || theMotionSelection->targetMotion == MotionRequest::STAND) && theWalkingEngineOutput->active)
  {
    if (!theWalkingEngineOutput->active)
      log << "ERROR motion WALK selected, but output not active!\n";
    theJointRequest.values = theWalkingEngineOutput->values;
    log << "motion from WalkingEngine" << std::endl;
  }

  //error
  else
    log << "WARNING: No motion module active! No Joint angles set!" << std::endl;

  prevSpecialActionActive = specialActionActive;
  prevSpecialMotionActive = specialMotionActive;

  //set head angles
  theJointRequest.values[JID_HEAD_PAN].angle = theHeadMotionRequest->pan;
  theJointRequest.values[JID_HEAD_TILT].angle = theHeadMotionRequest->tilt;
}

