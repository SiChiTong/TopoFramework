#include "KickMotion.h"

MAKE_MODULE(KickMotion)

void KickMotion::init()
{
}

void KickMotion::update(KickMotionOutput& theKickMotionOutput)
{
  bool kicking = ((theFrameInfo->time_ms - time_kick_started) < 50);

  //check if kick requested or kick already running
#ifdef TARGET_NAO
  theKickMotionOutput->active = false; // do not use these kicks on the nao
#else
  theKickMotionOutput.active = kicking
      || (theMotionRequest->motion == MotionRequest::KICK_LEFT
          || theMotionRequest->motion == MotionRequest::KICK_LEFT_45
          || theMotionRequest->motion == MotionRequest::KICK_LEFT_90
          || theMotionRequest->motion == MotionRequest::KICK_LEFT_TO_RIGHT
          || theMotionRequest->motion == MotionRequest::KICK_RIGHT
          || theMotionRequest->motion == MotionRequest::KICK_RIGHT_45
          || theMotionRequest->motion == MotionRequest::KICK_RIGHT_90
          || theMotionRequest->motion == MotionRequest::KICK_RIGHT_TO_LEFT);
#endif
  if (!theKickMotionOutput.active)
    return;

  //if kick was not already running, set type
  if (!kicking)
  {
    time_kick_started = theFrameInfo->time_ms;
    kick_type = theMotionRequest->motion;
    kickSpeed = theMotionRequest->kickSpeed;

    //log << "kick type " << kick_type << std::endl;
  }
  else
  {
    //log << "kick continued " << kick_type << std::endl;
  }
  //Vector2<double> ball_relative = theBallPos->relativePos;
  JointValues &output = theKickMotionOutput.values;

  //set all speeds to 0 first
  for (int i = 0; i < NUM_JOINT_ID; i++)
  {
    output[i].hardness = 0;  // This module should not run on the nao. Hardness 0 to avoid damage.
    output[i].speed = 0;
  }

  //set speeds for the required joints
  if (kick_type == MotionRequest::KICK_LEFT || kick_type == MotionRequest::KICK_LEFT_45
      || kick_type == MotionRequest::KICK_LEFT_90 || kick_type == MotionRequest::KICK_LEFT_TO_RIGHT)
  {
    output[JID_ARM_LEFT0].speed = -1; //1
    output[JID_ARM_RIGHT0].speed = -1; //+1 both arms to front, -1 this back
    //hip forward
    if (kick_type != MotionRequest::KICK_LEFT_90)
    {
      output[JID_LEG_LEFT2].speed = 1;
      output[JID_LEG_RIGHT2].speed = -1;
    }
    //hip side
    if (kick_type == MotionRequest::KICK_LEFT_45 || kick_type == MotionRequest::KICK_LEFT //forward kick
    || kick_type == MotionRequest::KICK_LEFT_90)
    {
      output[JID_LEG_LEFT1].speed = 1;
      output[JID_LEG_RIGHT1].speed = -1;
    }
    else if (kick_type == MotionRequest::KICK_LEFT_TO_RIGHT)
    {
      output[JID_LEG_LEFT1].speed = -1;
      output[JID_LEG_RIGHT1].speed = 0;
    }
    //knee
    //output[JID_LEG_LEFT3].speed = 1;
    //output[JID_LEG_RIGHT3].speed = 1;
    //foot
    //output[JID_LEG_LEFT4].speed = 1;    
    output[JID_LEG_RIGHT4].speed = 1;
  }
  if (kick_type == MotionRequest::KICK_RIGHT || kick_type == MotionRequest::KICK_RIGHT_45
      || kick_type == MotionRequest::KICK_RIGHT_90
      || kick_type == MotionRequest::KICK_RIGHT_TO_LEFT)
  {
    output[JID_ARM_RIGHT0].speed = -1;    //1
    output[JID_ARM_LEFT0].speed = -1;    //+1 both arms to front, -1 back
    //hip forward
    if (kick_type != MotionRequest::KICK_RIGHT_90)
    {
      output[JID_LEG_RIGHT2].speed = 1;
      output[JID_LEG_LEFT2].speed = -1;
    }
    //hip side
    if (kick_type == MotionRequest::KICK_RIGHT_45 || kick_type == MotionRequest::KICK_RIGHT //forward kick
    || kick_type == MotionRequest::KICK_RIGHT_90)
    {
      output[JID_LEG_RIGHT1].speed = -1;
      output[JID_LEG_LEFT1].speed = 1;
    }
    else if (kick_type == MotionRequest::KICK_RIGHT_TO_LEFT)
    {
      output[JID_LEG_RIGHT1].speed = 1;
      output[JID_LEG_LEFT1].speed = 0;
    }
    //knee
    //output[JID_LEG_RIGHT3].speed = 1;
    //output[JID_LEG_LEFT3].speed = 1;
    //foot
    //output[JID_LEG_RIGHT4].speed = 1;    
    output[JID_LEG_LEFT4].speed = 1;
  }

  //multiply speeds with kickSpeed
  for (int i = 0; i < NUM_JOINT_ID; i++)
    output[i].speed *= kickSpeed;
  //printf("kickSpeed = %f\n", kickSpeed);
}

