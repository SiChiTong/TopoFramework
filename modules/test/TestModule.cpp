/*
 * TestModule.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#include "TestModule.h"
#include <sstream>

TestModule::TestModule() :
    lastTime(0)
{
}

void TestModule::execute()
{
  /*
   fixMe: Sam
   std::cout << theFrameInfo->time << "  " << theFrameInfo->time_ms << " " << thePlayerInfo->unum
   << " " << theGamestate->playmode << std::endl;
   std::cout << "HearMessages=" << std::endl;
   for (std::vector<HearMessageData>::const_iterator iter = theHearMessage->messages.begin();
   iter != theHearMessage->messages.end(); ++iter)
   {
   const HearMessageData& hearMessageData = *iter;
   std::cout << "\t" << hearMessageData.data << ":" << hearMessageData.self << ":"
   << hearMessageData.time << std::endl;
   }
   */
}

void TestModule::update(SayMessage& theSayMessage)
{
  theSayMessage.active = true;
  std::stringstream ss;
  ss << "msg:" << thePlayerInfo->unum << ":" << theFrameInfo->time_ms;
  theSayMessage.msg = ss.str();
}

void TestModule::update(BeamRequest& theBeamRequest)
{
  // fixMe
}

void TestModule::update(MotionRequest& theMotionRequest)
{
  static float sign = -1.0f;
  if (config.getValue("walkRequest", false))
  {
    theMotionRequest.motion = MotionRequest::WALK;
    Pose2D walkRequest(0, 0, 0);
    walkRequest.translation.x = 900.0f * sign;
    walkRequest.translation.y = 0.0f;
    walkRequest.rotation = 0.0;
    if (theFrameInfo->time_ms - lastTime > 10000)
    {
      sign *= -1.0;
      lastTime = theFrameInfo->time_ms;
    }

    theMotionRequest.walkRequest = walkRequest;
  }
  else
    theMotionRequest.motion = MotionRequest::STAND;
}

void TestModule::update(SpecialActionsOutput& theSpecialActionsOutput)
{
  if (config.getValue("walkRequest", false))
    theSpecialActionsOutput.active = false;
  else
    theSpecialActionsOutput.active = true;
}

void TestModule::update(SpecialMotionsOutput& theSpecialMotionsOutput)
{
  theSpecialMotionsOutput.active = false;
}

void TestModule::update(DeadMotionOutput& theDeadMotionOutput)
{
  theDeadMotionOutput.active = false;
}

void TestModule::update(KickMotionOutput& theKickMotionOutput)
{
  theKickMotionOutput.active = false;
}

void TestModule::update(HeadMotionRequest& theHeadMotionRequest)
{
  // fixMe
}

MAKE_MODULE(TestModule)

