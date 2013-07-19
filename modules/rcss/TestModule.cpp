/*
 * TestModule.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#include "TestModule.h"
#include <sstream>

TestModule::TestModule()
{
}

void TestModule::update(SayMessage& theSayMessage)
{
  // fixMe
}

void TestModule::update(BeamRequest& theBeamRequest)
{
  // fixMe
}

void TestModule::update(MotionRequest& theMotionRequest)
{
  // fixMe
}

void TestModule::update(SpecialActionsOutput& theSpecialActionsOutput)
{
  // fixMe
  theSpecialActionsOutput.active = true; // Default behavior
}

void TestModule::update(SpecialMotionsOutput& theSpecialMotionsOutput)
{
  // fixMe
  theSpecialMotionsOutput.active = false;
}

void TestModule::update(DeadMotionOutput& theDeadMotionOutput)
{
  // fixMe
  theDeadMotionOutput.active = false;
}

void TestModule::update(KickMotionOutput& theKickMotionOutput)
{
  // fixMe
  theKickMotionOutput.active = false;
}

void TestModule::update(HeadMotionRequest& theHeadMotionRequest)
{
  // fixMe
}

MAKE_MODULE(TestModule)

