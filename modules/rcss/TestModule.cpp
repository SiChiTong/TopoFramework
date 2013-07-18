/*
 * TestModule.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#include "TestModule.h"
#include <sstream>

TestModule::TestModule() :
    mState(S_Wait), mCounter(0)
{
}

void TestModule::update(JointRequestWithSpeeds& theJointRequestWithSpeeds)
{
  // fixMe
  //Action
  std::stringstream ss;
  switch (mState)
  {
  case S_Wait:
    if (mCounter > 50)
    {
      mCounter = 0;
      mState = S_Sit;
      break;
    }
    break;

  case S_Sit:
    if (mCounter > 40)
    {
      mCounter = 0;
      mState = S_SwingLeft;
      ss << "(lae1 0)" << "(rae1 0)" << "(lae2 0)" << "(rae2 0)" << "(lae3 0)" << "(rae3 0)" << "(lae4 0)" << "(rae4 0)"
          << "(lle1 0)" << "(rle1 0)" << "(lle2 0)" << "(rle2 0)" << "(lle3 0)" << "(rle3 0)" << "(lle4 0)"
          << "(rle4 0)" << "(lle5 0)" << "(rle5 0)" << "(lle6 0)" << "(rle6 0)";
      break;
    }
    ss << "(lae1 -1.5)" << "(rae1 -1.5)" << "(lae2 0.9)" << "(rae2 -0.9)" << "(lae3 0)" << "(rae3 0)" << "(lae4 -0.8)"
        << "(rae4 0.8)" << "(lle3 1)" << "(rle3 1)" << "(lle4 -2)" << "(rle4 -2)" << "(lle5 1)" << "(rle5 1)"
        << "(lle6 -0)" << "(rle6 0)";
    break;

  case S_SwingLeft:
    if (mCounter > 50)
    {
      mCounter = 0;
      mState = S_SwingRight;
      break;
    }
    ss << "(he1 -1)";
    break;

  case S_SwingRight:
    if (mCounter > 50)
    {
      mCounter = 0;
      mState = S_SwingLeft;
      break;
    }
    ss << "(he1 1)";
    break;
  }

  mCounter++;

  if (mCounter % 10 == 0)
    ss << "(say ComeOnTopoFramework!)";

  //std::cout << "Sent: " << ss.str() << "\n";
  //theOutputJointValues.msg = ss.str();
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
}

void TestModule::update(SpecialMotionsOutput& theSpecialMotionsOutput)
{
  // fixMe
}

MAKE_MODULE(TestModule)

