/*
 * TestModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef TESTMODULE_H_
#define TESTMODULE_H_

#include "kernel/Framework.h"
#include "representations/motion/JointRequestWithSpeeds.h"
#include "representations/rcss/SayMessage.h"
#include "representations/rcss/BeamRequest.h"
#include "representations/motion/MotionRequest.h"
#include "representations/motion/SpecialActionsOutput.h"
#include "representations/motion/SpecialMotionsOutput.h"

MODULE(TestModule)
  PROVIDES(JointRequestWithSpeeds)
  PROVIDES(SayMessage)
  PROVIDES(BeamRequest)
  PROVIDES(MotionRequest)
  PROVIDES(SpecialActionsOutput)
  PROVIDES(SpecialMotionsOutput)
END_MODULE

class TestModule: public TestModuleBase
{
  public:
    TestModule();
    void update(JointRequestWithSpeeds& theJointRequestWithSpeeds);
    void update(SayMessage& theSayMessage);
    void update(BeamRequest& theBeamRequest);
    void update(MotionRequest& theMotionRequest);
    void update(SpecialActionsOutput& theSpecialActionsOutput);
    void update(SpecialMotionsOutput& theSpecialMotionsOutput);

  protected:
    enum eState
    {
      S_Wait, S_Sit, S_SwingLeft, S_SwingRight,
    } mState;

    int mCounter;


};

#endif /* TESTMODULE_H_ */
