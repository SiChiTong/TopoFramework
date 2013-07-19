/*
 * TestModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef TESTMODULE_H_
#define TESTMODULE_H_

#include "kernel/Framework.h"
#include "representations/rcss/SayMessage.h"
#include "representations/rcss/BeamRequest.h"
#include "representations/motion/MotionRequest.h"
#include "representations/motion/SpecialActionsOutput.h"
#include "representations/motion/SpecialMotionsOutput.h"
#include "representations/motion/DeadMotionOutput.h"
#include "representations/motion/KickMotionOutput.h"
#include "representations/motion/HeadMotionRequest.h"

MODULE(TestModule)
  PROVIDES(SayMessage)
  PROVIDES(BeamRequest)
  PROVIDES(MotionRequest)
  PROVIDES(SpecialActionsOutput)
  PROVIDES(SpecialMotionsOutput)
  PROVIDES(DeadMotionOutput)
  PROVIDES(KickMotionOutput)
  PROVIDES(HeadMotionRequest)
END_MODULE

class TestModule: public TestModuleBase
{
  public:
    TestModule();
    void update(SayMessage& theSayMessage);
    void update(BeamRequest& theBeamRequest);
    void update(MotionRequest& theMotionRequest);
    void update(SpecialActionsOutput& theSpecialActionsOutput);
    void update(SpecialMotionsOutput& theSpecialMotionsOutput);
    void update(DeadMotionOutput& theDeadMotionOutput);
    void update(KickMotionOutput& theKickMotionOutput);
    void update(HeadMotionRequest& theHeadMotionRequest);
};

#endif /* TESTMODULE_H_ */
