/*
 * TestModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef TESTMODULE_H_
#define TESTMODULE_H_

#include "kernel/Framework.h"

#include "representations/perception/FrameInfo.h"
#include "representations/perception/PlayerInfo.h"
#include "representations/perception/Gamestate.h"
#include "representations/perception/HearMessage.h"
#include "representations/modeling/FallState.h"

#include "representations/rcss/SayMessage.h"
#include "representations/rcss/BeamRequest.h"
#include "representations/motion/MotionRequest.h"
#include "representations/motion/SpecialMotionsOutput.h"
#include "representations/motion/DeadMotionOutput.h"
#include "representations/motion/KickMotionOutput.h"
#include "representations/motion/HeadMotionRequest.h"

#include "math/Pose2D.h"

MODULE(TestModule)
  REQUIRES(FrameInfo)
  REQUIRES(PlayerInfo)
  REQUIRES(Gamestate)
  REQUIRES(HearMessage)
  REQUIRES(FallState)
  // Only for testing purposes
  PROVIDES(SayMessage)
  PROVIDES(BeamRequest)
  PROVIDES(MotionRequest)
  PROVIDES(SpecialMotionsOutput)
  PROVIDES(DeadMotionOutput)
  PROVIDES(KickMotionOutput)
  PROVIDES(HeadMotionRequest)
END_MODULE

class TestModule: public TestModuleBase
{
  public:
    TestModule();

    void execute();
    void update(SayMessage& theSayMessage);
    void update(BeamRequest& theBeamRequest);
    void update(MotionRequest& theMotionRequest);
    void update(SpecialMotionsOutput& theSpecialMotionsOutput);
    void update(DeadMotionOutput& theDeadMotionOutput);
    void update(KickMotionOutput& theKickMotionOutput);
    void update(HeadMotionRequest& theHeadMotionRequest);

  private:
    uint32_t lastTime;
};

#endif /* TESTMODULE_H_ */
