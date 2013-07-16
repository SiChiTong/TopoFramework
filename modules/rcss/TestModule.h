/*
 * TestModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef TESTMODULE_H_
#define TESTMODULE_H_

#include "kernel/Framework.h"
#include "representations/perception/JointData.h"
#include "representations/motion/JointRequestWithSpeeds.h"
#include "representations/rcss/SayMessage.h"
#include "representations/rcss/BeamRequest.h"

MODULE(TestModule)
  REQUIRES(JointData)
  PROVIDES(JointRequestWithSpeeds)
  PROVIDES(SayMessage)
  PROVIDES(BeamRequest)
END_MODULE

class TestModule: public TestModuleBase
{
  public:
    TestModule();
    void update(JointRequestWithSpeeds& theJointRequestWithSpeeds);
    void update(SayMessage& theSayMessage);
    void update(BeamRequest& theBeamRequest);

  protected:
    enum eState
    {
      S_Wait, S_Sit, S_SwingLeft, S_SwingRight,
    } mState;

    int mCounter;


};

#endif /* TESTMODULE_H_ */
