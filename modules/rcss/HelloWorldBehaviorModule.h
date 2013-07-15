/*
 * HelloWorldBehaviorModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef HELLOWORLDBEHAVIORMODULE_H_
#define HELLOWORLDBEHAVIORMODULE_H_

#include "kernel/Framework.h"
#include "representations/perception/JointData.h"
#include "representations/motion/JointRequestWithSpeeds.h"
#include "representations/rcss/SayMessage.h"
#include "representations/rcss/BeamRequest.h"

MODULE(HelloWorldBehaviorModule)
  REQUIRES(JointData)
  PROVIDES(JointRequestWithSpeeds)
  PROVIDES(SayMessage)
  PROVIDES(BeamRequest)
END_MODULE

class HelloWorldBehaviorModule: public HelloWorldBehaviorModuleBase
{
  public:
    HelloWorldBehaviorModule();
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

#endif /* HELLOWORLDBEHAVIORMODULE_H_ */
