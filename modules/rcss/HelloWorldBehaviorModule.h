/*
 * HelloWorldBehaviorModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef HELLOWORLDBEHAVIORMODULE_H_
#define HELLOWORLDBEHAVIORMODULE_H_

#include "kernel/Framework.h"
#include "representations/rcss/JointValues.h"

MODULE(HelloWorldBehaviorModule)
REQUIRES(InputJointValues)
PROVIDES(OutputJointValues)
END_MODULE

class HelloWorldBehaviorModule: public HelloWorldBehaviorModuleBase
{
  public:
    HelloWorldBehaviorModule();
    void update(OutputJointValues& theOutputJointValues);

  protected:
    enum eState
    {
      S_Wait, S_Sit, S_SwingLeft, S_SwingRight,
    } mState;

    int mCounter;


};

#endif /* HELLOWORLDBEHAVIORMODULE_H_ */
