/*
 * InputModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef INPUTMODULE_H_
#define INPUTMODULE_H_

#include "kernel/Framework.h"
#include "representations/rcss/JointValues.h"

MODULE(InputModule)
PROVIDES(InputJointValues)
END_MODULE

class InputModule: public InputModuleBase
{
  public:
    InputModule();
    ~InputModule();
    void init();
    void update(InputJointValues& theInputJointValues);

  private:
    bool connected;
    std::string msg;
};

#endif /* INPUTMODULE_H_ */
