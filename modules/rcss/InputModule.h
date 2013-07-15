/*
 * InputModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef INPUTMODULE_H_
#define INPUTMODULE_H_

#include "kernel/Framework.h"
#include "representations/rcss/ServerMessage.h"

#include "utils/com/Communication.h"

MODULE(InputModule)
  PROVIDES(ServerMessage)
END_MODULE

class InputModule: public InputModuleBase
{
  public:
    InputModule();
    ~InputModule();
    void init();
    void update(ServerMessage& theServerMessage);
};

#endif /* INPUTMODULE_H_ */
