/*
 * InputModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef INPUTMODULE_H_
#define INPUTMODULE_H_

#include "kernel/Framework.h"
#include "kernel/Communication.h"
#include "representations/rcss/ServerMessage.h"

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

  private:
    bool connected;
    std::string initTeamname;
    int initUnum;
};

#endif /* INPUTMODULE_H_ */
