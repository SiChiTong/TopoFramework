/*
 * InputModule.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef INPUTMODULE_CPP_
#define INPUTMODULE_CPP_

#include "InputModule.h"

InputModule::InputModule() :
    connected(false)
{
}

InputModule::~InputModule()
{
  ime::Communication::deleteInstance();
}

void InputModule::update(ServerMessage& theServerMessage)
{
  if (!connected)
  {
    std::string msg;
    std::string msgNao = config.getValue("rcssAgent", std::string("(scene rsg/agent/nao/nao.rsg)"));
    ime::Communication& com = ime::Communication::getInstance();
    com.initInstance(config.getValue("rcssHost", std::string("127.0.0.1")), config.getValue("rcssPort", 3100));
    com.putMessage(msgNao);
    com.getMessage(msg);
    std::string msgInit = config.getValue("rcssTeam", std::string("(init (unum 0)(teamname NaoRobot))(syn)"));
    com.putMessage(msgInit);
    connected = true;
  }
  ime::Communication& com = ime::Communication::getInstance();
  com.getMessage(theServerMessage.msg);

  std::cout << theServerMessage.msg << std::endl;
}

MAKE_MODULE(InputModule)

#endif /* INPUTMODULE_CPP_ */
