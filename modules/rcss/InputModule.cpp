/*
 * InputModule.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef INPUTMODULE_CPP_
#define INPUTMODULE_CPP_

#include "InputModule.h"

InputModule::InputModule()
{
}

InputModule::~InputModule()
{
  ime::Communication::deleteInstance();
}

void InputModule::init()
{
  // Established connection.
  // fixMe: get these values from a config
  std::string gHost = "127.0.0.1";
  int gPort = 3100;
  std::string msg;
  std::string msgNao = "(scene rsg/agent/nao/nao.rsg)";
  ime::Communication& com = ime::Communication::getInstance();
  com.initInstance(gHost, gPort);
  com.putMessage(msgNao);
  com.getMessage(msg);
  std::string msgInit = "(init (unum 0)(teamname NaoRobot))(syn)";
  com.putMessage(msgInit);

}

void InputModule::update(ServerMessage& theServerMessage)
{
  ime::Communication& com = ime::Communication::getInstance();
  com.getMessage(theServerMessage.msg);
}

MAKE_MODULE(InputModule)

#endif /* INPUTMODULE_CPP_ */
