/*
 * InputModule.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef INPUTMODULE_CPP_
#define INPUTMODULE_CPP_

#include "InputModule.h"
#include "utils/com/Communication.h"

InputModule::InputModule() :
    connected(false)
{
}

InputModule::~InputModule()
{
  ime::Communication::deleteInstance();
}

void InputModule::init()
{

  if (!connected)
  {
    // fixMe: get these values from a config
    std::string gHost = "127.0.0.1";
    int gPort = 3100;
    std::string msgNao = "(scene rsg/agent/nao/nao.rsg)";
    ime::Communication& com = ime::Communication::getInstance();
    com.initInstance(gHost, gPort);
    com.putMessage(msgNao);
    com.getMessage(msg);
    std::string msgInit = "(init (unum 0)(teamname NaoRobot))";
    com.putMessage(msgInit);
    connected = true;
  }
}

void InputModule::update(InputJointValues& theJointValues)
{
  ime::Communication& com = ime::Communication::getInstance();
  com.getMessage(msg);

  std::cout << msg << std::endl;
  // TODO: fixMe parse the new message.
}

MAKE_MODULE(InputModule)

#endif /* INPUTMODULE_CPP_ */
