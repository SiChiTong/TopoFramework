/*
 * InputModule.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef INPUTMODULE_CPP_
#define INPUTMODULE_CPP_

#include "InputModule.h"
#include <sstream>

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
    std::string rcssAgent = config.getValue("rcssAgent", std::string("rsg/agent/nao/nao.rsg"));
    int unum = config.getValue("unum", 0);
    std::string rcssTeamname = config.getValue("rcssTeamname", std::string("NaoRobot"));
    std::stringstream ssMsgNao, ssMsgInit;
    ssMsgNao << "(scene " << rcssAgent << ")";
    ssMsgInit << "(init " << "(unum " << unum << ")(teamname " << rcssTeamname << "))(syn)";
    ime::Communication& com = ime::Communication::getInstance();
    com.initInstance(config.getValue("rcssHost", std::string("127.0.0.1")), config.getValue("rcssPort", 3100));
    com.putMessage(ssMsgNao.str());
    com.getMessage(msg);
    std::cout << msg << std::endl;
    com.putMessage(ssMsgInit.str());
    connected = true;
  }
  ime::Communication& com = ime::Communication::getInstance();
  com.getMessage(theServerMessage.msg);

  //std::cout << theServerMessage.msg << std::endl;
}

MAKE_MODULE(InputModule)

#endif /* INPUTMODULE_CPP_ */
