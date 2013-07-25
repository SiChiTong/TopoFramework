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
    connected(false), initUnum(0)
{
}

InputModule::~InputModule()
{
  ime::Communication::deleteInstance();
}

void InputModule::init()
{
  initTeamname = config.getValue("rcssTeamname", std::string("NaoRobot"));
  initUnum = config.getValue("unum", 0);
}

void InputModule::update(ServerMessage& theServerMessage)
{
  if (!connected && !theServerMessage.firstMsg)
    theServerMessage.firstMsg = true;
  else
    theServerMessage.firstMsg = false;

  if (!connected)
  {
    theServerMessage.initTeamname = initTeamname;
    theServerMessage.initUnum = initUnum;
    std::string rcssAgent = config.getValue("rcssAgent", std::string("rsg/agent/nao/nao.rsg"));
    std::stringstream ssMsgNao, ssMsgInit;
    ssMsgNao << "(scene " << rcssAgent << ")";
    ssMsgInit << "(init " << "(unum " << initUnum << ")(teamname " << initTeamname << "))(syn)";
    ime::Communication& com = ime::Communication::getInstance();
    com.initInstance(config.getValue("rcssHost", std::string("127.0.0.1")),
        config.getValue("rcssPort", 3100));
    com.putMessage(ssMsgNao.str());
    com.getMessage(theServerMessage.msg);
    com.putMessage(ssMsgInit.str());
    connected = true;
  }

  ime::Communication& com = ime::Communication::getInstance();
  com.getMessage(theServerMessage.msg);

  //std::cout << "msg=> " << theServerMessage.msg << std::endl;
}

void InputModule::update(FieldDimensions& theFieldDimensions)
{
  if (theFieldDimensions.initialized)
    return;

  ime::Config configFieldDimensions("FieldDimensions", "config");
  configFieldDimensions.resurrect();
  //field size
  theFieldDimensions.length = configFieldDimensions.getValue("fieldLength", -1.0f);
  theFieldDimensions.halfLength = 0.5f * theFieldDimensions.length;
  theFieldDimensions.width = configFieldDimensions.getValue("fieldWidth", -1.0f);
  theFieldDimensions.halfWidth = 0.5f * theFieldDimensions.width;

  theFieldDimensions.borderStripWidth = configFieldDimensions.getValue("borderStripWidth", -1.0f);
  theFieldDimensions.lengthPlusBorder = theFieldDimensions.length
      + theFieldDimensions.borderStripWidth * 2.0f;
  theFieldDimensions.halfLengthPlusBorder = 0.5f * theFieldDimensions.lengthPlusBorder;
  theFieldDimensions.widthPlusBorder = theFieldDimensions.width
      + theFieldDimensions.borderStripWidth * 2.0f;
  theFieldDimensions.halfWidthPlusBorder = 0.5f * theFieldDimensions.widthPlusBorder;

  //goal
  theFieldDimensions.goalWidth = configFieldDimensions.getValue("goalWidth", -1.0f);
  theFieldDimensions.halfGoalWidth = 0.5f * theFieldDimensions.goalWidth;
  theFieldDimensions.goalDepth = configFieldDimensions.getValue("goalDepth", -1.0f);
  theFieldDimensions.goalHeight = configFieldDimensions.getValue("goalHeight", -1.0f);
  theFieldDimensions.goalPostRadius = configFieldDimensions.getValue("goalPostRadius", -1.0f);

  //field lines
  theFieldDimensions.lineWidth = configFieldDimensions.getValue("lineWidth", -1.0f);
  theFieldDimensions.penaltyAreaWidth = configFieldDimensions.getValue("penaltyAreaWidth", -1.0f);
  theFieldDimensions.halfPenaltyAreaWidth = 0.5f * theFieldDimensions.penaltyAreaWidth;
  theFieldDimensions.penaltyAreaDepth = configFieldDimensions.getValue("penaltyAreaDepth", -1.0f);
  theFieldDimensions.centerCircleRadius = configFieldDimensions.getValue("centerCircleRadius", -1.0f);

  //ball
  theFieldDimensions.ballRadius = configFieldDimensions.getValue("ballRadius", -1.0f);

  //spl
  theFieldDimensions.penaltyMarkSize = configFieldDimensions.getValue("penaltyMarkSize", -1.0f);
  theFieldDimensions.penaltyMarkDistance = configFieldDimensions.getValue("penaltyMarkDistance", -1.0f);

  //simspark rules, move somewhere else?
  theFieldDimensions.freeKickDistance = configFieldDimensions.getValue("rcss.freeKickDistance", -1.0f);
  theFieldDimensions.freeKickMoveDist = configFieldDimensions.getValue("rcss.freeKickMoveDist", -1.0f);
  theFieldDimensions.goalKickDistance = configFieldDimensions.getValue("rcss.goalKickDistance", -1.0f);

  theFieldDimensions.initialized = true;
}

MAKE_MODULE(InputModule)

#endif /* INPUTMODULE_CPP_ */
