/*
 * DrawingModule.cpp
 *
 *  Created on: Jul 23, 2013
 *      Author: sam
 */

#include "DrawingModule.h"

DrawingModule::DrawingModule() :
    connected(false)
{
}

void DrawingModule::execute()
{
  if (!connected && thePlayerInfo->unum > 0)
  {
    ime::Drawing& drawingInstace = ime::Drawing::getInstance();
    drawingInstace.initInstance(config.getValue("robovizHost", std::string("127.0.0.1")),
        config.getValue("robovizPort", 32769), thePlayerInfo->teamname, thePlayerInfo->unum);
    connected = true;
  }
}

