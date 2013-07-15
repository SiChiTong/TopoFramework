/*
 * OutputModule.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#include "OutputModule.h"
#include "utils/com/Communication.h"

void OutputModule::execute()
{
  ime::Communication& com = ime::Communication::getInstance();
  com.putMessage(theOutputJointValues->msg);
}

MAKE_MODULE(OutputModule)

