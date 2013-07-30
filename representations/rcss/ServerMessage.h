/*
 * ServerMessage.h
 *
 *  Created on: Jul 15, 2013
 *      Author: sam
 */

#ifndef SERVERMESSAGE_H_
#define SERVERMESSAGE_H_

#include "kernel/Template.h"
#include <string>

REPRESENTATION(ServerMessage)
class ServerMessage : public ServerMessageBase
{
  public:
    ServerMessage() : msg(""), initTeamname(""), initUnum(0), firstMsg(false) {}
    std::string msg;
    //information used for the init message
    std::string initTeamname;
    int initUnum;
    bool firstMsg;
};



#endif /* SERVERMESSAGE_H_ */
