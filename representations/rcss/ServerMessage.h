/*
 * ServerMessage.h
 *
 *  Created on: Jul 15, 2013
 *      Author: sam
 */

#ifndef SERVERMESSAGE_H_
#define SERVERMESSAGE_H_

#include "kernel/Framework.h"
#include <string>

REPRESENTATION(ServerMessage)
class ServerMessage : public ServerMessageBase
{
  public:
    ServerMessage() : msg("") {}
    std::string msg;
};



#endif /* SERVERMESSAGE_H_ */
