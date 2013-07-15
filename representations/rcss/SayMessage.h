#ifndef SAYMESSAGE_H
#define SAYMESSAGE_H

#include "kernel/Framework.h"
#include <string>

REPRESENTATION(SayMessage)

class SayMessage: public SayMessageBase
{
  public:
    SayMessage() :
        active(false), msg("")
    {
    }
    bool active;
    std::string msg;

};

#endif

