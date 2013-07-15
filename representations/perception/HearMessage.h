#ifndef HEARMESSAGE_H
#define HEARMESSAGE_H

#include "kernel/Framework.h"
#include <string>

REPRESENTATION(HearMessage)

class HearMessageData
{
  public:
    float time;
    float direction;
    std::string data;
    bool self;
};

class HearMessage: public HearMessageBase
{
  public:

    std::vector<HearMessageData> messages;

};

#endif

