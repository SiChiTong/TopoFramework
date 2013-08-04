#ifndef MSGENCODER_H
#define MSGENCODER_H

#include <string>
#include "representations/teamcom/BitBuffer.h"
#include "BaseConverter.h"

class MsgEncoder
{
  private:

    //message encoding
    int charSetSize;
    char *charSet;
    int charIndex[256];
    int msgBits;
    BaseConverter converter;

    unsigned char *buffer;
    int p;

  public:
    enum
    {
      MSG_LENGTH = 20
    };

    MsgEncoder();
    ~MsgEncoder();

    void clear();
    bool readMessage(const std::string &msg);
    void createMessage(std::string &msg);

    int getBitSize();

    //write and read methods
    int write(const BitBuffer &bitbuffer);
    int read(BitBuffer &bitbuffer);

};

#endif

