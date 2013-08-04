#include "MsgEncoder.h"
#include <cstring>
#include <cstdio>

MsgEncoder::MsgEncoder() :
    converter(MSG_LENGTH)
{
  char chars[256] =
  { 0 };
  strcat(chars, "abcdefghijklmnopqrstuvwxyz");
  strcat(chars, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
  strcat(chars, "1234567890");
  strcat(chars, "_-");
  /** @todo parser has to accept all allowed characters */

  charSetSize = strlen(chars);

  charSet = new char[charSetSize + 1];
  strcpy(charSet, chars);

  for (int i = 0; i < 256; i++)
    charIndex[i] = -1;
  for (int i = 0; i < charSetSize; i++)
    charIndex[(unsigned char) charSet[i]] = i;

  msgBits = ::log(pow((double) charSetSize, (double) MSG_LENGTH)) / ::log(2.0);
  buffer = converter.getData();
}

MsgEncoder::~MsgEncoder()
{
  delete[] charSet;
}

void MsgEncoder::clear()
{
  converter.clear();
  p = 0;
}

int MsgEncoder::getBitSize()
{
  return msgBits;
}

bool MsgEncoder::readMessage(const std::string &msg)
{
  //write message into converter buffer
  converter.clear();
  for (unsigned int i = 0; i < msg.length(); i++)
  {
    const int index = charIndex[(unsigned char) msg.at(i)];
    if (index < 0)
      return false;
    converter[i] = index;
  }
  //convert to binary
  converter.convert(charSetSize, 256);
  p = 0;
  return true;
}

void MsgEncoder::createMessage(std::string &msg)
{
  converter.convert(256, charSetSize);
  msg = "";
  for (int i = 0; i < MSG_LENGTH; i++)
    msg.append(1, charSet[converter.getData()[i]]);
}

int MsgEncoder::write(const BitBuffer &bitbuffer)
{
  const int size = bitbuffer.getSize();
  BitBuffer tmp(buffer, p, size);
  for (int i = 0; i < size; i++)
    tmp.setBit(i, bitbuffer.getBit(i));
  p += size;
  return size;
}
int MsgEncoder::read(BitBuffer &bitbuffer)
{
  const int size = bitbuffer.getSize();
  BitBuffer tmp(buffer, p, size);
  for (int i = 0; i < size; i++)
    bitbuffer.setBit(i, tmp.getBit(i));
  p += size;
  return size;
}

