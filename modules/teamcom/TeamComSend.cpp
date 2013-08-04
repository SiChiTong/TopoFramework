#include "TeamComSend.h"

MAKE_MODULE(TeamComSend)

TeamComSend::TeamComSend()
{
  timeout = 3000;
  prevId = 0;
  for (int i = 0; i < MAX_PLAYERS + 1; i++)
    lastReceiveTime[i] = 0;
}
;

TeamComSend::~TeamComSend()
{
}

void TeamComSend::init()
{
  //log << "max message size: " << msgEncoder.getBitSize() << std::endl;
}

void TeamComSend::execute()
{
  if (theTeamComInfo->messageReceived)
  {
    updateSendOrder(theTeamComInfo->senderUnum);
  }
}

void TeamComSend::updateSendOrder(int id)
{
  lastReceiveTime[id] = theFrameInfo->time_ms;
  lastReceivedId = id;
  prevId = thePlayerInfo->unum;
  do
  {
    prevId -= 1;
    if (prevId < 1)
      prevId = MAX_PLAYERS;
  } while ((theFrameInfo->time_ms - lastReceiveTime[prevId]) > timeout
      && prevId != thePlayerInfo->unum);
}

void TeamComSend::update(SayMessage& theSayMessage)
{
  if (thePlayerInfo->unum > 0
      && ((lastReceivedId == prevId && (prevId == thePlayerInfo->unum - 1 || (rand() & 7) != 0))
          || (theFrameInfo->time_ms - lastReceiveTime[prevId]) > timeout))
  {
    lastReceivedId = -1;
    msgEncoder.clear();
    createOutputMessage();
    msgEncoder.createMessage(theSayMessage.msg);
    theSayMessage.active = true;
    //log << "sending message" << std::endl;
  }
  else
  {
    theSayMessage.active = false;
  }
}

void TeamComSend::createOutputMessage()
{
  //write stuff
  BitInt senderUnum(4);
  BitBool senderTeam;
  BitVector2 bitSenderPos(theFieldDimensions, 14);

  senderUnum = thePlayerInfo->unum;
  senderTeam = (thePlayerInfo->team == TEAM_LEFT);
  bitSenderPos = theRobotPose->pose.translation;

  int bitsWritten = 0;
  bitsWritten += msgEncoder.write(senderUnum);
  bitsWritten += msgEncoder.write(senderTeam);
  bitsWritten += msgEncoder.write(bitSenderPos);

  //append user data
  const int messageSize = msgEncoder.getBitSize();
  const int lengthBits = ceil(::log((double) (messageSize - bitsWritten)) / ::log(2.0));
  const int maxBufferSize = messageSize - (bitsWritten + lengthBits);
  std::vector<BitBuffer*>::const_iterator iter = theTeamComDataOut->data.begin();
  while (iter != theTeamComDataOut->data.end()
      && (bitsWritten + lengthBits + (*iter)->getSize() <= messageSize
          || (*iter)->getSize() > maxBufferSize))
  {
    if ((*iter)->getSize() <= maxBufferSize)
    {
      //length
      bitsWritten += msgEncoder.write(BitInt(lengthBits) = (*iter)->getSize());
      //data
      bitsWritten += msgEncoder.write(*(*iter));
    }
    else
    {
      //log << "WARNING: Message skipped. Too long to be sent!" << std::endl;
    }
    iter++;
  }
  //if it fits, add length value 0 as stop signal
  if (bitsWritten + lengthBits <= messageSize)
    bitsWritten += msgEncoder.write(BitInt(lengthBits) = 0);
}

