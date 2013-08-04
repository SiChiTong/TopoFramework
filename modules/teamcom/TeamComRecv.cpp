#include "TeamComRecv.h"

MAKE_MODULE(TeamComRecv)

TeamComRecv::TeamComRecv()
{
}

TeamComRecv::~TeamComRecv()
{
}

void TeamComRecv::init()
{
  //log << "max message size: " << msgEncoder.getBitSize() << std::endl;
}

void TeamComRecv::execute()
{
  //clear old received data
  for (std::vector<BitBuffer*>::iterator iter = data.begin(); iter < data.end(); iter++)
    delete *iter;
  data.clear();

  //read new received data
  messageReceived = false;
  for (unsigned int i = 0; i < theHearMessage->messages.size(); i++)
    if ( //!theHearMessage->messages[i].self &&
    theHearMessage->messages[i].data.length() == MsgEncoder::MSG_LENGTH)
    {
      if (processHeardMessage(theHearMessage->messages[i]))
        messageReceived = true;
      else
      {
        //log << "invalid message" << std::endl;
      }
    }
}

bool TeamComRecv::processHeardMessage(const HearMessageData& msg)
{
  if (!msgEncoder.readMessage(msg.data))
    return false;
  //log << msg.data << std::endl;

  //read stuff
  BitInt senderUnum(4);
  BitBool senderTeam;
  BitVector2 bitSenderPos(theFieldDimensions, 14);

  //read first values  
  int bitsRead = 0;
  bitsRead += msgEncoder.read(senderUnum);
  if (senderUnum > MAX_PLAYERS)
    return false;
  bitsRead += msgEncoder.read(senderTeam);
  if (senderTeam != (thePlayerInfo->team == TEAM_LEFT))
    return false;
  bitsRead += msgEncoder.read(bitSenderPos);

  //check if sender pose is in hear direction
  senderPos = bitSenderPos;
  double angle = (senderPos - theRobotPose->pose.translation).angle();
  poseInconsistency = sin(fabs(angle - msg.direction))
      * (senderPos - theRobotPose->pose.translation).abs();
  draw(senderPos, poseInconsistency > 0.5, msg.direction);

  //read user data
  const int messageSize = msgEncoder.getBitSize();
  const int lengthBits = ceil(::log((double) (messageSize - bitsRead)) / ::log(2.0));
  const int maxBufferSize = messageSize - (bitsRead + lengthBits);
  BitInt len(lengthBits);
  while (bitsRead + lengthBits <= messageSize)
  {
    //read length
    bitsRead += msgEncoder.read(len);
    int l = len;
    if (l == 0)
      break;
    if (l > maxBufferSize)
    {
      //log << "wrong length l=" << l << " (max " << maxBufferSize << ")" << std::endl;
      return false;
    }
    //create buffer and read data
    BitBuffer *buffer = new BitBuffer(l);
    bitsRead += msgEncoder.read(*buffer);
    data.push_back(buffer);
  }

  //if everything was read correctly, trust that it is one of our messages
  this->senderUnum = senderUnum;
  return true;
}

void TeamComRecv::draw(const Vector2<double> &pos, bool red, double direction)
{
  int r = (red ? 255 : 120);
  Vector2<double> vec(0.5, 0);
  vec.rotate(direction);
  vec += theRobotPose->pose.translation;
  drawing.line("TeamCom.msgDirection", theRobotPose->pose.translation.x,
      theRobotPose->pose.translation.y, 0, vec.x, vec.y, 0, r, 200, 200, 5);
  drawing.circle("TeamCom.msgPos", pos.x, pos.y, 0.2, 2, r, 200, 200);
}

void TeamComRecv::update(TeamComInfo& theTeamComInfo)
{
  theTeamComInfo.messageReceived = messageReceived;
  theTeamComInfo.senderUnum = senderUnum;
  theTeamComInfo.senderPos = senderPos;
  theTeamComInfo.poseInconsistency = poseInconsistency;
}

void TeamComRecv::update(TeamComDataIn& theTeamComDataIn)
{
  theTeamComDataIn.data.clear();
  theTeamComDataIn.data = data;
}

