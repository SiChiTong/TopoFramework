#include "BeliefShare.h"

MAKE_MODULE(BeliefShare)

BeliefShare::BeliefShare()
{
  emptyState.unum = 0;
  for (int i = 0; i <= MAX_PLAYERS; i++)
  {
    teamTeammates[i].unum = i;
    teamTeammates[i].confidence = 0;
    teamOpponents[i].unum = i;
    teamOpponents[i].confidence = 0;
  }

  buffersInitialized = false;
}
;

BeliefShare::~BeliefShare()
{
  if (buffersInitialized)
  {
    delete ballBuffer;
  }
}

void BeliefShare::init()
{
  bitsPoseT = config.getValue("bitsPoseTranslation", 12);
  bitsPoseR = config.getValue("bitsPoseRotation", 4);

  //TODO Get goalie unum in a different way, without accessing the config
  //     of the formation module here.
  ime::Config formationConfig("Formation", "config");
  goalieUnum = formationConfig.getValue("goalieUnum", 1);
  goalieBallPosTimeReceived = -1;
}

void BeliefShare::execute()
{
  if (theTC_BeliefShareIn->data.size() > 0)
  {
    if (theTeamComInfo->poseInconsistency > 0.3)
      return;

    //read all received buffers
    std::vector<BitBuffer*>::const_iterator iter = theTC_BeliefShareIn->data.begin();
    while (iter != theTC_BeliefShareIn->data.end())
    {
      BitBuffer *buffer = *iter;

      //receive ball position
      if (buffer->getSize() == 23)
      {
        BallPosData ballPos;
        ballPos.absPos = BitVector2(theFieldDimensions, *buffer, 0, 16);
        ballPos.confidence = BitFloat(*buffer, 16, 4, 3, false);
        processReceivedBall(ballPos);
      }
      //receive other robots positions
      if (buffer->getSize() == 17 + bitsPoseT + bitsPoseR)
      {
        OtherRobots::State state;
        int offset = 0;
        bool ownTeam = BitBool(*buffer, offset);
        offset += 1;
        state.unum = BitInt(*buffer, offset, 4);
        offset += 4;
        state.confidence = BitFloat(*buffer, offset, 4, 3, false);
        offset += 7;
        const int uprightBits = 5;
        int upright = BitInt(*buffer, offset, uprightBits);
        offset += uprightBits;
        state.pose = BitPose2D(theFieldDimensions, *buffer, offset, bitsPoseT, bitsPoseR);
        offset += bitsPoseT + bitsPoseR;
        if (offset != buffer->getSize())
          printf("offset = %d, buffer size = %d (reading robot states)\n", offset,
              buffer->getSize());
        //upright vector (5 bit, 31 angles + not fallen)
        if (upright == 0)
          state.upright = Vector3<double>(0, 0, 0.5);
        else
        {
          double angle = ((double) upright + 0.5 - 1.0) / ((1 << uprightBits) - 1) * pi2 - pi;
          Vector2<double> v(0.5, 0);
          v.rotate(angle);
          state.upright = Vector3<double>(v.x, v.y, 0);
        }
        if (state.unum <= MAX_PLAYERS)
          processReceivedRobotState(ownTeam, state);
      }
      iter++; //next BitBuffer*
    }
  }
}

void BeliefShare::update(TC_BeliefShareOut& theTC_BeliefShareOut)
{
  //first clear previous content
  theTC_BeliefShareOut.data.clear();

#ifdef TARGET_NAO
  if(thePlayerInfo->penalized
      || theGamestate->playmode == Gamestate::PREINITIAL
      || theFallState->fallen
      || !theGroundContactState->contact)
  return;
#endif

  //init buffers
  if (!buffersInitialized)
  {
    ballBuffer = new BitBuffer(23);
    for (int i = 0; i < MAX_OTHER_ROBOT_STATES; i++)
      otherRobotBuffer[i] = new BitBuffer(17 + bitsPoseT + bitsPoseR);
    buffersInitialized = true;
  }

  //send ball if local confidence higher than confidence of team ball
  BallPosData ballPos;
  createBallPos(ballPos);
  bool sendingBall = (!theFallState->fallen && ballPos.confidence > teamBallPos.confidence);
  if (sendingBall)
  {
    BitVector2(theFieldDimensions, *ballBuffer, 0, 16) = ballPos.absPos;
    BitFloat(*ballBuffer, 16, 4, 3, false) = ballPos.confidence;
    theTC_BeliefShareOut.data.push_back(ballBuffer);
  }
//return; // NO OTHER ROBOT STATES !!!!!!!!!!!!!!!

  //add other robot states
  int nOtherRobots = 0;
  const OtherRobots::State *state;
  double prevGain = 1000000.0;
  bool ownTeam = getStateToSend(state, prevGain);

//printf("start send other robot states\n");
  while (nOtherRobots < MAX_OTHER_ROBOT_STATES && state != &emptyState)
  {
    BitBuffer *buffer = otherRobotBuffer[nOtherRobots];
    int offset = 0;
    BitBool(*buffer, offset) = ownTeam;
    offset += 1;
    BitInt(*buffer, offset, 4) = state->unum;
    offset += 4;
    BitFloat(*buffer, offset, 4, 3, false) = state->confidence;
    offset += 7;
    const int uprightBits = 5;
    int upright;
    //upright vector (5 bit, 31 lying angles + not fallen)
    if (!state->fallen())
      upright = 0;
    else
    {
      const double angle = std::max(0.0,
          std::min(pi2, Vector2<double>(state->upright.x, state->upright.y).angle() + pi));
      upright = angle / pi2 * ((1 << uprightBits) - 1) + 1;
      if (upright == (1 << uprightBits))
        upright = (1 << uprightBits) - 1;
    }
//printf("send robot state team=%d  unum=%d  fallen=%d\n",ownTeam, state->unum, state->fallen()); 
    BitInt(*buffer, offset, uprightBits) = upright;
    offset += uprightBits;
    BitPose2D(theFieldDimensions, *buffer, offset, bitsPoseT, bitsPoseR) = state->pose;
    offset += bitsPoseT + bitsPoseR;
    if (offset != buffer->getSize())
      printf("offset = %d, buffer size = %d (sending robot states)\n", offset, buffer->getSize());
    theTC_BeliefShareOut.data.push_back(buffer);
    //get next state to send
    nOtherRobots++;
    ownTeam = getStateToSend(state, prevGain);
  }
}

void BeliefShare::createBallPos(BallPosData &ballpos)
{
  ballpos = *theLocalBallPos;
  const RobotPoseData *robot;
  if (theLocalRobotPose->confidence > teamRobotPose.confidence)
    robot = theLocalRobotPose;
  else
    robot = &teamRobotPose;
  ballpos.confidence = std::min(ballpos.confidence, robot->confidence);
  ballpos.absPos = robot->pose * ballpos.relativePos;
}

bool BeliefShare::getStateToSend(const OtherRobotsData::State* &state, double &prevGain)
{
  state = &emptyState;
  bool ownTeam = true;

  double maxGain = 0;
  if (!theFallState->fallen)
  {
    //find max confidence gain for all states in LocalOtherRobots
    std::vector<OtherRobotsData::State>::const_iterator iter;
    /*for(iter = theLocalOtherRobots->teammates.begin();
     iter != theLocalOtherRobots->teammates.end(); ++iter)
     if(iter->unum > 0 && iter->unum < MAX_PLAYERS)
     {
     double gain = iter->confidence - teamTeammates[iter->unum].confidence;
     if(gain < prevGain && gain > maxGain)
     {
     state = &(*iter);
     //don't overwrite upright in team belief
     state.upright = teamTeammates[state->unum].upright;
     maxGain = gain;
     }
     }*/
    for (iter = theLocalOtherRobots->opponents.begin();
        iter != theLocalOtherRobots->opponents.end(); ++iter)
      if (iter->unum > 0 && iter->unum <= MAX_PLAYERS)
      {
        double gain = iter->confidence - teamOpponents[iter->unum].confidence;
        if (gain < prevGain && gain > maxGain)
        {
          ownTeam = false;
          state = &(*iter);
          maxGain = gain;
        }
      }
  }

  //check gain for own RobotPose
  double ownGain = theLocalRobotPose->confidence - teamRobotPose.confidence;
  if (ownGain < prevGain && (ownGain > maxGain || theFallState->fallen))
  {
    ownTeam = true;
    state = &ownState;
    maxGain = ownGain;
    ownState.unum = thePlayerInfo->unum;
    ownState.confidence = theLocalRobotPose->confidence;
    ownState.pose = theLocalRobotPose->pose;
    ownState.upright = (
        theFallState->fallen ? Vector3<double>(1.0, 0, 0) : Vector3<double>(0, 0, 1.0));
  }

  if (state->unum > 0)
    drawing.pose("TeamCom.sendState", state->pose, 0.2, 2, 255, 255, 255);
  prevGain = maxGain;
  return ownTeam;
}

void BeliefShare::processReceivedBall(const BallPosData &pos)
{
  if (theTeamComInfo->senderUnum == goalieUnum)
  {
    goalieBallPos = pos;
    goalieBallPosTimeReceived = theFrameInfo->time_ms;
  }

  if (pos.confidence > teamBallPos.confidence)
  {
    teamBallPos = pos;
    //teamBallPos.confidence *= 0.95;
  }
  drawing.circle("TeamCom.receivedBallPos", pos.absPos.x, pos.absPos.y, 0.1, 2, 200, 200, 200);
}

void BeliefShare::processReceivedRobotState(bool ownTeam, const OtherRobotsData::State &state)
{
  if (state.unum == 0) //skip if empty
    return;

  if (ownTeam && state.unum == thePlayerInfo->unum)
  {
    //TODO maybe accept own position, if fallen ???
    if (state.confidence > teamRobotPose.confidence)
    {
      //update teamRobotPose from state
      teamRobotPose.pose = state.pose;
      teamRobotPose.confidence = 0; //state.confidence; //don't trust others
    }
  }
  else
  {
    OtherRobotsData::State &teamState = (
        ownTeam ? teamTeammates[state.unum] : teamOpponents[state.unum]);
    if (state.confidence > teamState.confidence)
    {
      if (ownTeam && theTeamComInfo->senderUnum == state.unum && state.fallen())
        //state received from fallen robot itself, only update upright value
        teamState.upright = state.upright;
      else
        //otherwise use whole state
        teamState = state;
    }
  }
  if (state.fallen())
    drawing.pose("TeamCom.receivedRobotState", state.pose, 0.35, 2, 200, 200, 200);
  else
    drawing.pose("TeamCom.receivedRobotState", state.pose, 0.25, 2, 200, 200, 200);
}

//--- update methods for team belief ---

void BeliefShare::update(TeamRobotPose& theTeamRobotPose)
{
  *((RobotPoseData*) &theTeamRobotPose) = teamRobotPose;
  teamRobotPose.confidence *= 0.95;
  //TODO motionUpdate using odometry
}

void BeliefShare::update(TeamBallPos& theTeamBallPos)
{
  *((BallPosData*) &theTeamBallPos) = teamBallPos;
  teamBallPos.confidence *= 0.95;
  //TODO if the ball had a speed, the position would have to be updated here
}

void BeliefShare::update(GoalieBallPos& theGoalieBallPos)
{
  *((BallPosData*) &theGoalieBallPos) = goalieBallPos;
  theGoalieBallPos.timeReceived = goalieBallPosTimeReceived;
}

void BeliefShare::update(TeamOtherRobots& theTeamOtherRobots)
{
  theTeamOtherRobots.teammates.clear();
  theTeamOtherRobots.opponents.clear();
  for (int i = 1; i <= MAX_PLAYERS; i++)
  {
    if (teamTeammates[i].confidence > 0.01)
      theTeamOtherRobots.teammates.push_back(teamTeammates[i]);
    teamTeammates[i].confidence *= 0.985;
    //TODO motionUpdate using velocity
  }
  for (int i = 1; i <= MAX_PLAYERS; i++)
  {
    if (teamOpponents[i].confidence > 0.01)
      theTeamOtherRobots.opponents.push_back(teamOpponents[i]);
    teamOpponents[i].confidence *= 0.95;
    //TODO motionUpdate using velocity
  }
}

