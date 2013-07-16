#include "ParserModule.h"

#include "math/Matrix.h"
#include <cstdio>
using namespace std;

MAKE_MODULE(ParserModule)

void ParserModule::init()
{
  // fixMe
  //parseGroundtruth = config.getValue("parseGroundtruth", false);
  parseGroundtruth = false;
  frameNumber = 0;
  lastMessageTime = 0;
  messageTime = 0;

  //ownTeamname = theServerMessage->initTeamname;
  ownTeamname = "Foo";
  //unum = theServerMessage->initUnum;
  unum = 0;

  // Joint translation map
  joint_name_2_id.insert(pair<string, JOINT_ID>("hj1", JID_HEAD_PAN));
  joint_name_2_id.insert(pair<string, JOINT_ID>("hj2", JID_HEAD_TILT));
  joint_name_2_id.insert(pair<string, JOINT_ID>("laj1", JID_ARM_LEFT0));
  joint_name_2_id.insert(pair<string, JOINT_ID>("laj2", JID_ARM_LEFT1));
  joint_name_2_id.insert(pair<string, JOINT_ID>("laj3", JID_ARM_LEFT2));
  joint_name_2_id.insert(pair<string, JOINT_ID>("laj4", JID_ARM_LEFT3));
  joint_name_2_id.insert(pair<string, JOINT_ID>("raj1", JID_ARM_RIGHT0));
  joint_name_2_id.insert(pair<string, JOINT_ID>("raj2", JID_ARM_RIGHT1));
  joint_name_2_id.insert(pair<string, JOINT_ID>("raj3", JID_ARM_RIGHT2));
  joint_name_2_id.insert(pair<string, JOINT_ID>("raj4", JID_ARM_RIGHT3));
  joint_name_2_id.insert(pair<string, JOINT_ID>("llj1", JID_LEG_LEFT0));
  joint_name_2_id.insert(pair<string, JOINT_ID>("llj2", JID_LEG_LEFT1));
  joint_name_2_id.insert(pair<string, JOINT_ID>("llj3", JID_LEG_LEFT2));
  joint_name_2_id.insert(pair<string, JOINT_ID>("llj4", JID_LEG_LEFT3));
  joint_name_2_id.insert(pair<string, JOINT_ID>("llj5", JID_LEG_LEFT4));
  joint_name_2_id.insert(pair<string, JOINT_ID>("llj6", JID_LEG_LEFT5));
  joint_name_2_id.insert(pair<string, JOINT_ID>("rlj1", JID_LEG_RIGHT0));
  joint_name_2_id.insert(pair<string, JOINT_ID>("rlj2", JID_LEG_RIGHT1));
  joint_name_2_id.insert(pair<string, JOINT_ID>("rlj3", JID_LEG_RIGHT2));
  joint_name_2_id.insert(pair<string, JOINT_ID>("rlj4", JID_LEG_RIGHT3));
  joint_name_2_id.insert(pair<string, JOINT_ID>("rlj5", JID_LEG_RIGHT4));
  joint_name_2_id.insert(pair<string, JOINT_ID>("rlj6", JID_LEG_RIGHT5));

  // Team Side tanslation map
  team_side_2_id.insert(pair<string, TEAM_ID>("left", TEAM_LEFT));
  team_side_2_id.insert(pair<string, TEAM_ID>("right", TEAM_RIGHT));

  // Playmode translation map
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("BeforeKickOff", Gamestate::BEFORE_KICKOFF));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("PlayOn", Gamestate::PLAYON));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("GameOver", Gamestate::GAMEOVER));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("KickOff_Left", Gamestate::KICKOFF_OWN));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("KickOff_Right", Gamestate::KICKOFF_OPP));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("KickIn_Left", Gamestate::KICKIN_OWN));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("KickIn_Right", Gamestate::KICKIN_OPP));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("goal_kick_left", Gamestate::GOALKICK_OWN));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("goal_kick_right", Gamestate::GOALKICK_OPP));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("Goal_Left", Gamestate::GOAL_OWN));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("Goal_Right", Gamestate::GOAL_OPP));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("corner_kick_left", Gamestate::CORNERKICK_OWN));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("corner_kick_right", Gamestate::CORNERKICK_OPP));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("offside_left", Gamestate::OFFSIDE_OWN));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("offside_right", Gamestate::OFFSIDE_OPP));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("free_kick_left", Gamestate::FREEKICK_OWN));
  pm_name_2_id.insert(pair<string, Gamestate::PLAYMODES>("free_kick_right", Gamestate::FREEKICK_OPP));
  //for TEAM_RIGHT
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("BeforeKickOff", Gamestate::BEFORE_KICKOFF));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("PlayOn", Gamestate::PLAYON));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("GameOver", Gamestate::GAMEOVER));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("KickOff_Left", Gamestate::KICKOFF_OPP));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("KickOff_Right", Gamestate::KICKOFF_OWN));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("KickIn_Left", Gamestate::KICKIN_OPP));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("KickIn_Right", Gamestate::KICKIN_OWN));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("goal_kick_left", Gamestate::GOALKICK_OPP));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("goal_kick_right", Gamestate::GOALKICK_OWN));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("Goal_Left", Gamestate::GOAL_OPP));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("Goal_Right", Gamestate::GOAL_OWN));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("corner_kick_left", Gamestate::CORNERKICK_OPP));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("corner_kick_right", Gamestate::CORNERKICK_OWN));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("offside_left", Gamestate::OFFSIDE_OPP));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("offside_right", Gamestate::OFFSIDE_OWN));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("free_kick_left", Gamestate::FREEKICK_OPP));
  pm_name_2_id2.insert(pair<string, Gamestate::PLAYMODES>("free_kick_right", Gamestate::FREEKICK_OWN));

  /* 
   // Touch sensor translation map
   insert(this->touch_name_2_id)
   ("lf", TID_FOOT_LEFT)
   ("rf", TID_FOOT_RIGHT);
   */
}

void ParserModule::execute()
{
  reset();

  // Parser logic
  const char* msg = theServerMessage->msg.c_str();
  size_t len = strlen(msg);
  if (len > 0)
  {
    sx = 0;
    cc = 0;
    cc = init_continuation((char*) msg);
    sx = (sexp_t *) iparse_sexp((char*) msg, len, cc);
    while (sx != NULL)
    {
      destroy_sexp(sx);
      sexp_cleanup();
      sx = (sexp_t *) iparse_sexp((char*) msg, len, cc);
    }
    destroy_continuation(cc);
  }

  // fixMe: parse the messages message
  frameNumber++;
}

//-------------------------------------------------------------------------
// update methods
void ParserModule::update(FrameInfo& theFrameInfo)
{
  theFrameInfo.time = messageTime;
  theFrameInfo.time_ms = messageTime * 1000.0;
  theFrameInfo.frameNumber = frameNumber;
  //printf("---------- Received server message, time = %f ----------\n",
  //       theFrameInfo->time);
}
void ParserModule::update(PlayerInfo& thePlayerInfo)
{
  thePlayerInfo.isValid = true;
  thePlayerInfo.unum = unum;
  thePlayerInfo.team = team;
  thePlayerInfo.teamname = ownTeamname;
}
void ParserModule::update(JointData& theJointData)
{
  //check if joint angles are exactly the same
  bool sameAngles = true;
  for (int i = 0; i < NUM_JOINT_ID && sameAngles; i++)
    if (theJointData.values[i].angle != jointValues[i].angle)
      sameAngles = false;
  //if (sameAngles)
  //  log << "WARNING: Same joint angles received!" << std::endl;

  theJointData.values = jointValues;
}
void ParserModule::update(SimSparkLinePercept& theLinePercept)
{
  theLinePercept.lines.clear();
  theLinePercept.lines.insert(theLinePercept.lines.begin(), lines.begin(), lines.end());
}
void ParserModule::update(BallPercept& theBallPercept)
{
  theBallPercept.polar = ballPercept;
  theBallPercept.updated = ballPerceptUpdated;
}
void ParserModule::update(FlagPercept& theFlagPercept)
{
  theFlagPercept.flags.clear();
  map<std::string, Polar>::iterator iter;
  iter = staticVisionObjects.find("F2R");
  if (iter != staticVisionObjects.end())
    theFlagPercept.flags.insert(make_pair((team == TEAM_RIGHT ? FLAG_OWN_LEFT : FLAG_FOE_RIGHT), iter->second));
  iter = staticVisionObjects.find("F1R");
  if (iter != staticVisionObjects.end())
    theFlagPercept.flags.insert(make_pair((team == TEAM_RIGHT ? FLAG_OWN_RIGHT : FLAG_FOE_LEFT), iter->second));
  iter = staticVisionObjects.find("F2L");
  if (iter != staticVisionObjects.end())
    theFlagPercept.flags.insert(make_pair((team == TEAM_RIGHT ? FLAG_FOE_LEFT : FLAG_OWN_RIGHT), iter->second));
  iter = staticVisionObjects.find("F1L");
  if (iter != staticVisionObjects.end())
    theFlagPercept.flags.insert(make_pair((team == TEAM_RIGHT ? FLAG_FOE_RIGHT : FLAG_OWN_LEFT), iter->second));
}
void ParserModule::update(GoalPercept& theGoalPercept)
{
  theGoalPercept.goalposts.clear();
  map<std::string, Polar>::iterator iter;
  iter = staticVisionObjects.find("G2R");
  if (iter != staticVisionObjects.end())
    theGoalPercept.goalposts.insert(
        make_pair((team == TEAM_RIGHT ? GOALPOST_OWN_LEFT : GOALPOST_FOE_RIGHT), iter->second));
  iter = staticVisionObjects.find("G1R");
  if (iter != staticVisionObjects.end())
    theGoalPercept.goalposts.insert(
        make_pair((team == TEAM_RIGHT ? GOALPOST_OWN_RIGHT : GOALPOST_FOE_LEFT), iter->second));
  iter = staticVisionObjects.find("G2L");
  if (iter != staticVisionObjects.end())
    theGoalPercept.goalposts.insert(
        make_pair((team == TEAM_RIGHT ? GOALPOST_FOE_LEFT : GOALPOST_OWN_RIGHT), iter->second));
  iter = staticVisionObjects.find("G1L");
  if (iter != staticVisionObjects.end())
    theGoalPercept.goalposts.insert(
        make_pair((team == TEAM_RIGHT ? GOALPOST_FOE_RIGHT : GOALPOST_OWN_LEFT), iter->second));
}

void ParserModule::update(SensorData& theSensorData)
{
  theSensorData.acc = acc;
  theSensorData.gyro = gyro;
}
void ParserModule::update(Gamestate& theGamestate)
{
  theGamestate.gametime = this->gametime;
  if (team == TEAM_RIGHT)
    theGamestate.playmode = translate_playmode(playmode, pm_name_2_id2);
  else
    theGamestate.playmode = translate_playmode(playmode, pm_name_2_id);
}
void ParserModule::update(Groundtruth& theGroundtruth)
{
  if (!parseGroundtruth)
    return;
  theGroundtruth.myself = groundtruthMyself;
  theGroundtruth.myselfUp = groundtruthMyselfUp;
  theGroundtruth.myselfZ = groundtruthMyselfZ;
  theGroundtruth.myselfForward = groundtruthMyselfFw;
  theGroundtruth.calculateTorsoTransformation();
  theGroundtruth.ball3D = groundtruthBall3D;
  theGroundtruth.ball.x = groundtruthBall3D.x;
  theGroundtruth.ball.y = groundtruthBall3D.y;
  theGroundtruth.otherRobots = groundtruthOtherRobots;
  theGroundtruth.updated = groundtruthUpdated;
  if (team == TEAM_RIGHT) //rotate groundtruth, when playing on right side
  {
    theGroundtruth.myself.translation.rotate(pi);
    theGroundtruth.myself.rotate(pi);
    theGroundtruth.ball.rotate(pi);
    theGroundtruth.ball3D = RotationMatrix(0, 0, pi) * theGroundtruth.ball3D;
    //TODO rotate other robots
  }
}
void ParserModule::update(HearMessage& theHearMessage)
{
  theHearMessage.messages.clear();
  theHearMessage.messages.insert(theHearMessage.messages.begin(), hearMessages.begin(), hearMessages.end());
}
void ParserModule::update(ForceData& theForceData)
{
  theForceData.updatedLeft = forceDataUpdatedLeft;
  theForceData.updatedRight = forceDataUpdatedRight;
  theForceData.contactLeft = forceContactLeft;
  theForceData.vectorLeft = forceVectorLeft;
  theForceData.contactRight = forceContactRight;
  theForceData.vectorRight = forceVectorRight;
}
void ParserModule::update(RobotPartPercept& theRobotPartPercept)
{
  theRobotPartPercept.teammates = robotPartsOwn;
  theRobotPartPercept.opponents = robotPartsOpp;
}

//-------------------------------------------------------------------------
// Methods called from the parser

void ParserModule::reset()
{
  lines.clear();
  staticVisionObjects.clear();
  ballPerceptUpdated = false;
  hearMessages.clear();
  forceDataUpdatedLeft = false;
  forceDataUpdatedRight = false;
  groundtruthOtherRobots.teammates.clear();
  groundtruthOtherRobots.opponents.clear();
  groundtruthUpdated = false;
  robotPartsOwn.clear();
  robotPartsOpp.clear();
  robotPartsCurrent.clear();
}

void ParserModule::setMessageTime(const float t)
{
  if (t > lastMessageTime)
    messageTime += t - lastMessageTime;
  lastMessageTime = t;
}
void ParserModule::setPlayerInfo(const int unum, const std::string& team_index)
{
  this->team = translate_team_side(team_index);

  //debug.init(ownTeamname, unum, (team==TEAM_LEFT)); 
  //log.setPlayerId(unum);
  //monitor.init((team == TEAM_LEFT), unum);

  this->unum = unum;
  //log << "received unum: " << this->unum << std::endl;
}

void ParserModule::addLinePercept(const float a1, const float a2, const float a3, const float b1, const float b2,
    const float b3)
{
  lines.push_back(make_pair(Polar(a1, a2, a3), Polar(b1, b2, b3)));
}
void ParserModule::setStaticVisionObject(const std::string& obj_id, const float distance, const float azimuth,
    const float elevation)
{
  staticVisionObjects.insert(make_pair(obj_id, Polar(distance, azimuth, elevation)));
}
void ParserModule::setBallPercept(const float distance, const float azimuth, const float elevation)
{
  ballPercept = Polar(distance, azimuth, elevation);
  ballPerceptUpdated = true;
}
void ParserModule::setJoint(const std::string& joint_id, const float angle)
{
  jointValues[translate_joint_id(joint_id)].angle = angle * pi / 180.0;
}
void ParserModule::setAcc(const std::string& /*acc_id*/, const float x, const float y, const float z)
{
  //swap x and y and invert x to convert into out local coordinate system
  acc = Vector3<double>(y, -x, z);
}
void ParserModule::setGyro(const std::string& /*gyr_id*/, const float x, const float y, const float z)
{
  gyro = Vector3<double>(x, y, z);
  gyro *= pi / 180.0; //convert to radian
}
void ParserModule::setGamestate(const double game_time, const std::string& playmode)
{
  this->gametime = game_time;
  this->playmode = playmode;
}
void ParserModule::setGroundtruthMyself(const double x, const double y, const double r, const double upx,
    const double upy, const double upz, const double z, const double fwx, const double fwy, const double fwz)
{
  groundtruthMyself = Pose2D(r, x, y);
  groundtruthMyselfUp = RotationMatrix::fromRotationZ(-r) * Vector3<double>(upx, upy, upz);
  groundtruthMyselfZ = z;
  groundtruthMyselfFw = RotationMatrix::fromRotationZ(-r) * Vector3<double>(fwx, fwy, fwz);
  groundtruthUpdated = true;
}
void ParserModule::setGroundtruthBall(const double x, const double y, const double z)
{
  groundtruthBall3D = Vector3<double>(x, y, z);
  groundtruthUpdated = true;
}
void ParserModule::setGroundtruthPlayer(const std::string& team, const float id, const double x, const double y,
    const double r)
{
  if (team == ownTeamname && id == unum) //only other robots
    return;
  std::vector<OtherRobotsData::State> &targetVec = (
      team == ownTeamname ? groundtruthOtherRobots.teammates : groundtruthOtherRobots.opponents);
  OtherRobotsData::State state;
  state.unum = id;
  state.pose = Pose2D(r, x, y);
  state.relativePose.translation = groundtruthMyself.invert() * state.pose.translation;
  state.relativePose.rotation = normalize(state.pose.rotation - groundtruthMyself.rotation);
  state.upright = Vector3<double>(0, 0, 1);
  targetVec.push_back(state);
  groundtruthUpdated = true;
}
void ParserModule::addHearMsg(const float time, const float dir, const bool self, const std::string &msg)
{
  HearMessageData m;
  m.time = time;
  m.self = self;
  m.direction = dir / 180.0 * M_PI;
  m.data = msg;
  hearMessages.push_back(m);
}
void ParserModule::setForce(const std::string name, const float x, const float y, const float z, const float vx,
    const float vy, const float vz)
{
  if (name == "rf")
  {
    forceContactRight = Vector3<double>(x, y, z);
    forceVectorRight = Vector3<double>(vx, vy, vz);
    forceDataUpdatedRight = true;
  }
  if (name == "lf")
  {
    forceContactLeft = Vector3<double>(x, y, z);
    forceVectorLeft = Vector3<double>(vx, vy, vz);
    forceDataUpdatedLeft = true;
  }
}

void ParserModule::setRobotPartPlayer(const std::string& team, const float id)
{
  std::vector<PartPercept> &targetVec = (team == ownTeamname ? robotPartsOwn : robotPartsOpp);
  std::vector<PartPercept>::iterator iter = robotPartsCurrent.begin();
  while (iter != robotPartsCurrent.end())
  {
    iter->unum = id;
    targetVec.push_back(*iter);
    iter++;
  }
  robotPartsCurrent.clear();
}
void ParserModule::setRobotPartPercept(PartPercept::ROBOT_PART type, const float distance, const float azimuth,
    const float elevation)
{
  PartPercept part;
  part.unum = -1;
  part.type = type;
  part.polar = Polar(distance, azimuth, elevation);
  robotPartsCurrent.push_back(part);
}

//------------------------------------------------------------------------------
//translate methods

const JOINT_ID ParserModule::translate_joint_id(const std::string& joint_id) const
{
  map<string, JOINT_ID>::const_iterator iter = this->joint_name_2_id.find(joint_id);
  return (iter != this->joint_name_2_id.end()) ? iter->second : NUM_JOINT_ID;
}
const TEAM_ID ParserModule::translate_team_side(const std::string& team) const
{
  map<string, TEAM_ID>::const_iterator iter = this->team_side_2_id.find(team);
  return (iter != this->team_side_2_id.end()) ? iter->second : NUM_TEAM_ID;
}
const Gamestate::PLAYMODES ParserModule::translate_playmode(const std::string& pm,
    std::map<std::string, Gamestate::PLAYMODES>& map) const
{
  std::map<std::string, Gamestate::PLAYMODES>::const_iterator iter = map.find(pm);
  return (iter != map.end()) ? iter->second : Gamestate::NUM_PLAYMODES;
}

/*
 const TOUCH_ID ParserModule::translate_touch_id(const string& touch_id) const
 {
 map<string,TOUCH_ID>::const_iterator iter
 = this->touch_name_2_id.find(touch_id);
 return (iter!=this->touch_name_2_id.end()) ? iter->second : num_TOUCH_ID;
 }


 */
