#include "ParserModule.h"
#include "math/Matrix.h"
#include <cstdio>

// S-Expressions
#define SEXP_time "time"
#define SEXP_now "now"
#define SEXP_GS "GS"
#define SEXP_unum "unum"
#define SEXP_team "team"
#define SEXP_t "t"
#define SEXP_pm "pm"
#define SEXP_GYR "GYR"
#define SEXP_torso "torso"
#define SEXP_rt "rt"
#define SEXP_ACC "ACC"
#define SEXP_a "a"
#define SEXP_HJ "HJ"
#define SEXP_n "n"
#define SEXP_ax "ax"
#define SEXP_See "See"
// flags
#define SEXP_F1L "F1L"
#define SEXP_F1R "F1R"
#define SEXP_F2L "F2L"
#define SEXP_F2R "F2R"
// goalposts
#define SEXP_G1L "G1L"
#define SEXP_G1R "G1R"
#define SEXP_G2L "G2L"
#define SEXP_G2R "G2R"
#define SEXP_B "B"
#define SEXP_L "L"
#define SEXP_P "P"
// players
#define SEXP_id "id"
#define SEXP_head "head"
#define SEXP_rlowerarm "rlowerarm"
#define SEXP_llowerarm "llowerarm"
#define SEXP_rfoot "rfoot"
#define SEXP_lfoot "lfoot"
#define SEXP_pol "pol"
//
#define SEXP_FRP "FRP"
#define SEXP_rf "rf"
#define SEXP_c "c"
#define SEXP_f "f"
#define SEXP_lf "lf"
#define SEXP_hear "hear"
#define SEXP_self "self"

#define SEXP_COMPARE(sx, SEXP_VALUE) (strcmp(sx->list->val, SEXP_VALUE) == 0)

void ParserModule::init()
{
  config.setPersist(false);
  parseGroundtruth = config.getValue("parseGroundtruth", false);
  frameNumber = 0;
  lastMessageTime = 0;
  messageTime = 0;

  // Joint translation map
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("hj1", JID_HEAD_PAN));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("hj2", JID_HEAD_TILT));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("laj1", JID_ARM_LEFT0));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("laj2", JID_ARM_LEFT1));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("laj3", JID_ARM_LEFT2));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("laj4", JID_ARM_LEFT3));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("raj1", JID_ARM_RIGHT0));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("raj2", JID_ARM_RIGHT1));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("raj3", JID_ARM_RIGHT2));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("raj4", JID_ARM_RIGHT3));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("llj1", JID_LEG_LEFT0));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("llj2", JID_LEG_LEFT1));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("llj3", JID_LEG_LEFT2));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("llj4", JID_LEG_LEFT3));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("llj5", JID_LEG_LEFT4));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("llj6", JID_LEG_LEFT5));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("rlj1", JID_LEG_RIGHT0));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("rlj2", JID_LEG_RIGHT1));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("rlj3", JID_LEG_RIGHT2));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("rlj4", JID_LEG_RIGHT3));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("rlj5", JID_LEG_RIGHT4));
  joint_name_2_id.insert(std::pair<std::string, JOINT_ID>("rlj6", JID_LEG_RIGHT5));

  // Team Side tanslation map
  team_side_2_id.insert(std::pair<std::string, TEAM_ID>("left", TEAM_LEFT));
  team_side_2_id.insert(std::pair<std::string, TEAM_ID>("right", TEAM_RIGHT));

  // Playmode translation map
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("BeforeKickOff", Gamestate::BEFORE_KICKOFF));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("PlayOn", Gamestate::PLAYON));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("GameOver", Gamestate::GAMEOVER));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("KickOff_Left", Gamestate::KICKOFF_OWN));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("KickOff_Right", Gamestate::KICKOFF_OPP));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("KickIn_Left", Gamestate::KICKIN_OWN));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("KickIn_Right", Gamestate::KICKIN_OPP));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("goal_kick_left", Gamestate::GOALKICK_OWN));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("goal_kick_right", Gamestate::GOALKICK_OPP));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("Goal_Left", Gamestate::GOAL_OWN));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("Goal_Right", Gamestate::GOAL_OPP));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("corner_kick_left", Gamestate::CORNERKICK_OWN));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("corner_kick_right", Gamestate::CORNERKICK_OPP));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("offside_left", Gamestate::OFFSIDE_OWN));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("offside_right", Gamestate::OFFSIDE_OPP));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("free_kick_left", Gamestate::FREEKICK_OWN));
  pm_name_2_id.insert(std::pair<std::string, Gamestate::PLAYMODES>("free_kick_right", Gamestate::FREEKICK_OPP));
  //for TEAM_RIGHT
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("BeforeKickOff", Gamestate::BEFORE_KICKOFF));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("PlayOn", Gamestate::PLAYON));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("GameOver", Gamestate::GAMEOVER));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("KickOff_Left", Gamestate::KICKOFF_OPP));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("KickOff_Right", Gamestate::KICKOFF_OWN));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("KickIn_Left", Gamestate::KICKIN_OPP));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("KickIn_Right", Gamestate::KICKIN_OWN));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("goal_kick_left", Gamestate::GOALKICK_OPP));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("goal_kick_right", Gamestate::GOALKICK_OWN));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("Goal_Left", Gamestate::GOAL_OPP));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("Goal_Right", Gamestate::GOAL_OWN));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("corner_kick_left", Gamestate::CORNERKICK_OPP));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("corner_kick_right", Gamestate::CORNERKICK_OWN));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("offside_left", Gamestate::OFFSIDE_OPP));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("offside_right", Gamestate::OFFSIDE_OWN));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("free_kick_left", Gamestate::FREEKICK_OPP));
  pm_name_2_id2.insert(std::pair<std::string, Gamestate::PLAYMODES>("free_kick_right", Gamestate::FREEKICK_OWN));

  identifier_2_body_part.insert(std::pair<std::string, PartPercept::ROBOT_PART>("head", PartPercept::HEAD));
  identifier_2_body_part.insert(std::pair<std::string, PartPercept::ROBOT_PART>("rlowerarm", PartPercept::RIGHT_ARM));
  identifier_2_body_part.insert(std::pair<std::string, PartPercept::ROBOT_PART>("llowerarm", PartPercept::LEFT_ARM));
  identifier_2_body_part.insert(std::pair<std::string, PartPercept::ROBOT_PART>("rfoot", PartPercept::RIGHT_FOOT));
  identifier_2_body_part.insert(std::pair<std::string, PartPercept::ROBOT_PART>("lfoot", PartPercept::LEFT_FOOT));

}

void ParserModule::execute()
{
  reset();

  if (theServerMessage->firstMsg)
  {
    ownTeamname = theServerMessage->initTeamname;
    unum = theServerMessage->initUnum;
  }

  // Parser logic
  const char* msg = theServerMessage->msg.c_str();
  size_t len = strlen(msg);
  if (len > 0)
  {
    cc = init_continuation((char*) msg);
    sx = (sexp_t *) iparse_sexp((char*) msg, len, cc);
    while (sx != NULL)
    {
      parseExpr();
      destroy_sexp(sx);
      sexp_cleanup();
      sx = (sexp_t *) iparse_sexp((char*) msg, len, cc);
    }
    destroy_continuation(cc);
  }

  frameNumber++;
}

void ParserModule::parseExpr()
{
  //printf("Data tag: [%s]\n", sx->list->val);
  if (SEXP_COMPARE(sx, SEXP_time))
  {
    float data = 0;
    /* s = (vallist) */
    sexp_t* varlist = sx->list->next; // ()
    if (varlist != 0)
    {
      sexp_t* atom = varlist->list->next;
      parseReal(atom, data);
      setMessageTime(data);
    }
    return;
  }

  if (SEXP_COMPARE(sx, SEXP_hear))
  {
    std::string data[3];
    /* s = (vallist) */
    sexp_t* varlist = sx; // (x)
    if (varlist != 0)
    {
      sexp_t* atom = varlist->list->next;
      parseStrings(atom, data, 3);
    }
    float time = 0, direction = 0;
    bool self = false;
    sscanf(data[0].c_str(), "%f", &time);
    if (strcmp(data[1].c_str(), SEXP_self) != 0)
    {
      sscanf(data[1].c_str(), "%f", &direction);
    } else
    {
      self = true;
    }
    addHearMsg(time, direction, self, data[2]);
    return;
  }

  if (SEXP_COMPARE(sx, SEXP_HJ))
  {
    std::string joint_id;
    float angle = 0;
    sexp_t* varlist = sx->list->next; // (x)
    while (varlist != 0)
    {
      if (SEXP_COMPARE(varlist, SEXP_n))
      {
        sexp_t* atom = varlist->list->next;
        parseString(atom, joint_id);
      }
      else if (SEXP_COMPARE(varlist, SEXP_ax))
      {
        sexp_t* atom = varlist->list->next;
        parseReal(atom, angle);
      }
      varlist = varlist->next; // ()(x)
    }
    setJoint(joint_id, angle);
    return;
  }

  if (SEXP_COMPARE(sx, SEXP_GS))
  {
    std::string playmode, teamIndex;
    float gameTime = 0;
    int agentUnum = 0;
    sexp_t* varlist = sx->list->next; // (x)
    while (varlist != 0)
    {
      if (SEXP_COMPARE(varlist, SEXP_t))
      {
        sexp_t* atom = varlist->list->next;
        parseReal(atom, gameTime);
      }
      else if (SEXP_COMPARE(varlist, SEXP_pm))
      {
        sexp_t* atom = varlist->list->next;
        parseString(atom, playmode);
      }
      else if (SEXP_COMPARE(varlist, SEXP_unum))
      {
        sexp_t* atom = varlist->list->next;
        parseInt(atom, agentUnum);
      }
      else if (SEXP_COMPARE(varlist, SEXP_team))
      {
        sexp_t* atom = varlist->list->next;
        parseString(atom, teamIndex);
      }
      varlist = varlist->next; // ()(x)
    }
    setGamestate(gameTime, playmode);
    if (agentUnum > 0 && teamIndex.length() > 0)
    {
      setPlayerInfo(agentUnum, teamIndex);
    }
    return;
  }

  if (SEXP_COMPARE(sx, SEXP_GYR))
  {
    float data[3] =
    { 0 };
    sexp_t* varlist = sx->list->next; // (x)
    while (varlist != 0)
    {
      if (SEXP_COMPARE(varlist, SEXP_rt))
      {
        sexp_t* atomList = varlist->list->next;
        parseReals(atomList, data, 3);
        setGyro(std::string(SEXP_torso), data[0], data[1], data[2]);
      }
      varlist = varlist->next; // ()(x)
    }
    return;
  }

  if (SEXP_COMPARE(sx, SEXP_ACC))
  {
    float data[3] =
    { 0 };
    sexp_t* varlist = sx->list->next; // (x)
    while (varlist != 0)
    {
      if (SEXP_COMPARE(varlist, SEXP_a))
      {
        sexp_t* atomList = varlist->list->next;
        parseReals(atomList, data, 3);
        setAcc(std::string(SEXP_torso), data[0], data[1], data[2]);
      }
      varlist = varlist->next; // ()(x)
    }
    return;
  }

  if (SEXP_COMPARE(sx, SEXP_FRP))
  {
    std::string name;
    float cdata[3] =
    { 0 };
    float fdata[3] =
    { 0 };
    sexp_t* varlist = sx->list->next; // (x)
    while (varlist != 0)
    {
      if (SEXP_COMPARE(varlist, SEXP_n))
      {
        sexp_t* atom = varlist->list->next;
        parseString(atom, name);
      }
      else if (SEXP_COMPARE(varlist, SEXP_c))
      {
        sexp_t* atomList = varlist->list->next;
        parseReals(atomList, cdata, 3);
      }
      else if (SEXP_COMPARE(varlist, SEXP_f))
      {
        sexp_t* atomList = varlist->list->next;
        parseReals(atomList, fdata, 3);
      }
      varlist = varlist->next; // ()(x)
    }

    if (name.length() > 0)
    {
      setForce(name, cdata[0], cdata[1], cdata[2], fdata[0], fdata[1], fdata[2]);
    }
    return;
  }

  if (SEXP_COMPARE(sx, SEXP_See))
  {
    float data[3] =
    { 0 };
    sexp_t* varlist = sx->list->next; // (x)
    while (varlist != 0)
    {
      if (SEXP_COMPARE(varlist, SEXP_B))
      {
        //printf("Data value: [%s]\n", varlist->list->val);
        sexp_t* atomList = varlist->list->next->list->next; // (x a)
        parseReals(atomList, data, 3);
        setBallPercept(data[0], data[1], data[2]);
      }
      else if (SEXP_COMPARE(varlist, SEXP_F1L) || SEXP_COMPARE(varlist, SEXP_F1R) || SEXP_COMPARE(varlist, SEXP_F2L)
          || SEXP_COMPARE(varlist, SEXP_F2R))
      {
        //printf("Data value: [%s]\n", varlist->list->val);
        sexp_t* atomList = varlist->list->next->list->next; // (x a)
        parseReals(atomList, data, 3);
        setStaticVisionObject(std::string(varlist->list->val), data[0], data[1], data[2]);
      }
      else if (SEXP_COMPARE(varlist, SEXP_G1L) || SEXP_COMPARE(varlist, SEXP_G1R) || SEXP_COMPARE(varlist, SEXP_G2L)
          || SEXP_COMPARE(varlist, SEXP_G2R))
      {
        //printf("Data value: [%s]\n", varlist->list->val);
        sexp_t* atomList = varlist->list->next->list->next; // (x a)
        parseReals(atomList, data, 3);
        setStaticVisionObject(std::string(varlist->list->val), data[0], data[1], data[2]);
      }
      else if (SEXP_COMPARE(varlist, SEXP_P))
      {
        //printf("Data value: [%s]\n", varlist->list->val);
        std::string teamname;
        int playerID = 0;
        sexp_t* innerVarList = varlist->list->next;
        do
        {
          //printf("\t Data value: [%s]\n", innerVarList->list->val);
          if (SEXP_COMPARE(innerVarList, SEXP_team))
          {
            sexp_t* atom = innerVarList->list->next;
            parseString(atom, teamname);
          }
          else if (SEXP_COMPARE(innerVarList, SEXP_id))
          {
            sexp_t* atom = innerVarList->list->next;
            parseInt(atom, playerID);
          }
          else if (SEXP_COMPARE(innerVarList, SEXP_head)
              || SEXP_COMPARE(innerVarList, SEXP_rlowerarm)
              || SEXP_COMPARE(innerVarList, SEXP_llowerarm)
              || SEXP_COMPARE(innerVarList, SEXP_rfoot) || SEXP_COMPARE(innerVarList, SEXP_lfoot))
          {
            std::string body_part_identifier(innerVarList->list->val, strlen(innerVarList->list->val));
            //printf("\t Part name: [%s]\n", partName.c_str());
            sexp_t* atomList = innerVarList->list->next->list->next; // (x a)
            parseReals(atomList, data, 3);
            if (playerID > 0 && teamname.length() > 0)
            {
              setRobotPartPercept(teamname, playerID, body_part_identifier, data[0], data[1], data[2]);
            }
          }

        } while ((innerVarList = innerVarList->next) != 0);
      }


      varlist = varlist->next; // ()(x)
    }
    return;
  }
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
  lines.push_back(std::make_pair(Polar(a1, a2, a3), Polar(b1, b2, b3)));
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


void ParserModule::setRobotPartPercept(const std::string& team, const int& payerId,
    const std::string& body_part_identifier, const float distance, const float azimuth,
    const float elevation)
{
  std::vector<PartPercept> &robotPartsCurrent =
      (team == ownTeamname ? robotPartsOwn : robotPartsOpp);
  PartPercept part;
  part.unum = payerId;
  part.type = translate_robot_part(body_part_identifier);
  part.polar = Polar(distance, azimuth, elevation);
  robotPartsCurrent.push_back(part);
}

//------------------------------------------------------------------------------
//translate methods

const JOINT_ID ParserModule::translate_joint_id(const std::string& joint_id) const
{
  std::map<std::string, JOINT_ID>::const_iterator iter = this->joint_name_2_id.find(joint_id);
  return (iter != this->joint_name_2_id.end()) ? iter->second : NUM_JOINT_ID;
}
const TEAM_ID ParserModule::translate_team_side(const std::string& team) const
{
  std::map<std::string, TEAM_ID>::const_iterator iter = this->team_side_2_id.find(team);
  return (iter != this->team_side_2_id.end()) ? iter->second : NUM_TEAM_ID;
}
const Gamestate::PLAYMODES ParserModule::translate_playmode(const std::string& pm,
    std::map<std::string, Gamestate::PLAYMODES>& map) const
{
  std::map<std::string, Gamestate::PLAYMODES>::const_iterator iter = map.find(pm);
  return (iter != map.end()) ? iter->second : Gamestate::NUM_PLAYMODES;
}

const PartPercept::ROBOT_PART ParserModule::translate_robot_part(
    const std::string& body_part_identifier) const
{
  std::map<std::string, PartPercept::ROBOT_PART>::const_iterator iter = identifier_2_body_part.find(
      body_part_identifier);
  return (iter != identifier_2_body_part.end()) ? iter->second : PartPercept::NUM_ROBOT_PART;
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
  std::map<std::string, Polar>::iterator iter;
  iter = staticVisionObjects.find("F2R");
  if (iter != staticVisionObjects.end())
    theFlagPercept.flags.insert(std::make_pair((team == TEAM_RIGHT ? FLAG_OWN_LEFT : FLAG_FOE_RIGHT), iter->second));
  iter = staticVisionObjects.find("F1R");
  if (iter != staticVisionObjects.end())
    theFlagPercept.flags.insert(std::make_pair((team == TEAM_RIGHT ? FLAG_OWN_RIGHT : FLAG_FOE_LEFT), iter->second));
  iter = staticVisionObjects.find("F2L");
  if (iter != staticVisionObjects.end())
    theFlagPercept.flags.insert(std::make_pair((team == TEAM_RIGHT ? FLAG_FOE_LEFT : FLAG_OWN_RIGHT), iter->second));
  iter = staticVisionObjects.find("F1L");
  if (iter != staticVisionObjects.end())
    theFlagPercept.flags.insert(std::make_pair((team == TEAM_RIGHT ? FLAG_FOE_RIGHT : FLAG_OWN_LEFT), iter->second));
}
void ParserModule::update(GoalPercept& theGoalPercept)
{
  theGoalPercept.goalposts.clear();
  std::map<std::string, Polar>::iterator iter;
  iter = staticVisionObjects.find("G2R");
  if (iter != staticVisionObjects.end())
    theGoalPercept.goalposts.insert(
        std::make_pair((team == TEAM_RIGHT ? GOALPOST_OWN_LEFT : GOALPOST_FOE_RIGHT), iter->second));
  iter = staticVisionObjects.find("G1R");
  if (iter != staticVisionObjects.end())
    theGoalPercept.goalposts.insert(
        std::make_pair((team == TEAM_RIGHT ? GOALPOST_OWN_RIGHT : GOALPOST_FOE_LEFT), iter->second));
  iter = staticVisionObjects.find("G2L");
  if (iter != staticVisionObjects.end())
    theGoalPercept.goalposts.insert(
        std::make_pair((team == TEAM_RIGHT ? GOALPOST_FOE_LEFT : GOALPOST_OWN_RIGHT), iter->second));
  iter = staticVisionObjects.find("G1L");
  if (iter != staticVisionObjects.end())
    theGoalPercept.goalposts.insert(
        std::make_pair((team == TEAM_RIGHT ? GOALPOST_FOE_RIGHT : GOALPOST_OWN_LEFT), iter->second));
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

MAKE_MODULE(ParserModule)
