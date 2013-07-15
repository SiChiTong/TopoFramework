#ifndef PARSER_H
#define PARSER_H

#include "kernel/Framework.h"
#include "representations/rcss/ServerMessage.h"

#include "representations/perception/FrameInfo.h"
#include "representations/perception/PlayerInfo.h"
#include "representations/perception/JointData.h"
#include "representations/perception/SensorData.h"
#include "representations/perception/ForceData.h"
#include "representations/perception/SimSparkLinePercept.h"
#include "representations/perception/GoalPercept.h"
#include "representations/perception/FlagPercept.h"
#include "representations/perception/BallPercept.h"
#include "representations/perception/RobotPartPercept.h"
#include "representations/perception/Gamestate.h"
#include "representations/perception/Groundtruth.h"
#include "representations/perception/HearMessage.h"

#include "utils/sexpr/sexp.h"
#include "utils/sexpr/sexp_ops.h"
#include <string.h>


MODULE(ParserModule)
  REQUIRES(ServerMessage)

  PROVIDES(FrameInfo)
  PROVIDES(PlayerInfo)
  PROVIDES(Gamestate)
  
  PROVIDES(SimSparkLinePercept)
  PROVIDES(BallPercept)
  PROVIDES(FlagPercept)
  PROVIDES(GoalPercept)
  PROVIDES(RobotPartPercept)

  PROVIDES(SensorData)
  PROVIDES(ForceData)
  PROVIDES(HearMessage)
  PROVIDES(Groundtruth)
  
  PROVIDES(JointData)  
END_MODULE


class ParserModule: public ParserModuleBase
{
  public:

    void init();

    void execute();

    //update methods
    void update(FrameInfo& theFrameInfo);
    void update(PlayerInfo& thePlayerInfo);
    void update(JointData& theJointData);
    void update(SensorData& theSensorData);
    void update(ForceData& theForceData);
    void update(SimSparkLinePercept& theLinePercept);
    void update(GoalPercept& theGoalPercept);
    void update(FlagPercept& theFlagPercept);
    void update(BallPercept& theBallPercept);
    void update(RobotPartPercept& theRobotPartPercept);
    void update(Gamestate& theGamestate);
    void update(Groundtruth& theGroundtruth);
    void update(HearMessage& theHearMessage);

    //methods called by the parser
    void reset();
    void setMessageTime(const float t_now);
    void setPlayerInfo(const int unum, const std::string& team_index);
    void setJoint(const std::string& joint_id, const float angle);
    void setBallPercept(const float distance, const float azimuth, const float elevation);

    void addLinePercept(const float a1, const float a2, const float a3, const float b1,
        const float b2, const float b3);
    void setStaticVisionObject(const std::string& obj_id, const float distance, const float azimuth,
        const float elevation);
    void setAcc(const std::string& acc_id, const float x, const float y, const float z);
    void setGyro(const std::string& acc_id, const float x, const float y, const float z);
    void setGamestate(const double game_time, const std::string& playmode);
    void setGroundtruthMyself(const double x, const double y, const double r, const double upx,
        const double upy, const double upz, const double z, const double fwx, const double fwy,
        const double fwz);
    void setGroundtruthBall(const double x, const double y, const double z);
    void setGroundtruthPlayer(const std::string& team, const float id, const double x,
        const double y, const double z);
    void addHearMsg(const float time, const float dir, const bool self, const std::string &msg);
    void setForce(const std::string name, const float x, const float y, const float z,
        const float vx, const float vy, const float vz);
    void setRobotPartPlayer(const std::string& team, const float id);
    void setRobotPartPercept(PartPercept::ROBOT_PART type, const float x, const float y,
        const float r);

  private:

    sexp_t *sx;
    pcont_t *cc;

    std::string ownTeamname;
    std::string oppTeamname;

    //time, gamestate
    double lastMessageTime;
    double messageTime;
    long frameNumber;
    int unum;
    TEAM_ID team;
    double gametime;
    std::string playmode;
    //sensors
    Vector3<double> acc, gyro;
    //vision
    std::vector<std::pair<Polar, Polar> > lines;
    std::map<std::string, Polar> staticVisionObjects;
    Polar ballPercept;
    bool ballPerceptUpdated;
    //joints
    JointValues jointValues;
    //groundtruth
    Pose2D groundtruthMyself;
    double groundtruthMyselfZ;
    Vector3<double> groundtruthMyselfUp;
    Vector3<double> groundtruthMyselfFw;
    Vector3<double> groundtruthBall3D;
    OtherRobotsData groundtruthOtherRobots;
    bool groundtruthUpdated;
    bool parseGroundtruth;
    //hear message
    std::vector<HearMessageData> hearMessages;
    //force vectors
    Vector3<double> forceContactLeft;
    Vector3<double> forceContactRight;
    Vector3<double> forceVectorLeft;
    Vector3<double> forceVectorRight;
    bool forceDataUpdatedLeft;
    bool forceDataUpdatedRight;
    //robot parts
    std::vector<PartPercept> robotPartsOwn;
    std::vector<PartPercept> robotPartsOpp;
    std::vector<PartPercept> robotPartsCurrent;

    /** Translator methods */
    const JOINT_ID translate_joint_id(const std::string& joint_id) const;
    const TEAM_ID translate_team_side(const std::string& team_id) const;
    const Gamestate::PLAYMODES translate_playmode(const std::string& pm,
        std::map<std::string, Gamestate::PLAYMODES>& map) const;
    /*    const TOUCH_ID translate_touch_id(const std::string& touch_id) const ;
     */

    /** Translation maps */
    std::map<std::string, JOINT_ID> joint_name_2_id;
    std::map<std::string, TEAM_ID> team_side_2_id;
    std::map<std::string, Gamestate::PLAYMODES> pm_name_2_id, pm_name_2_id2;
    /*    std::map<std::string,TOUCH_ID> touch_name_2_id;
     */
};

#endif