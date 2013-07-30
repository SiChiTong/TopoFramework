#ifndef PLAYERINFO_H
#define PLAYERINFO_H

#include "kernel/Template.h"

REPRESENTATION(PlayerInfo)

enum TEAM_ID
{
  TEAM_LEFT, TEAM_RIGHT, NUM_TEAM_ID,

  TEAMCOLOR_BLUE = TEAM_LEFT, TEAMCOLOR_RED = TEAM_RIGHT
};

class PlayerInfo: public PlayerInfoBase
{
  public:

    PlayerInfo() :
        isValid(false), unum(0), team(TEAM_LEFT)
    {
    }
    bool isValid;

    int unum;

    TEAM_ID team;
    std::string teamname;

};

#endif

