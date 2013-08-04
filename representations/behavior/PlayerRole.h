#ifndef PLAYERROLE_H
#define PLAYERROLE_H

#include "kernel/Template.h"

REPRESENTATION(PlayerRole)

class PlayerRole: public PlayerRoleBase
{
  public:

    enum ROLE
    {
      GOALIE, STRIKER, SUPPORTER, NUM_ROLES
    };

    ROLE role;
    Pose2D pose;
    bool closestToBall;

};

#endif

