#ifndef TEAMCOMINFO_H
#define TEAMCOMINFO_H

#include "kernel/Template.h"

REPRESENTATION(TeamComInfo)

class TeamComInfo: public TeamComInfoBase
{
  public:

    bool messageReceived;

    int senderUnum;
    Vector2<double> senderPos;
    double poseInconsistency;

};

#endif

