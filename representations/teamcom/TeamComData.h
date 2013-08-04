#ifndef TEAMCOMDATA_H
#define TEAMCOMDATA_H

#include "kernel/Template.h"

#include "BitBuffer.h"
#include <vector>

class TeamComData
{
  public:
    enum
    {
      BELIEFSHARE, FORMATION, NUM_TYPES
    };

    std::vector<BitBuffer*> data;
};

// main send and receive data
REPRESENTATION(TeamComDataIn)
class TeamComDataIn: public TeamComDataInBase, public TeamComData
{
};
REPRESENTATION(TeamComDataOut)
class TeamComDataOut: public TeamComDataOutBase, public TeamComData
{
};

// BeliefShare data 
REPRESENTATION(TC_BeliefShareIn)
class TC_BeliefShareIn: public TC_BeliefShareInBase, public TeamComData
{
};
REPRESENTATION(TC_BeliefShareOut)
class TC_BeliefShareOut: public TC_BeliefShareOutBase, public TeamComData
{
};

// Formation data 
REPRESENTATION(TC_FormationIn)
class TC_FormationIn: public TC_FormationInBase, public TeamComData
{
};
REPRESENTATION(TC_FormationOut)
class TC_FormationOut: public TC_FormationOutBase, public TeamComData
{
};

#endif

