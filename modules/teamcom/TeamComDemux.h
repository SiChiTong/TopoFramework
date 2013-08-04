#ifndef TEAMCOMDEMUX_H
#define TEAMCOMDEMUX_H

#include "kernel/Template.h"

#include "representations/teamcom/TeamComInfo.h"
#include "representations/teamcom/TeamComData.h"

MODULE(TeamComDemux)
  REQUIRES(TeamComInfo)
  REQUIRES(TeamComDataIn)
  
  PROVIDES(TC_BeliefShareIn)
  PROVIDES(TC_FormationIn)
  
END_MODULE


class TeamComDemux : public TeamComDemuxBase
{
  private:
    int typeBits;
    void deleteData(std::vector<BitBuffer*> &vec);
  public:
  
    void init();
    void execute();

    std::vector<BitBuffer*> dataBeliefShare;
    void update(TC_BeliefShareIn& theTC_BeliefShareIn);
     
    std::vector<BitBuffer*> dataFormation;
    void update(TC_FormationIn& theTC_FormationIn);
    
};


#endif

