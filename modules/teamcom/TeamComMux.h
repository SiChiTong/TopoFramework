#ifndef TEAMCOMMUX_H
#define TEAMCOMMUX_H

#include "kernel/Template.h"

#include "representations/teamcom/TeamComInfo.h"
#include "representations/teamcom/TeamComData.h"

MODULE(TeamComMux)
  REQUIRES(TeamComInfo)
  
  REQUIRES(TC_BeliefShareOut)
  //REQUIRES(TC_FormationOut)

  PROVIDES(TeamComDataOut)  
END_MODULE



class TeamComMux : public TeamComMuxBase
{
  public:
  
    void init();
    void update(TeamComDataOut& theTeamComDataOut);
 
  private:
  
    std::vector<const std::vector<BitBuffer*>*> alldata;
    
    int typeBits;
    unsigned int starttype;
    void addData(const std::vector<BitBuffer*> &vec, int type,
                 std::vector<BitBuffer*> &out);
    void deleteData(std::vector<BitBuffer*> &vec);  
};


#endif

