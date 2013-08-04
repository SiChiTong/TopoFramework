#include "TeamComDemux.h"

MAKE_MODULE(TeamComDemux)

void TeamComDemux::init()
{
  typeBits = ceil(::log((double) TeamComData::NUM_TYPES) / ::log(2.0));
}

void TeamComDemux::deleteData(std::vector<BitBuffer*> &vec)
{
  for (std::vector<BitBuffer*>::iterator i = vec.begin(); i < vec.end(); i++)
    delete *i;
  vec.clear();
}

void TeamComDemux::execute()
{
  //delete data from pervious frame  
  deleteData(dataBeliefShare);
  deleteData(dataFormation);

  for (std::vector<BitBuffer*>::const_iterator iter = theTeamComDataIn->data.begin();
      iter < theTeamComDataIn->data.end(); iter++)
  {
    int type = BitInt(*(*iter), 0, typeBits);
    BitBuffer *buffer = new BitBuffer(*(*iter), typeBits, (*iter)->getSize() - typeBits);

    //put received buffer into a vector depending on type
    if (type == TeamComData::BELIEFSHARE)
      dataBeliefShare.push_back(buffer);
    if (type == TeamComData::FORMATION)
      dataFormation.push_back(buffer);
  }
}

void TeamComDemux::update(TC_BeliefShareIn& theTC_BeliefShareIn)
{
  theTC_BeliefShareIn.data = dataBeliefShare;
}

void TeamComDemux::update(TC_FormationIn& theTC_FormationIn)
{
  theTC_FormationIn.data = dataFormation;
}

