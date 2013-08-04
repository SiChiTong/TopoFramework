#include "TeamComMux.h"

MAKE_MODULE(TeamComMux)

void TeamComMux::init()
{
  typeBits = ceil(::log((double) TeamComData::NUM_TYPES) / ::log(2.0));
  starttype = 0;

  alldata.push_back(&theTC_BeliefShareOut->data);
  //alldata.push_back(&theTC_FormationOut->data);

}

void TeamComMux::update(TeamComDataOut& theTeamComDataOut)
{
  //delete data from last frame
  deleteData(theTeamComDataOut.data);
  //add new data
  unsigned int t = starttype;
  do
  {
    addData(*alldata[t], t, theTeamComDataOut.data);
    t = (t + 1) % alldata.size();
  } while (t != starttype);
  //next time start with another type
  starttype = (starttype + 1) % alldata.size();
}

void TeamComMux::addData(const std::vector<BitBuffer*> &vec, int type, std::vector<BitBuffer*> &out)
{
  for (std::vector<BitBuffer*>::const_iterator iter = vec.begin(); iter < vec.end(); iter++)
  {
    BitBuffer *buffer = new BitBuffer((*iter)->getSize() + typeBits);
    BitInt(*buffer, 0, typeBits) = type;
    for (int i = 0; i < (*iter)->getSize(); i++)
      buffer->setBit(typeBits + i, (*iter)->getBit(i));
    out.push_back(buffer);
  }
}

void TeamComMux::deleteData(std::vector<BitBuffer*> &vec)
{
  for (std::vector<BitBuffer*>::iterator i = vec.begin(); i < vec.end(); i++)
    delete *i;
  vec.clear();
}
