#ifndef FORCEDATA_H
#define FORCEDATA_H

#include "kernel/Framework.h"
#include "math/Vector3.h"

REPRESENTATION(ForceData)

class ForceData: public ForceDataBase
{
  public:

    /** force left foot */
    Vector3<double> contactLeft;
    Vector3<double> vectorLeft;
    bool updatedLeft;

    /** force right foot */
    Vector3<double> contactRight;
    Vector3<double> vectorRight;
    bool updatedRight;

};

#endif

