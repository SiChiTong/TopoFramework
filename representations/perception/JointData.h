#ifndef JOINTDATA_H
#define JOINTDATA_H

#include "kernel/Template.h"
#include "common/JointValues.h"

REPRESENTATION(JointData)
class JointData: public JointDataBase
{
  public:
    JointValues values;
};

REPRESENTATION(RawJointData)
class RawJointData: public RawJointDataBase
{
  public:
    JointValues values;
};

#endif

