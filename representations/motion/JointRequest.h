#ifndef JOINTREQUEST_H
#define JOINTREQUEST_H

#include "kernel/Template.h"
#include "common/JointValues.h"

REPRESENTATION(JointRequest)

class JointRequest: public JointRequestBase
{
  public:

    JointValues values;

    bool hasAlreadySpeeds;

};

#endif

