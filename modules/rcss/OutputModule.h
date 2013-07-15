/*
 * OutputModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef OUTPUTMODULE_H_
#define OUTPUTMODULE_H_

#include "kernel/Framework.h"
#include "representations/rcss/JointValues.h"

MODULE(OutputModule)
REQUIRES(OutputJointValues)
END_MODULE

class OutputModule: public OutputModuleBase
{
  public:
    void execute();
};

#endif /* OUTPUTMODULE_H_ */
