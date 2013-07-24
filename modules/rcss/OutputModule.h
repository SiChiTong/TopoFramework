/*
 * OutputModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef OUTPUTMODULE_H_
#define OUTPUTMODULE_H_

#include "kernel/Framework.h"
#include "kernel/Communication.h"
#include "representations/motion/JointRequestWithSpeeds.h"
#include "representations/rcss/SayMessage.h"
#include "representations/rcss/BeamRequest.h"
#include <sstream>

MODULE(OutputModule)
  REQUIRES(JointRequestWithSpeeds)
  REQUIRES(SayMessage)
  REQUIRES(BeamRequest)
END_MODULE

class OutputModule: public OutputModuleBase
{
  public:
    void init();
    void execute();

  private:
    void createJointMessage(std::stringstream &stream);
    void createSayMessage(std::stringstream &stream);
    void createBeamMessage(std::stringstream &stream);

};

#endif /* OUTPUTMODULE_H_ */
