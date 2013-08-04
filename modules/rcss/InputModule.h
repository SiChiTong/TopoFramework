/*
 * InputModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef INPUTMODULE_H_
#define INPUTMODULE_H_

#include "kernel/Template.h"
#include "kernel/Communication.h"
#include "representations/rcss/ServerMessage.h"
#include "representations/rcss/FieldDimensions.h"
#include "representations/skills/SkillKickParameters.h"

MODULE(InputModule)
  PROVIDES(ServerMessage)
  PROVIDES(FieldDimensions)
  PROVIDES(SkillKickParameters)
END_MODULE

class InputModule: public InputModuleBase
{
  public:
    InputModule();
    ~InputModule();
    void init();
    void update(ServerMessage& theServerMessage);
    void update(FieldDimensions& theFieldDimensions);
    void update(SkillKickParameters& theSkillKickParameters);

  private:
    bool connected;
    std::string initTeamname;
    int initUnum;
};

#endif /* INPUTMODULE_H_ */
