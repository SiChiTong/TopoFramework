#ifndef BEHAVIOROUTPUTS_H
#define BEHAVIOROUTPUTS_H

#include "kernel/Template.h"
#include "representations/skills/SkillRequest.h"
#include "representations/rcss/BeamRequest.h"

class BehaviorOutput
{
  public:
    bool active;
    SkillRequest skillRequest;

    BeamRequest beamRequest;
};

REPRESENTATION(BeforeKickOffOutput)
class BeforeKickOffOutput: public BeforeKickOffOutputBase, public BehaviorOutput
{
};
REPRESENTATION(KickOffOwnOutput)
class KickOffOwnOutput: public KickOffOwnOutputBase, public BehaviorOutput
{
};
REPRESENTATION(FreekickOppOutput)
class FreekickOppOutput: public FreekickOppOutputBase, public BehaviorOutput
{
};
REPRESENTATION(GoalkickOppOutput)
class GoalkickOppOutput: public GoalkickOppOutputBase, public BehaviorOutput
{
};
REPRESENTATION(GoalkickOwnOutput)
class GoalkickOwnOutput: public GoalkickOwnOutputBase, public BehaviorOutput
{
};

// additional SPL gamestates
REPRESENTATION(InitialStateOutput)
class InitialStateOutput: public InitialStateOutputBase, public BehaviorOutput
{
};
REPRESENTATION(ReadyStateOutput)
class ReadyStateOutput: public ReadyStateOutputBase, public BehaviorOutput
{
};
REPRESENTATION(PenalizedOutput)
class PenalizedOutput: public PenalizedOutputBase, public BehaviorOutput
{
};

// Behaviors
REPRESENTATION(StrikerOutput)
class StrikerOutput: public StrikerOutputBase, public BehaviorOutput
{
};
REPRESENTATION(SupporterOutput)
class SupporterOutput: public SupporterOutputBase, public BehaviorOutput
{
};
REPRESENTATION(GoalieOutput)
class GoalieOutput: public GoalieOutputBase, public BehaviorOutput
{
};

#endif

