#ifndef SPECIALACTIONS_H
#define SPECIALACTIONS_H

#include "kernel/Template.h"

#include <map>
#include <string>

#include "representations/rcss/BeamRequest.h"
#include "representations/perception/FrameInfo.h"
#include "representations/perception/PlayerInfo.h"
#include "representations/motion/MotionRequest.h"
#include "representations/motion/SpecialActionsOutput.h"

#include "SpecialAction.h"


MODULE(SpecialActions)
  USES(BeamRequest) //reset if beaming
  REQUIRES(FrameInfo)
  REQUIRES(PlayerInfo)
  REQUIRES(MotionRequest)
  //
  PROVIDES(SpecialActionsOutput)
END_MODULE


/**
 * Inline prefix-increment-operator for vision objects
 * @warning Should be used with care...
 */
inline MotionRequest::SpecialAction&
operator++(MotionRequest::SpecialAction& action)
{
  return action = static_cast<MotionRequest::SpecialAction>(action + 1);
}

class SpecialActions: public SpecialActionsBase
{
  public:
    SpecialActions();
    void init();
    void update(SpecialActionsOutput& theSpecialActionsOutput);

  protected:
    void reset();
    void interpolateStates(double factor);
    void parseActions(const MotionRequest::SpecialAction& a, const std::string& fileName);

  private:
    // Map containing the filenames associated to their special action ID
    std::map<MotionRequest::SpecialAction, std::string> filenames;

    // Map containing the actions
    std::map<MotionRequest::SpecialAction, SpecialAction> repository;

    // the currently executed special action
    MotionRequest::SpecialAction currentAction;

    // the start time of current skill
    uint32_t skillStartTime;

    // the current time since start time
    uint32_t skillCurrentTime;

    SpecialActionState state;
    SpecialActionState nextState;
    SpecialActionState iState;

    /** List containing the joint states */
    std::map<JOINT_ID, float> jointStates;
    std::map<std::string, JOINT_ID> jointNameToJointID;
    std::map<int, std::string> jointNameOrder;

};


#endif

