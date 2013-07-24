#ifndef SPECIAL_ACTION_STATE_H
#define SPECIAL_ACTION_STATE_H

#include <map>
#include "common/JointValues.h"

/**
 * This class stores the data for one state of a special action.
 */
class SpecialActionState
{

  public:

    SpecialActionState();

    /** Set the state for a single joint */
    void setJointState(const JOINT_ID& joint, const float& state);

    /** Set the time */
    void setTime(const float& value);

    /** Set the progress */
    void setProgress(const double& value);

    /** Get a single joint state */
    const float getJointState(const JOINT_ID& joint) const;

    /** Get all joint states */
    const std::map<JOINT_ID, float>& getJointStates() const;

    /** Get time */
    const float& getTime() const;

    /** Get progress */
    const double& getProgress() const;

  private:

    /** List containing the joint states */
    std::map<JOINT_ID, float> jointStates;

    /** Time for reaching the state */
    float time;

    /** Progress from 0 ... 1 */
    double progress;
};

#endif // SPECIAL_ACTION_STATE_H
