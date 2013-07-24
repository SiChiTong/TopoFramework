#include "SpecialActionState.h"
#include <iostream>

SpecialActionState::SpecialActionState(void) :
    time(0.0f), progress(0.0f)
{
}

void SpecialActionState::setJointState(const JOINT_ID& joint, const float& state)
{
  jointStates[joint] = state;
}

void SpecialActionState::setTime(const float& value)
{
  time = value;
}

void SpecialActionState::setProgress(const double& value)
{
  progress = value;
}

const float SpecialActionState::getJointState(const JOINT_ID& joint) const
{
  std::map<JOINT_ID, float>::const_iterator iter = jointStates.find(joint);
  if (iter != jointStates.end())
  {
    return iter->second;
  }
  else
  {
    return 0.0f;
  }
}

const std::map<JOINT_ID, float>& SpecialActionState::getJointStates() const
{
  return jointStates;
}

const float& SpecialActionState::getTime() const
{
  return time;
}

const double& SpecialActionState::getProgress() const
{
  return progress;
}

