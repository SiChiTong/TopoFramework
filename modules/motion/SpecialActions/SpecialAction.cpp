#include "SpecialAction.h"
#include <cstdio>

SpecialAction::SpecialAction() :
    currentIndex(0), finished(false)
{
}

void SpecialAction::addState(const SpecialActionState& state)
{
  states.push_back(state);
}

const SpecialActionState& SpecialAction::getNextState()
{
  if ((currentIndex < states.size()) && (!finished))
  {
    currentIndex++;
    if (currentIndex == states.size())
    {
      currentIndex = 0;
      finished = true;
      return states[states.size() - 1];
    }
    else
    {
      return states[currentIndex - 1];
    }
  }
  else if (finished)
  {
    printf("Action finished, you must reset first!\n");
  }
  else
  {
    printf("No state available for this action!\n");
  }
  return states[currentIndex];
}

const bool SpecialAction::isFinished() const
{
  return finished;
}

void SpecialAction::reset(void)
{
  finished = false;
}

const int SpecialAction::getStateCount()
{
  return states.size();
}

const int SpecialAction::getCurrentIndex()
{
  return currentIndex;
}

const unsigned int SpecialAction::size() const
{
  return states.size();
}

