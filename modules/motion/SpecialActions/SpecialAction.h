#ifndef SPECIAL_ACTION_H
#define SPECIAL_ACTION_H

#include <vector>
#include "SpecialActionState.h"

/**
 * This class stores the data for one special action.
 */
class SpecialAction
{

  public:

    SpecialAction();

    /** Add a state to the list */
    void addState(const SpecialActionState& state);

    /** Gets the next state */
    const SpecialActionState& getNextState(void);

    /** Checks if complete action is finished */
    const bool isFinished() const;

    /** Resets action */
    void reset();

    /** Get number of states */
    const int getStateCount();

    /** Get current index */
    const int getCurrentIndex();

    const unsigned int size() const;

  private:

    /** List containing the states */
    std::vector<SpecialActionState> states;

    /** Index of the action state to be executed next */
    unsigned int currentIndex;

    /** Flag set true if the action is completed */
    bool finished;

};

#endif // SPECIAL_ACTION_H
