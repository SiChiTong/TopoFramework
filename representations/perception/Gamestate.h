#ifndef GAMESTATE_H
#define GAMESTATE_H

#include "kernel/Framework.h"

REPRESENTATION(Gamestate)

class Gamestate: public GamestateBase
{
  public:

    /** Enumeration for all possible playmodes during a regular game
     *  of simulated soccer */
    enum PLAYMODES
    {
      BEFORE_KICKOFF, SET = BEFORE_KICKOFF, PLAYON, // play
      GAMEOVER,
      // Start of game flow
      KICKOFF_OWN,      // 3
      KICKOFF_OPP,      // 4
      // Kick-In from side
      KICKIN_OWN,
      KICKIN_OPP,
      // Team managed to score a goal
      GOAL_OWN,
      GOAL_OPP,
      // Goalie needs to perform kickoff
      GOALKICK_OWN,
      GOALKICK_OPP,
      // Corner-Kick for team
      CORNERKICK_OWN,
      CORNERKICK_OPP,
      // Offside
      OFFSIDE_OWN,
      OFFSIDE_OPP,
      // freekick for team (due to offside position of other team)
      FREEKICK_OWN,
      FREEKICK_OPP,
      // initial state and ready state for SPL
      PREINITIAL,
      INITIAL,
      READY,         // walk to kickoff position
      NUM_PLAYMODES
    };

    Gamestate() :
        playmode(PREINITIAL), gametime(0), gameControllerRunning(false), penaltyShootout(false), firstHalf(
            false), scoreOwn(false), scoreOpp(false)
    {
    }

    PLAYMODES playmode;
    double gametime;

    //following values added for SPL, not set in 3D sim yet
    bool gameControllerRunning;

    bool penaltyShootout;
    bool firstHalf;
    int scoreOwn;
    int scoreOpp;
};

#endif

