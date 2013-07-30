/**
 * @file BallPos.h
 * Contains the RobotPose representation.
 */
#ifndef BALLPOS_H
#define BALLPOS_H

#include "kernel/Framework.h"
#include "math/Vector2.h"

/** 
 * @class BallPosData
 * Class to store an estimated ball position.
 * This class is extended by the different BallPos representations.
 */
class BallPosData
{
  public:

    /** The ball position relative to the robot. */
    Vector2<double> relativePos;

    /** The ball position in world coordinates (position on the field). */
    Vector2<double> absPos;

    double confidence;

    Vector2<double> position; /**< The position of the ball relative to the robot */
    Vector2<double> velocity; /**< The velocity of the ball relative to the robot */

    /**
     * Computes the position were a rolling ball is expected to stop rolling. (using uniform deceleration)
     * @return The position relative to the robot
     */
    Vector2<double> getEndPosition(float ballFriction) const
    {
      return position + velocity * (velocity.abs() / (2.f * ballFriction));
    }

};

REPRESENTATION(BallPos)
class BallPos: public BallPosBase, public BallPosData
{
  public:
    /** Draws absPos. */
    void draw() const
    {
      drawing.sphere("BallPos.BallPosRelative", relativePos.x, relativePos.y, 0.04, 0.04, 255, 100,
          100);
    }
};

REPRESENTATION(LocalBallPos)
class LocalBallPos: public LocalBallPosBase, public BallPosData
{
  public:

    /** Draws absPos. */
    void draw() const
    {
      drawing.sphere("LocalBallPos.BallPosAbsolute", absPos.x, absPos.y, 0.04, 0.04, 255, 255, 0);
    }

};

REPRESENTATION(TeamBallPos)
class TeamBallPos: public TeamBallPosBase, public BallPosData
{
  public:

};

REPRESENTATION(GoalieBallPos)
class GoalieBallPos: public GoalieBallPosBase, public BallPosData
{
  public:
    int timeReceived;


};

#endif

