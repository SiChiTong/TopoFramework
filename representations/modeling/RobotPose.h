/**
 * @file RobotPose.h
 * Contains the RobotPose representation.
 */
#ifndef ROBOTPOSE_H
#define ROBOTPOSE_H

#include "kernel/Framework.h"
#include "math/Pose2D.h"
#include "math/Vector3.h"

#include <vector>

/** 
 * @class RobotPoseData
 * A class that stores the 2d pose of the robot.
 */
class RobotPoseData
{
  public:

    /** The 2-dimensional robot pose (translation and rotation). */
    Pose2D pose;

    enum
    {
      unknownDeviation = 100000,
    };

    double validity; /**< The validity of the robot pose. (0 = invalid, 1 = perfect) */
    double confidence; /**< The deviation of the robot pose. */

    /** Constructor */
    RobotPoseData() :
        validity(0.0), confidence(1.0 - unknownDeviation)
    {
    }

};

REPRESENTATION(RobotPose)
class RobotPose: public RobotPoseBase, public RobotPoseData
{
  public:
    /** Draws the 2d pose. */
    void draw() const
    {
      drawing.pose("RobotPose", pose, 0.2, 3, 20, 20, 20);
    }
};

REPRESENTATION(LocalRobotPose)
class LocalRobotPose: public LocalRobotPoseBase, public RobotPoseData
{
};

REPRESENTATION(TeamRobotPose)
class TeamRobotPose: public TeamRobotPoseBase, public RobotPoseData
{
};

#endif

