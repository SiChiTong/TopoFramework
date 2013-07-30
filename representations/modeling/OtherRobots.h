#ifndef OTHERROBOTS_H
#define OTHERROBOTS_H

#include "kernel/Template.h"
#include "math/Pose2D.h"
#include "math/Vector3.h"

/** 
 * @class OtherRobotsData
 */
class OtherRobotsData
{
  public:
    enum
    {
      MAX_UNUM = 11
    };

    class State
    {
      public:
        int unum;
        Pose2D pose;
        Pose2D velocity;
        Vector3<double> upright;
        Pose2D relativePose;
        double confidence;

        bool fallen() const
        {
          return (upright.z < 0.15);
        }
    };

    std::vector<State> teammates;
    std::vector<State> opponents;
};

REPRESENTATION(OtherRobots)
class OtherRobots: public OtherRobotsBase, public OtherRobotsData
{
  public:

    void draw() const
    {
      for (std::vector<State>::const_iterator iter = teammates.begin(); iter != teammates.end();
          ++iter)
      {
        drawing.pose("OtherRobots.teammates", iter->pose, 0.3, 3, 150, 150, 255);
        drawing.line("OtherRobots.teammates", iter->pose.translation.x, iter->pose.translation.y, 0,
            iter->pose.translation.x, iter->pose.translation.y, (iter->upright * 2).z, 150, 150,
            255, 1);
        char unum[3] =
        { 0, 0, 0 };
        snprintf(unum, 2, "%d", iter->unum);
        drawing.annotation("OtherRobots.teammates", unum, iter->pose.translation.x,
            iter->pose.translation.y, 0.2, 255, 255, 255);
      }
      for (std::vector<State>::const_iterator iter = opponents.begin(); iter != opponents.end();
          ++iter)
      {
        drawing.pose("OtherRobots.opponents.pose", iter->pose, 0.3, 3, 255, 150, 150);
        drawing.line("OtherRobots.opponents.upright", iter->pose.translation.x,
            iter->pose.translation.y, 0, iter->pose.translation.x, iter->pose.translation.y,
            (iter->upright * 2).z, 255, 150, 150, 1);
      }
    }

};

REPRESENTATION(LocalOtherRobots)
class LocalOtherRobots: public LocalOtherRobotsBase, public OtherRobotsData
{
};

REPRESENTATION(TeamOtherRobots)
class TeamOtherRobots: public TeamOtherRobotsBase, public OtherRobotsData
{
};

#endif

