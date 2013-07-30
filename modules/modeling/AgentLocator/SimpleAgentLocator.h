/*
 * SimpleAgentLocator.h
 *
 *  Created on: Jul 29, 2013
 *      Author: sam
 */

#ifndef SIMPLEAGENTLOCATOR_H_
#define SIMPLEAGENTLOCATOR_H_

#include <iostream>
#include <vector>
#include <map>

#include "kernel/Template.h"
#include "representations/perception/RobotPartPercept.h"
#include "representations/modeling/Odometry.h"
#include "representations/perception/PlayerInfo.h"
#include "representations/perception/JointData.h"
#include "representations/modeling/RobotPose.h"
#include "representations/perception/FrameInfo.h"
#include "representations/modeling/OtherRobots.h"

#include "math/Pose2D.h"
#include "math/Polar.h"
#include "math/Pose3D.h"

using namespace std;

MODULE(SimpleAgentLocator)
  REQUIRES(RobotPartPercept)
  REQUIRES(Odometry)
  REQUIRES(PlayerInfo)
  REQUIRES(JointData)
  REQUIRES(LocalRobotPose)
  REQUIRES(FrameInfo)
  PROVIDES(OtherRobots)
  PROVIDES(LocalOtherRobots)
END_MODULE

class SimpleAgentLocator: public SimpleAgentLocatorBase
{
  protected:

    typedef map<PartPercept::ROBOT_PART, Vector2<double> > Features;

    class TheOtherAgent
    {
      public:
        Features features, featuresAbs, lastSeen;
        bool isActive;

        TheOtherAgent() :
            isActive(false)
        {
        }
    };

    Pose2D prevOdometry;

    double prevRotation, stepTranslationX, stepTranslationY;

    typedef map<int, TheOtherAgent*> TrackingObjects;

    TrackingObjects teammates, opponents;

  public:

    virtual ~SimpleAgentLocator();
    void init();
    void execute();
    void update(OtherRobots& theOtherRobots);
    void update(LocalOtherRobots& theLocalOtherRobots);

  protected:
    void init(const vector<PartPercept>& recvObjects, TrackingObjects& theObjects, bool ownTeam);
    void predict(const vector<PartPercept>& recvObjects, TrackingObjects& theObjects, bool ownTeam);
    void update(TrackingObjects& theObjects);
    void updateProvider(vector<OtherRobots::State>& provider, TrackingObjects& theObjects);

};


#endif /* SIMPLEAGENTLOCATOR_H_ */
