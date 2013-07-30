/*
 * SimpleAgentLocator.cpp
 *
 *  Created on: Jul 29, 2013
 *      Author: sam
 */

#include "SimpleAgentLocator.h"

MAKE_MODULE(SimpleAgentLocator)

SimpleAgentLocator::~SimpleAgentLocator()
{
  for (TrackingObjects::iterator iter = teammates.begin(); iter != teammates.end(); ++iter)
  {
    delete iter->second;
  }

  for (TrackingObjects::iterator iter = opponents.begin(); iter != opponents.end(); ++iter)
  {
    delete iter->second;
  }
  teammates.clear();
  teammates.clear();
}

void SimpleAgentLocator::init()
{
  // All of the allocations are dynamic
}

void SimpleAgentLocator::init(const vector<PartPercept>& recvObjects, TrackingObjects& theObjects,
    bool ownTeam)
{
  for (vector<PartPercept>::const_iterator iter = recvObjects.begin(); iter != recvObjects.end();
      ++iter)
  {
    if (theObjects.find(iter->unum) == theObjects.end()
        && (!(thePlayerInfo->unum == iter->unum) || !ownTeam))
    {
      theObjects.insert(make_pair(iter->unum, new TheOtherAgent));
    }
  }
}

void SimpleAgentLocator::execute()
{

  init(theRobotPartPercept->teammates, teammates, true);
  init(theRobotPartPercept->opponents, opponents, false);

  predict(theRobotPartPercept->teammates, teammates, true);
  predict(theRobotPartPercept->opponents, opponents, false);

  update(teammates);
  update(opponents);

  prevRotation = theLocalRobotPose->pose.rotation;
  prevOdometry = theOdometry->pose;

}

void SimpleAgentLocator::update(OtherRobots& theOtherRobots)
{

  theOtherRobots.teammates.clear();
  theOtherRobots.opponents.clear();

  updateProvider(theOtherRobots.teammates, teammates);
  updateProvider(theOtherRobots.opponents, opponents);

}
void SimpleAgentLocator::update(LocalOtherRobots& theLocalOtherRobots)
{

  theLocalOtherRobots.teammates.clear();
  theLocalOtherRobots.opponents.clear();

  updateProvider(theLocalOtherRobots.teammates, teammates);
  updateProvider(theLocalOtherRobots.opponents, opponents);

}

void SimpleAgentLocator::updateProvider(vector<OtherRobots::State>& provider,
    TrackingObjects& theObjects)
{

  for (TrackingObjects::iterator iter = theObjects.begin(); iter != theObjects.end(); ++iter)
  {
    Vector3<double> avg(0, 0, 0);
    double k = 1e-5;
    for (Features::iterator iter2 = iter->second->featuresAbs.begin();
        iter2 != iter->second->featuresAbs.end(); ++iter2)
    {
      avg = (avg + Vector3<double>(iter2->second.x, iter2->second.y, 0.0f))
          + Vector3<double>(0, 0, 0.47);
      ++k;
    }
    avg = avg / k;

    OtherRobots::State state;
    state.confidence = 1.0
        / (1.0 + theFrameInfo->time - iter->second->lastSeen[PartPercept::HEAD].x);
    state.unum = iter->first;
    state.pose.translation = Vector2<double>(avg.x, avg.y);
    if (avg.z > 0.1)
      state.upright = Vector3<double>(0, 0, 0.5);
    else
      state.upright = Vector3<double>(0, 0, -0.5);

    if (state.confidence > 0.2)
    {
      provider.push_back(state);
      iter->second->isActive = true;
    }
    else
      iter->second->isActive = false;
  }

}

void SimpleAgentLocator::predict(const vector<PartPercept>& recvObjects,
    TrackingObjects& theObjects, bool ownTeam)
{

  // In degrees to be compatible with vision information
  float panAngle = theJointData->values[JID_HEAD_PAN].angle * 180. / M_PI;
  float tiltAngle = theJointData->values[JID_HEAD_TILT].angle * 180. / M_PI;

  Pose3D transHeadGivenTorso;
  transHeadGivenTorso.rotateZ(panAngle);
  transHeadGivenTorso.rotateY(-tiltAngle);

  Pose2D step = theOdometry->pose - prevOdometry;
  stepTranslationX = step.translation.x;
  stepTranslationY = step.translation.y;

  map<int, map<PartPercept::ROBOT_PART, bool> > theObjectFeatureStatus;

  for (vector<PartPercept>::const_iterator iter = recvObjects.begin(); iter != recvObjects.end();
      ++iter)
  {
    if (ownTeam && iter->unum == thePlayerInfo->unum)
      continue;

    theObjectFeatureStatus[iter->unum].insert(make_pair(iter->type, true));
    const Polar pol = iter->polar;
    Vector3<double> relativeAgent = pol.toVector();
    Vector3<double> pt = transHeadGivenTorso * relativeAgent;
    theObjects[iter->unum]->features[iter->type] = Vector2<double>(pt.x, pt.y);
    theObjects[iter->unum]->lastSeen[iter->type].x = theFrameInfo->time;
  }

  for (TrackingObjects::iterator iter = theObjects.begin(); iter != theObjects.end(); ++iter)
  {
    map<int, map<PartPercept::ROBOT_PART, bool> >::iterator findFeature =
        theObjectFeatureStatus.find(iter->first);
    if (findFeature == theObjectFeatureStatus.end())
    {
      // The agent is not seen at all so all the parts needs to be updated.
      for (Features::iterator iter2 = iter->second->features.begin();
          iter2 != iter->second->features.end(); ++iter2)
      {
        Vector2<double> basisPosTrans(step.translation);
        Vector2<double> basisPosChange(iter2->second);
        basisPosChange = basisPosChange.rotate(-step.rotation) - basisPosTrans;
        iter2->second = basisPosChange;
      }
    }
    else if (findFeature != theObjectFeatureStatus.end())
    {
      for (Features::iterator iter2 = iter->second->features.begin();
          iter2 != iter->second->features.end(); ++iter2)
      {
        if (findFeature->second.find(iter2->first) != findFeature->second.end())
          continue;

        Vector2<double> basisPosTrans(step.translation);
        Vector2<double> basisPosChange(iter2->second);
        basisPosChange = basisPosChange.rotate(-step.rotation) - basisPosTrans;
        iter2->second = basisPosChange;
      }
    }
    else
    {
      // TODO: update though the communication message
    }

  }
}

void SimpleAgentLocator::update(TrackingObjects& theObjects)
{
  for (TrackingObjects::iterator iter = theObjects.begin(); iter != theObjects.end(); ++iter)
  {
    Features& localFeatures = iter->second->features;
    Features& localFeaturesAbs = iter->second->featuresAbs;

    for (Features::iterator iter2 = localFeatures.begin(); iter2 != localFeatures.end(); ++iter2)
    {
      Vector2<double> absAgent = theLocalRobotPose->pose * iter2->second;
      localFeaturesAbs[iter2->first] += (absAgent - localFeaturesAbs[iter2->first]) * 0.5f ;
    }
  }

}
