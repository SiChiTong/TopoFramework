/*
 * AugmentedSelfLocator.cpp
 *
 *  Created on: Oct 6, 2011
 *      Author: sam
 */

#include <iostream>
#include <cstdlib>
#include <algorithm>

#include "AugmentedSelfLocator.h"

using namespace std;

MAKE_MODULE(AugmentedSelfLocator)

// public
void AugmentedSelfLocator::init()
{
  config.setPersist(false);

  wSlow = wFast = 0.;
  alphaSlow = config.getValue("alphaSlow", .02);
  alphaFast = config.getValue("alphaFast", .5);
  alpha1 = config.getValue("alpha1", .004);
  alpha2 = config.getValue("alpha2", .1);
  alpha3 = config.getValue("alpha3", .004);
  alpha4 = config.getValue("alpha4", .001);

  supParticles = config.getValue("supParticles", 20);
  unifDist = 1. / supParticles;
  confidence = 1.;

  x.resize(supParticles, 0.);
  y.resize(supParticles, 0.);
  z.resize(supParticles, 0.);
  w.resize(supParticles, 0.);

  xn.resize(supParticles, 0.);
  yn.resize(supParticles, 0.);
  zn.resize(supParticles, 0.);

  c.resize(supParticles, 0.);
  u.resize(supParticles + 1, 0.);

  for (unsigned p = 0; p < supParticles; p++)
  {
    x[p] = theFieldDimensions->length * (drand48() - .5);
    y[p] = theFieldDimensions->width * (drand48() - .5);
    z[p] = 2. * M_PI * (drand48() - 0.5); //[-pi, pi]
    w[p] = unifDist;
  }

  preInit = false;
}

bool AugmentedSelfLocator::preExecute()
{
  if (!thePlayerInfo->isValid || !theFieldDimensions->initialized)
    return false;

  if (!preInit)
  {
    preInit = true;
    // Landmark initialization
    double fl = theFieldDimensions->halfLength; // field length x-axis
    double fw = theFieldDimensions->halfWidth; // field width  y-axis
    double gw = theFieldDimensions->halfGoalWidth; // this will be fixed soon

    if (thePlayerInfo->team == TEAM_RIGHT)
    {
      // F2R own
      flags.insert(make_pair(FLAG_OWN_LEFT, Vector2<double>(-fl, fw)));
      // F1R own
      flags.insert(make_pair(FLAG_OWN_RIGHT, Vector2<double>(-fl, -fw)));
      // F2L foe
      flags.insert(make_pair(FLAG_FOE_LEFT, Vector2<double>(fl, fw)));
      // F1L foe
      flags.insert(make_pair(FLAG_FOE_RIGHT, Vector2<double>(fl, -fw)));
      // ==================================================================
      // G2R own
      goals.insert(make_pair(GOALPOST_OWN_LEFT, Vector2<double>(-fl, gw)));
      // G1R own
      goals.insert(make_pair(GOALPOST_OWN_RIGHT, Vector2<double>(-fl, -gw)));
      // G2L foe
      goals.insert(make_pair(GOALPOST_FOE_LEFT, Vector2<double>(fl, gw)));
      // G2L foe
      goals.insert(make_pair(GOALPOST_FOE_RIGHT, Vector2<double>(fl, -gw)));
    }
    else
    {
      // F2R foe
      flags.insert(make_pair(FLAG_FOE_RIGHT, Vector2<double>(fl, -fw)));
      // F1R foe
      flags.insert(make_pair(FLAG_FOE_LEFT, Vector2<double>(fl, fw)));
      // F1L own
      flags.insert(make_pair(FLAG_OWN_RIGHT, Vector2<double>(-fl, -fw)));
      // F2L own
      flags.insert(make_pair(FLAG_OWN_LEFT, Vector2<double>(-fl, fw)));
      // =================================================================
      // G2R foe
      goals.insert(make_pair(GOALPOST_FOE_RIGHT, Vector2<double>(fl, -gw)));
      // G1R foe
      goals.insert(make_pair(GOALPOST_FOE_LEFT, Vector2<double>(fl, gw)));
      // G2L own
      goals.insert(make_pair(GOALPOST_OWN_RIGHT, Vector2<double>(-fl, -gw)));
      // G2R own
      goals.insert(make_pair(GOALPOST_OWN_LEFT, Vector2<double>(-fl, gw)));
    }
  }

  return preInit;
}

void AugmentedSelfLocator::execute()
{
#ifdef DEBUG_MODE
  draw();
#endif

  if (!preExecute())
    return;

  vector<vector<double> > percepts;
  // In degrees to be compatible with vision information
  double panAngle = theJointData->values[JID_HEAD_PAN].angle * 180. / M_PI;

  for (map<FLAG_ID, Polar>::const_iterator iter = theFlagPercept->flags.begin();
      iter != theFlagPercept->flags.end(); ++iter)
  {
    Polar p = iter->second;
    double dist = sqrt(max(p.distance * p.distance - 0.47 * 0.47, 0.)); // assumption
    if (!(dist >= 0)) //this detects nan, if p.distance is too small !
      dist = 0;
    double angle = (p.azimuth + panAngle) * M_PI / 180.;
    normalize(angle);
    vector<double> land;
    land.push_back(flags.find(iter->first)->second.x);
    land.push_back(flags.find(iter->first)->second.y);
    land.push_back(dist);
    land.push_back(angle);
    percepts.push_back(land);
  }

  for (map<GOALPOST_ID, Polar>::const_iterator iter = theGoalPercept->goalposts.begin();
      iter != theGoalPercept->goalposts.end(); ++iter)
  {
    Polar p = iter->second;
    double dist = sqrt(max((p.distance * p.distance - 0.47 * 0.47), 0.));
    if (!(dist >= 0)) //this detects nan, if p.distance is too small !
      dist = 0;
    double angle = (p.azimuth + panAngle) * M_PI / 180.;
    normalize(angle);
    vector<double> land;
    land.push_back(goals.find(iter->first)->second.x);
    land.push_back(goals.find(iter->first)->second.y);
    land.push_back(dist);
    land.push_back(angle);
    percepts.push_back(land);
  }

  wAvg = wTotal = 0.;
  for (unsigned m = 0; m < supParticles; m++)
  {
    predict(m);
    update(percepts, m);
    wAvg += unifDist * w[m];
    wTotal += w[m];
  }

  wTotal += 1e-10;
  wSlow += alphaSlow * (wAvg - wSlow);
  wSlow += 1e-10;
  wFast += alphaFast * (wAvg - wFast);

  for (unsigned i = 0; i < supParticles; i++)
    w[i] = w[i] / wTotal;

  // calculate the expected state
  meanPosition();

  //if (angleOut.is_open())
  //  angleOut << pose.rotation << " " << theGroundtruth->myself.rotation << endl;

  if (percepts.size() > 0)
  {
    systematicResampling(percepts);
    confidence = 1.0;
  }
  else
    confidence *= 0.95;

  prev = theOdometry->pose;

}
void AugmentedSelfLocator::update(RobotPose& theRobotPose)
{
  theRobotPose.pose = pose;
  theRobotPose.confidence = confidence;
}

void AugmentedSelfLocator::update(LocalRobotPose& theLocalRobotPose)
{
  theLocalRobotPose.pose = pose;
  theLocalRobotPose.confidence = confidence;
}

// private

void AugmentedSelfLocator::predict(const unsigned m)
{
  Pose2D curr = theOdometry->pose;
  Pose2D diff = curr - prev;
  double dRot1 = diff.translation.angle();
  double dTrans = diff.translation.abs();
  double dRot2 = normalize(normalize(curr.rotation) - (normalize(prev.rotation) + dRot1));
  double dRot1Hat = dRot1 + sampleNormDist(0, alpha1 * fabs(dRot1) + alpha2 * dTrans);
  double dTransHat = dTrans
      + sampleNormDist(0, alpha3 * dTrans + alpha4 * (fabs(dRot1) + fabs(dRot2)));
  double dRot2Hat = dRot2 + sampleNormDist(0, alpha1 * fabs(dRot2) + alpha2 * dTrans);

  x[m] = x[m] + dTransHat * cos(z[m] + dRot1Hat);
  y[m] = y[m] + dTransHat * sin(z[m] + dRot1Hat);
  z[m] = z[m] + dRot1Hat + dRot2Hat;
}
void AugmentedSelfLocator::update(vector<vector<double> >& percepts, const unsigned m)
{
  double pDet = 1., tmpX, tmpY, dHat/*, alphaHat*/;
  // using only distance
  if (percepts.size() == 0)
    return;

  for (unsigned i = 0; i < percepts.size(); i++)
  {
    tmpX = percepts[i][0] - x[m];
    tmpY = percepts[i][1] - y[m];
    dHat = sqrt(tmpX * tmpX + tmpY * tmpY);
    double alphaHat = normalize(atan2(percepts[i][1] - y[m], percepts[i][0] - x[m]) - z[m]);
    pDet *= (zeroMeanNormDist(dHat - percepts[i][2], 1.0)
        * zeroMeanNormDist(alphaHat - percepts[i][3], 1.5));
    //pDet *= zeroMeanNormDist(dHat - percepts[i][2], 1.);
  }
  w[m] = pDet;
}

void AugmentedSelfLocator::systematicResampling(vector<vector<double> >& percepts)
{

  c[0] = w[0];
  for (unsigned i = 1; i < supParticles; i++)
    c[i] = c[i - 1] + w[i];

  u[0] = drand48() / supParticles;
  unsigned i = 0;

  for (unsigned j = 0; j < supParticles; j++)
  {
    while (u[j] > c[i])
    {
      i++;
      if (i >= supParticles)
      {
        i = supParticles - 1;
        break;
      }
    }
    xn[j] = x[i];
    yn[j] = y[i];
    zn[j] = z[i];
    w[j] = unifDist;
    u[j + 1] = u[j] + unifDist;
  }

  if (percepts.size() > 1)
  {
    unsigned int idx = 0;
    for (unsigned j = 0; j < supParticles; j++)
    {
      if (max(0., 1. - wFast / wSlow) > drand48())
      {
        double temp[2] =
        { percepts[0][3], percepts[1][3] };
        addSample(percepts[0][0], percepts[0][1], percepts[1][0], percepts[1][1], percepts[0][2],
            percepts[1][2], temp, idx);
        ++idx;
      }
    }
  }

  // swap
  for (unsigned i = 0; i < supParticles; i++)
  {
    x[i] = xn[i];
    y[i] = yn[i];
    z[i] = zn[i];
    w[i] = unifDist;
  }

}

void AugmentedSelfLocator::meanPosition()
{
  // expected value
  double xx = 0, yy = 0, zzc = 0, zzs = 0;
  for (unsigned i = 0; i < supParticles; i++)
  {
    xx += x[i] * w[i];
    yy += y[i] * w[i];
    zzc += cos(z[i]);
    zzs += sin(z[i]);
  }

  pose.translation.x = xx;
  pose.translation.y = yy;
  pose.rotation = atan2(zzs, zzc);
}

int AugmentedSelfLocator::intersectionTest(double x1, double y1, double x2, double y2)
{
  int ret = -1;
  // change these magic numbers
  double x1Check1 = theFieldDimensions->halfLength;
  double y1Check1 = theFieldDimensions->halfWidth;
  if ((x1 >= -(x1Check1 + 0.5) && x1 <= (x1Check1 + 0.5))
      && (y1 >= -(y1Check1 + 0.5) && y1 <= (y1Check1 + 0.5))
      && (x2 >= -(x1Check1 + 0.5) && x2 <= (x1Check1 + 0.5))
      && (y2 >= -(y1Check1 + 0.5) && y2 <= (y1Check1 + 0.5)))
  { // offset 0.5
    double p1 = gaus1d(2, x1, pose.translation.x) * gaus1d(2, y1, pose.translation.y);
    double p2 = gaus1d(2, x2, pose.translation.x) * gaus1d(2, y2, pose.translation.y);
    if (p1 > p2)
      ret = 1;
    else if (p2 > p1)
      ret = 2;
    else
      ret = 3; // doing something is better than nothing
  }
  else
  {
    if ((x1 >= -(x1Check1 + 0.2) && x1 <= (x1Check1 + 0.2))
        && (y1 >= -(y1Check1 + 0.2) && y1 <= (y1Check1 + 0.2))) // offset 0.2
      ret = 1;
    if ((x2 >= -(x1Check1 + 0.2) && x2 <= (x1Check1 + 0.2))
        && (y2 >= -(y1Check1 + 0.2) && y2 <= (y1Check1 + 0.2))) // offset 0.2
      ret = 2;
  }
  return ret;
}

void AugmentedSelfLocator::addSample(double lx0, double ly0, double lx1, double ly1, double r0,
    double r1, double* angle, int index)
{

  double d = sqrt((lx0 - lx1) * (lx0 - lx1) + (ly0 - ly1) * (ly0 - ly1));
  if (d > r0 + r1)
  {
    return;
  }
  if (d < abs(r0 - r1))
  {
    return;
  }
  if (d == 0 && r0 == r1)
  {
    return;
  }
  double a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d);
  double h = sqrt(r0 * r0 - a * a);
  double x2 = lx0 + a * (lx1 - lx0) / d;
  double y2 = ly0 + a * (ly1 - ly0) / d;
  double x31 = x2 + h * (ly1 - ly0) / d;
  double x32 = x2 - h * (ly1 - ly0) / d;
  double y31 = y2 - h * (lx1 - lx0) / d;
  double y32 = y2 + h * (lx1 - lx0) / d;

  int test = intersectionTest(x31, y31, x32, y32);
  double confidenceOrientation = 0;

#ifdef DEBUG_MODE
  if (test > 0)
  {
    if (test == 1)
    drawing.circle("AugmentedSelfLocator.crile.inter1", x31, y31, 1.,
        0.2, 255, 0, 0);
    else
    drawing.circle("AugmentedSelfLocator.crile.inter2", x32, y32, 1.,
        0.2, 255, 255, 255);
  }
#endif

  switch (test)
  {
  case 1:
  {
    double alpha1 = atan2(ly0 - y31, lx0 - x31);
    confidenceOrientation = alpha1 - angle[0];
    normalize(confidenceOrientation);
    xn[index] = x31;
    yn[index] = y31;
    zn[index] = confidenceOrientation + sampleNormDist(0, 2.0) * M_PI / 180.0;
    w[index] = unifDist;
    break;
  }
  case 2:
  case 3:
  {
    double alpha1 = atan2(ly0 - y32, lx0 - x32);
    confidenceOrientation = alpha1 - angle[0];
    normalize(confidenceOrientation);
    xn[index] = x32;
    yn[index] = y32;
    zn[index] = confidenceOrientation + sampleNormDist(0, 2.0) * M_PI / 180.0;
    w[index] = unifDist;
    break;
  }
  }
}

void AugmentedSelfLocator::draw() const
{
  for (unsigned i = 0; i < supParticles; i++)
  {
    drawing.point("SelfLocator.xy", x[i], y[i], 0, 5, 0, 0, 0);
    drawing.line("SelfLocator.theta", x[i], y[i], 0, x[i] + 0.5 * cos(z[i]), y[i] + 0.5 * sin(z[i]),
        0, 255, 0, 0, 0.05);
  }
}

