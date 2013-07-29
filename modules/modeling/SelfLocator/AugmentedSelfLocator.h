/*
 * AugmentedSelfLocator.h
 *
 *  Created on: Oct 6, 2011
 *      Author: sam
 */

#ifndef AUGMENTEDSELFLOCATOR_H_
#define AUGMENTEDSELFLOCATOR_H_

#include "kernel/Framework.h"

#include "representations/rcss/FieldDimensions.h"
#include "representations/perception/FlagPercept.h"
#include "representations/perception/GoalPercept.h"
#include "representations/perception/JointData.h"
#include "representations/modeling/UprightVec.h"
#include "representations/modeling/RobotPose.h"
#include "representations/modeling/Odometry.h"
#include "representations/perception/PlayerInfo.h"
#include "representations/perception/Groundtruth.h"

MODULE(AugmentedSelfLocator)
  REQUIRES(FieldDimensions)
  REQUIRES(FlagPercept)
  REQUIRES(GoalPercept)
  REQUIRES(UprightVec)
  REQUIRES(Odometry)
  REQUIRES(PlayerInfo)
  REQUIRES(JointData)
  REQUIRES(Groundtruth)
  PROVIDES(RobotPose)
  PROVIDES(LocalRobotPose)
END_MODULE

#include <vector>
#include <map>
#include <cmath>
#include <fstream>

#include "math/Vector3.h"
#include "math/Common.h"

using namespace std;

class AugmentedSelfLocator: public AugmentedSelfLocatorBase
{
  protected:
    double wSlow, wFast, wAvg, alphaSlow, alphaFast, wTotal, alpha1, alpha2,
        alpha3, alpha4;

    unsigned supParticles;
    double unifDist, confidence;

    map<FLAG_ID, Vector2<double> > flags;
    map<GOALPOST_ID, Vector2<double> > goals;

    /**
     * Data structures that holds the particle hypothesis
     */
    vector<double> x, y, z, w;
    vector<double> xn, yn, zn;

    vector<double> c;
    vector<double> u;

    Pose2D pose, prev, next;
    bool preInit;

  public:
    void init();
    void execute();
    void update(RobotPose& theRobotPose);
    void update(LocalRobotPose& theLocalRobotPose);

  private:
    bool preExecute();
    void predict(const unsigned m);
    void update(vector<vector<double> >& percepts, const unsigned m);
    void systematicResampling(vector<vector<double> >& percepts);
    void meanPosition();

    /**
     * Box-Muller transform to sample
     */
    double sampleNormDist(double mean, double stddev)
    {
      static double n2 = 0.0;
      static int n2_cached = 0;
      if (!n2_cached)
      {
        double x, y, r;
        do
        {
          x = 2.0 * rand() / RAND_MAX - 1;
          y = 2.0 * rand() / RAND_MAX - 1;

          r = x * x + y * y;
        } while (r == 0.0 || r > 1.0);
        {
          double d = sqrt(-2.0 * std::log(r) / r);
          double n1 = x * d;
          n2 = y * d;
          double result = n1 * stddev + mean;
          n2_cached = 1;
          return result;
        }
      }
      else
      {
        n2_cached = 0;
        return n2 * stddev + mean;
      }
    }

    double zeroMeanNormDist(double a, double b2)
    {
      return 1. / sqrt(2. * M_PI * b2 * b2) * exp(-.5 * a * a / b2) + 1e-5;
    }

    double gaus1d(double sd, double tv, double ev)
    {
      return 1. / sqrt(2. * M_PI * sd * sd)
          * exp(-.5 * (tv - ev) * (tv - ev) / (sd * sd)) + 1e-5;
    }

    void addSample(double, double, double, double, double, double, double*,
        int index);

    /**
     * return -1 if the circle intersection is not possible or out of ground
     * return 1 if (x1,y1) is possible
     * return 2 if (x2,y2) is possible
     */
    int intersectionTest(double, double, double, double);

    void draw() const;

};

#endif /* AUGMENTEDSELFLOCATOR_H_ */
