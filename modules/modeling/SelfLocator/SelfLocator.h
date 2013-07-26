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
#include "representations/modeling/Odometry.h"
#include "representations/perception/PlayerInfo.h"
#include "representations/modeling/TorsoPose.h"
#include "representations/modeling/RobotPose.h"


MODULE(SelfLocator)
  REQUIRES(FieldDimensions)
  REQUIRES(FlagPercept)
  REQUIRES(GoalPercept)
  REQUIRES(Odometry)
  REQUIRES(PlayerInfo)
  REQUIRES(JointData)
  REQUIRES(TorsoPose)
//
  PROVIDES(RobotPose)
  PROVIDES(LocalRobotPose)
END_MODULE

#include <vector>
#include <map>
#include <cmath>
#include <sstream>

#include "math/Vector3.h"
#include "math/Probabilistics.h"
#include "math/Boundary.h"

class Sample: public Pose2D
{
  public:
    double weighting;
    int cluster;

    Sample() :
        Pose2D(), weighting(1.0), cluster(0)
    {
    }

    void set(const double& x, const double& y, const double& z, const double& w)
    {
      translation.x = x;
      translation.y = y;
      rotation = z;
      weighting = w;
    }

    void set(const Sample* that)
    {
      set(that->translation.x, that->translation.y, that->rotation, that->weighting);
      cluster = that->cluster;
    }

};


class SampleSet
{
  public:
    typedef std::vector<Sample*> Container;
    typedef Container::iterator iterator;
    typedef Container::const_iterator const_iterator;
  protected:
    int nbSamples;
    Container* current;
    Container* other;

  public:
    SampleSet(const int nbSamples) :
        nbSamples(nbSamples), current(new Container()), other(new Container())
    {
    }

    ~SampleSet()
    {
      for (Container::iterator iter = current->begin(); iter != current->end(); ++iter)
        delete *iter;
      for (Container::iterator iter = other->begin(); iter != other->end(); ++iter)
        delete *iter;
      current->clear();
      other->clear();
      delete current;
      delete other;
    }

    void init()
    {
      for (int i = 0; i < nbSamples; i++)
      {
        Sample* curr = new Sample;
        Sample* next = new Sample;
        current->push_back(curr);
        other->push_back(next);
      }
    }

    iterator begin()
    {
      return current->begin();
    }

    const_iterator begin() const
    {
      return current->begin();
    }

    iterator end()
    {
      return current->end();
    }

    const_iterator end() const
    {
      return other->end();
    }

    /**
     * The function swaps the primary and secondary sample set.
     * @return The address of the previous sample set;
     */
    Container* swap()
    {
      Container* temp = current;
      current = other;
      other = temp;
      return other;
    }
    /**
     * Access operator.
     * @param index The index of the sample to access.
     */
    Sample*& at(int index)
    {
      return current->at(index);
    }

    /**
     * Constant access operator.
     * @param index The index of the sample to access.
     */
    Sample* const & at(int index) const
    {
      return current->at(index);
    }

    /**
     * Sample size.
     */
    int size() const
    {
      return nbSamples;
    }

};

class PoseCalculator
{
  public:
    SampleSet& samples;

    PoseCalculator(SampleSet& samples) :
        samples(samples)
    {
    }
    virtual ~PoseCalculator()
    {
    }

    virtual void calculatePose(RobotPose& robotPose) =0;

    virtual int getIndexOfBestCluster() const
    {
      return -1;
    }

    virtual int getNewClusterIndex()
    {
      return 0;
    }

    virtual void init()
    {
    }

};

class PoseCalculatorOverallAverage: public PoseCalculator
{
  public:
    PoseCalculatorOverallAverage(SampleSet& samples) :
        PoseCalculator(samples)
    {
    }

    virtual ~PoseCalculatorOverallAverage()
    {
    }
    void calculatePose(RobotPose& robotPose)
    {
      double xSum(0), ySum(0), cosSum(0), sinSum(0);
      double weightingsSum(0.0f);

      // Sum data off all samples:
      for (SampleSet::iterator iter = samples.begin(); iter != samples.end(); ++iter)
      {
        Sample* sample = *iter;
        xSum += sample->translation.x * sample->weighting;
        ySum += sample->translation.y * sample->weighting;
        cosSum += cos(sample->rotation);
        sinSum += sin(sample->rotation);
        weightingsSum += sample->weighting;
      }

      if (!weightingsSum)
        weightingsSum = 1.0; // Safe value

      double factorOne = 1.0 / weightingsSum;
      double factorTwo = 1.0 / double(samples.size());
      // Set the pose:
      robotPose.pose.translation.x = xSum * factorOne;
      robotPose.pose.translation.y = ySum * factorOne;
      robotPose.pose.rotation = atan2(sinSum, cosSum);
      robotPose.confidence = weightingsSum * factorTwo;
    }
};

/** Defines a circle by its center and its radius*/
class Circle
{
  public:
    Vector2<double> center;
    double radius;

    Circle() :
        radius(0)
    {
    }
    Circle(const Vector2<double>& center, double radius) :
        center(center), radius(radius)
    {
    }
};

class SelfLocator: public SelfLocatorBase
{
  protected:
    double alphaSlow;
    double alphaFast;

    double alpha1;
    double alpha2;
    double alpha3;
    double alpha4;

    double resamplingThreshold;
    double standardDeviationDistance;
    double standardDeviationAngle;
    double standardDeviationSampleDistance;

    // Sample set
    SampleSet* samples;
    PoseCalculator* poseCalculator;

    double slowWeighting;
    double fastWeighting;
    double averageWeighting;
    double totalWeighting;

    Boundary<double>* fieldBoundary;

    std::map<FLAG_ID, Vector2<double> > flags;
    std::map<GOALPOST_ID, Vector2<double> > goals;

    // Model, observation
    std::vector<Vector2<double> > selectedModels;
    std::vector<Vector2<double> > selectedObservations;

    RobotPose robotPose;
    Pose2D prevOdometry;
    bool fieldDimensions;

  public:
    SelfLocator();
    ~SelfLocator();
    void init();
    void execute();
    void update(RobotPose& theRobotPose);
    void update(LocalRobotPose& theLocalRobotPose);

  private:
    bool preExecute();
    void motionModel();
    void sensorModel();
    void adaptWeightings();
    void nextSamplesUpdate();
    void resampling();
    void generateTemplate(Sample* currentSample, Sample* nextSample);

    double computeAngleWeighting(const Pose2D& robotPose, const Vector2<double>& modelPosition,
        const Vector2<double>& observedPosition);
    double computeDistanceWeighting(const Pose2D& robotPose, const Vector2<double>& modelPosition,
        const Vector2<double>& observedPosition);
    double computeWeightings(Sample* sample, const Vector2<double>& modelPosition,
        const Vector2<double>& observedPosition);

    void generateTemplateFromPosts(Sample* currentSample, Sample* nextSample);
    int getIntersectionOfCircles(const Circle& c0, const Circle& c1, Vector2<double>& p1,
        Vector2<double>& p2);
    bool isInsideCarpet(const Vector2<double> &p) const;

    Vector3<double> toVector(const Polar& polar) const;
    void draw() const;

};

#endif /* AUGMENTEDSELFLOCATOR_H_ */
