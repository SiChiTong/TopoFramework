/*
 * AugmentedSelfLocator.cpp
 *
 *  Created on: Oct 6, 2011
 *      Author: sam
 */

#include "SelfLocator.h"

MAKE_MODULE(SelfLocator)

SelfLocator::SelfLocator() :
    alphaSlow(0), alphaFast(0), alpha1(0), alpha2(0), alpha3(0), alpha4(0), resamplingThreshold(0), standardDeviationDistance(
        0), standardDeviationAngle(0), standardDeviationSampleDistance(0), samples(0), poseCalculator(
        0), slowWeighting(0), fastWeighting(0), averageWeighting(0), totalWeighting(0), fieldBoundary(
        0), fieldDimensions(false)

{
}

SelfLocator::~SelfLocator()
{
  if (samples)
    delete samples;
  if (poseCalculator)
    delete poseCalculator;
  if (fieldBoundary)
    delete fieldBoundary;
}

// public
void SelfLocator::init()
{
  alphaSlow = config.getValue("alphaSlow", 0.0059f);
  alphaFast = config.getValue("alphaFast", 0.006f);

  alpha1 = config.getValue("alpha1", .01);
  alpha2 = config.getValue("alpha2", .01);
  alpha3 = config.getValue("alpha3", .0025);
  alpha4 = config.getValue("alpha4", .0025);

  resamplingThreshold = config.getValue("resamplingThreshold", 4);
  standardDeviationDistance = config.getValue("standardDeviationDistance", 0.4);
  standardDeviationAngle = config.getValue("standardDeviationAngle", 0.2);
  standardDeviationSampleDistance = config.getValue("standardDeviationSampleDistance", 0.10);

  samples = new SampleSet(config.getValue("sampleSetSize", 100));
  poseCalculator = new PoseCalculatorOverallAverage(*samples);

}

bool SelfLocator::preExecute()
{
  if (!theFieldDimensions->initialized || !thePlayerInfo->isValid)
  {
    return false;
  }

  if (!fieldDimensions && theFieldDimensions->initialized && thePlayerInfo->isValid)
  {
    fieldDimensions = true;

    // Field boundary
    fieldBoundary = new Boundary<double>(
        Range<double>(-theFieldDimensions->halfLengthPlusBorder,
            theFieldDimensions->halfLengthPlusBorder),
        Range<double>(-theFieldDimensions->halfWidthPlusBorder,
            theFieldDimensions->halfWidthPlusBorder));

    // Landmark initialization
    double fl = theFieldDimensions->halfLength; // field length x-axis
    double fw = theFieldDimensions->halfWidth; // field width  y-axis
    double gw = theFieldDimensions->halfGoalWidth; // this will be fixed soon

    if (thePlayerInfo->team == TEAM_RIGHT)
    {
      // F2R own
      flags.insert(std::make_pair(FLAG_OWN_LEFT, Vector2<double>(-fl, fw)));
      // F1R own
      flags.insert(std::make_pair(FLAG_OWN_RIGHT, Vector2<double>(-fl, -fw)));
      // F2L foe
      flags.insert(std::make_pair(FLAG_FOE_LEFT, Vector2<double>(fl, fw)));
      // F1L foe
      flags.insert(std::make_pair(FLAG_FOE_RIGHT, Vector2<double>(fl, -fw)));
      // ==================================================================
      // G2R own
      goals.insert(std::make_pair(GOALPOST_OWN_LEFT, Vector2<double>(-fl, gw)));
      // G1R own
      goals.insert(std::make_pair(GOALPOST_OWN_RIGHT, Vector2<double>(-fl, -gw)));
      // G2L foe
      goals.insert(std::make_pair(GOALPOST_FOE_LEFT, Vector2<double>(fl, gw)));
      // G2L foe
      goals.insert(std::make_pair(GOALPOST_FOE_RIGHT, Vector2<double>(fl, -gw)));
    }
    else
    {
      // F2R foe
      flags.insert(std::make_pair(FLAG_FOE_RIGHT, Vector2<double>(fl, -fw)));
      // F1R foe
      flags.insert(std::make_pair(FLAG_FOE_LEFT, Vector2<double>(fl, fw)));
      // F1L own
      flags.insert(std::make_pair(FLAG_OWN_RIGHT, Vector2<double>(-fl, -fw)));
      // F2L own
      flags.insert(std::make_pair(FLAG_OWN_LEFT, Vector2<double>(-fl, fw)));
      // =================================================================
      // G2R foe
      goals.insert(std::make_pair(GOALPOST_FOE_RIGHT, Vector2<double>(fl, -gw)));
      // G1R foe
      goals.insert(std::make_pair(GOALPOST_FOE_LEFT, Vector2<double>(fl, gw)));
      // G2L own
      goals.insert(std::make_pair(GOALPOST_OWN_RIGHT, Vector2<double>(-fl, -gw)));
      // G2R own
      goals.insert(std::make_pair(GOALPOST_OWN_LEFT, Vector2<double>(-fl, gw)));
    }
    samples->init();
    poseCalculator->init();

    for (SampleSet::iterator i = samples->begin(); i != samples->end(); ++i)
    {
      Sample* sample = *i;
      sample->translation.x = -theFieldDimensions->halfLength * drand48();
      sample->translation.y = theFieldDimensions->halfWidthPlusBorder * (2.0 * drand48() - 1.0);
      sample->rotation = normalize(2.0 * M_PI * (drand48() - 0.5));
      sample->weighting = 1.0;
      // Clip to field boundary
      fieldBoundary->clip(sample->translation);
    }

  }

  return theTorsoPose->translation.z > 0.30f;
}

void SelfLocator::execute()
{
#ifdef DEBUG_MODE
  draw();
#endif
  if (!preExecute())
    return;

  // Update all current particles with the motion model
  motionModel();
  // Update all current particles with sensor model
  sensorModel();
  // AMCL
  adaptWeightings();
  // Sample update
  nextSamplesUpdate();

  prevOdometry = theOdometry->pose;
}

void SelfLocator::motionModel()
{
  /**
   * Odometer reading from x_{t-1} to x_{t} is; w.r.t x_{t-1} coordinate
   * system there is a translation, and then rotation to obtain the new
   * coordinate system at x_{t}.
   */
  Pose2D curr = theOdometry->pose;
  curr.rotation = normalize(curr.rotation);
  Pose2D diff = curr - prevOdometry;
  double dRot1 = diff.translation.angle();
  double dTrans = diff.translation.abs();
  double dRot2 = normalize(curr.rotation - (normalize(prevOdometry.rotation) + dRot1));

  for (SampleSet::iterator iter = samples->begin(); iter != samples->end(); ++iter)
  {
    Sample* sample = *iter;
    double dRot1Hat = dRot1 + sampleNormalDistribution(alpha1 * fabs(dRot1) + alpha2 * dTrans);
    double dTransHat = dTrans
        + sampleNormalDistribution(alpha3 * dTrans + alpha4 * (fabs(dRot1) + fabs(dRot2)));
    double dRot2Hat = dRot2 + sampleNormalDistribution(alpha1 * fabs(dRot2) + alpha2 * dTrans);

    sample->translation.x += dTransHat * cos(sample->rotation + dRot1Hat);
    sample->translation.y += dTransHat * sin(sample->rotation + dRot1Hat);
    sample->rotation += dRot1Hat + dRot2Hat;
    sample->rotation = normalize(sample->rotation);

    // Clip to field boundary
    fieldBoundary->clip(sample->translation);
  }
}

Vector3<double> SelfLocator::toVector(const Polar& polar) const
{
  Vector3<double> vec;
  vec.x = polar.distance * std::cos(polar.elevation * M_PI / 180.0)
      * std::cos(polar.azimuth * M_PI / 180.0);
  vec.y = polar.distance * std::cos(polar.elevation * M_PI / 180.0)
      * std::sin(polar.azimuth * M_PI / 180.0);
  vec.z = polar.distance * std::sin(polar.elevation * M_PI / 180.0);
  return vec;
}

void SelfLocator::sensorModel()
{
  /* Debug
   Pose3D globalPose = Pose3D();

   Pose3D myRobotPose = Pose3D();
   myRobotPose.translate(robotPose.pose.translation.x, robotPose.pose.translation.y, 0.0f);
   myRobotPose.rotateZ(robotPose.pose.rotation);
   */

  Pose3D sensorPose = Pose3D();
  sensorPose.translate(0, 0.005, 0.09 + 0.065);
  sensorPose.rotateZ(theJointData->values[JID_HEAD_PAN].angle);
  sensorPose.rotateY(-theJointData->values[JID_HEAD_TILT].angle);

  Pose3D robotLocalPose = Pose3D();
  robotLocalPose.conc(*theTorsoPose).conc(sensorPose);

  /* Debug
   globalPose.conc(myRobotPose).conc(robotLocalPose);
   */
  selectedModels.clear();
  selectedObservations.clear();

  for (std::map<FLAG_ID, Polar>::const_iterator iter = theFlagPercept->flags.begin();
      iter != theFlagPercept->flags.end(); ++iter)
  {
    Vector2<double> modelVector = flags.find(iter->first)->second;
    Vector3<double> observationVector = robotLocalPose * toVector(iter->second);
    selectedModels.push_back(modelVector);
    selectedObservations.push_back(Vector2<double>(observationVector.x, observationVector.y));

    // Flags
    /* Debug
     Vector3<double> v0 = myRobotPose.translation;
     Vector3<double> v1 = globalPose.translation;
     Vector3<double> v2 = globalPose * toVector(iter->second);
     drawing.line("SelfLocator.sensorModel.flags", v0.x, v0.y, v0.z, v2.x, v2.y, 0, 255, 255, 0, 2);
     drawing.line("SelfLocator.sensorModel.flags", v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, 0, 0, 255, 2);
     std::stringstream ss;
     ss << "F=" << modelVector.x << ":" << modelVector.y;
     drawing.annotation("SelfLocator.sensorModel.flags", ss.str(), v2.x, v2.y, v2.z + 1, 0, 255,
     255);
     */
  }

  for (std::map<GOALPOST_ID, Polar>::const_iterator iter = theGoalPercept->goalposts.begin();
      iter != theGoalPercept->goalposts.end(); ++iter)
  {
    Vector2<double> modelVector = goals.find(iter->first)->second;
    Vector3<double> observationVector = sensorPose * toVector(iter->second);
    selectedModels.push_back(modelVector);
    selectedObservations.push_back(Vector2<double>(observationVector.x, observationVector.y));

    // Goals
    /* Debug
     Vector3<double> v0 = myRobotPose.translation;
     Vector3<double> v1 = globalPose.translation;
     Vector3<double> v2 = globalPose * toVector(iter->second);
     drawing.line("SelfLocator.sensorModel.flags", v0.x, v0.y, v0.z, v2.x, v2.y, 0, 255, 255, 0, 2);
     drawing.line("SelfLocator.sensorModel.goals", v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, 0, 0, 255, 2);
     std::stringstream ss;
     ss << "G=" << modelVector.x << ":" << modelVector.y;
     drawing.annotation("SelfLocator.sensorModel.goals", ss.str(), v2.x, v2.y, v2.z + 1, 0, 255,
     255);
     */
  }

  totalWeighting = 0.0f;
  for (unsigned int i = 0; i < selectedObservations.size(); i++)
  {
    for (SampleSet::iterator s = samples->begin(); s != samples->end(); ++s)
    {
      Sample* sample = *s;
      sample->weighting = computeWeightings(sample, selectedModels[i], selectedObservations[i]);
      totalWeighting += sample->weighting;
    }
  }
}

void SelfLocator::adaptWeightings()
{
  if (totalWeighting == 0)
    totalWeighting = 1.0;

  if (!selectedObservations.empty())
  {
    averageWeighting = totalWeighting / double(samples->size());
    slowWeighting = slowWeighting + alphaSlow * (averageWeighting - slowWeighting);
    fastWeighting = fastWeighting + alphaFast * (averageWeighting - fastWeighting);
  }
}

void SelfLocator::nextSamplesUpdate()
{
  if (!selectedObservations.empty())
    resampling();

  int lastIndexOfBestCluster = poseCalculator->getIndexOfBestCluster();
  poseCalculator->calculatePose(robotPose);
  if (lastIndexOfBestCluster != poseCalculator->getIndexOfBestCluster())
  {
    // @@<<TODO: needed to implement
  }

  // @@TODO: Fix me w.r.t to the covariance of the robot pose of the pose calculator
  if (!selectedObservations.empty())
    robotPose.confidence = 1.0;
  else
    robotPose.confidence *= 0.95;
}

void SelfLocator::resampling()
{
  // swap sample arrays
  SampleSet::Container* oldSet = samples->swap();
  const int numberOfSamples(samples->size());
  const double weightingsSum(totalWeighting);
  const double resamplingPercentage(
      fabs(slowWeighting) > 0 ? std::max(0.0, 1.0 - fastWeighting / slowWeighting) : 0.0f);
  // const double resamplingPercentage(1.0); // @@ debug value
  const double numberOfResampledSamples = numberOfSamples * (1.0f - resamplingPercentage);
  const double threshold = resamplingThreshold * weightingsSum / numberOfSamples;
  const double weightingBetweenTwoDrawnSamples(
      (weightingsSum + threshold * numberOfSamples) / numberOfResampledSamples);
  double nextPos(randomFloat() * weightingBetweenTwoDrawnSamples);
  double currentSum(0);

  // resample:
  int j(0);
  for (int i = 0; i < numberOfSamples; ++i)
  {
    currentSum += oldSet->at(i)->weighting + threshold;
    while (currentSum > nextPos && j < numberOfSamples)
    {
      // update
      samples->at(j)->set(oldSet->at(i));
      // update
      j++;
      nextPos += weightingBetweenTwoDrawnSamples;
    }
  }

  for (; j < numberOfSamples; ++j)
    generateTemplate(oldSet->at(j), samples->at(j));
}

void SelfLocator::generateTemplate(Sample* currentSample, Sample* nextSample)
{
  if (selectedObservations.size() >= 2)
    generateTemplateFromPosts(currentSample, nextSample);
  else
    nextSample->set(currentSample);
  nextSample->cluster = poseCalculator->getNewClusterIndex();
}

void SelfLocator::update(RobotPose& theRobotPose)
{
  theRobotPose.pose = robotPose.pose;
  theRobotPose.validity = robotPose.validity;
  theRobotPose.confidence = robotPose.confidence;
}

void SelfLocator::update(LocalRobotPose& theLocalRobotPose)
{
  theLocalRobotPose.pose = robotPose.pose;
  theLocalRobotPose.validity = robotPose.validity;
  theLocalRobotPose.confidence = robotPose.confidence;
}

// private
void SelfLocator::draw() const
{
  for (SampleSet::const_iterator i = samples->begin(); i != samples->end(); ++i)
  {
    const Sample s = *(*i);
    drawing.point("RobotPoseMisc.xy", s.translation.x, s.translation.y, 0, 3, 0, 0, 0);
    drawing.line("RobotPoseMisc.theta", s.translation.x, s.translation.y, 0.05,
        s.translation.x + 0.15 * cos(s.rotation), s.translation.y + 0.15 * sin(s.rotation), 0.05,
        255, 0, 0, 0.5);
  }
}

double SelfLocator::computeAngleWeighting(const Pose2D& robotPose,
    const Vector2<double>& modelPosition, const Vector2<double>& observedPosition)
{
  const Vector2<double> relativeModelPosition = robotPose.invert() * modelPosition;
  const double angleDifferance = abs(
      normalize(relativeModelPosition.angleToVector(observedPosition)));
  return gaussianProbability(angleDifferance, standardDeviationAngle);
}

double SelfLocator::computeDistanceWeighting(const Pose2D& robotPose,
    const Vector2<double>& modelPosition, const Vector2<double>& observedPosition)
{
  const float modelDistance = (robotPose.translation - modelPosition).abs();
  return gaussianProbability(abs(modelDistance - observedPosition.abs()), standardDeviationDistance);
}

double SelfLocator::computeWeightings(Sample* sample, const Vector2<double>& modelPosition,
    const Vector2<double>& observedPosition)
{
  const Pose2D robotPose(sample->rotation, sample->translation.x, sample->translation.y);
  return computeDistanceWeighting(robotPose, modelPosition, observedPosition)
      * computeAngleWeighting(robotPose, modelPosition, observedPosition);
}

void SelfLocator::generateTemplateFromPosts(Sample* currentSample, Sample* nextSample)
{
  Vector2<double> absoluteLeftPostion(selectedModels[0]);
  Vector2<double> observeLeftPosition(selectedObservations[0]);

  Vector2<double> absoluteRightPostion(selectedModels[selectedModels.size() - 1]);
  Vector2<double> observeRightPostion(selectedObservations[selectedObservations.size() - 1]);

  double leftPostDist = observeLeftPosition.abs();
  double leftDistUncertainty = sampleTriangularDistribution(standardDeviationSampleDistance);
  if (leftPostDist + leftDistUncertainty > standardDeviationSampleDistance)
    leftPostDist += leftDistUncertainty;
  double rightPostDist = observeRightPostion.abs();
  double rightDistUncertainty = sampleTriangularDistribution(standardDeviationSampleDistance);
  if (rightPostDist + rightDistUncertainty > standardDeviationSampleDistance)
    rightPostDist += rightDistUncertainty;

  Circle c1(absoluteLeftPostion, leftPostDist);
  Circle c2(absoluteRightPostion, rightPostDist);

  Vector2<double> p1, p2;
  if (getIntersectionOfCircles(c1, c2, p1, p2))
  {
    if (isInsideCarpet(p1))
    {
      float origAngle = (absoluteLeftPostion - p1).angle();
      float observedAngle = observeLeftPosition.angle();
      Pose2D templatePose(normalize(origAngle - observedAngle), p1);
      nextSample->set(templatePose.translation.x, templatePose.translation.y, templatePose.rotation,
          1.0f);
    }
    else if (isInsideCarpet(p2))
    {
      float origAngle = (absoluteLeftPostion - p2).angle();
      float observedAngle = observeLeftPosition.angle();
      Pose2D templatePose(normalize(origAngle - observedAngle), p2);
      nextSample->set(templatePose.translation.x, templatePose.translation.y, templatePose.rotation,
          1.0f);
    }
    else
    {
      nextSample->set(currentSample);
    }
  }
  else
  {
    nextSample->set(currentSample);
  }
}

int SelfLocator::getIntersectionOfCircles(const Circle& c0, const Circle& c1, Vector2<double>& p1,
    Vector2<double>& p2)
{
  /* Debug
   drawing.circle("SelfLocator.getIntersectionOfCircles", c0.center.x, c0.center.y, c0.radius, 2,
   255, 0, 0);
   drawing.circle("SelfLocator.getIntersectionOfCircles", c1.center.x, c1.center.y, c1.radius, 2, 0,
   255, 0);
   */

  double a, dx, dy, d, h, rx, ry;
  double x2, y2;

  /* dx and dy are the vertical and horizontal distances between
   * the circle centers.
   */
  dx = c1.center.x - c0.center.x;
  dy = c1.center.y - c0.center.y;

  /* Determine the straight-line distance between the centers. */
  d = sqrt((dy * dy) + (dx * dx));

  /* Check for solvability. */
  if (d > (c0.radius + c1.radius))
  {
    /* no solution. circles do not intersect. */
    return 0;
  }
  if (d < abs(c0.radius - c1.radius))
  {
    /* no solution. one circle is contained in the other */
    return 0;
  }

  /* 'point 2' is the point where the line through the circle
   * intersection points crosses the line between the circle
   * centers.
   */

  /* Determine the distance from point 0 to point 2. */
  a = ((c0.radius * c0.radius) - (c1.radius * c1.radius) + (d * d)) / (2.0f * d);

  /* Determine the coordinates of point 2. */
  x2 = c0.center.x + (dx * a / d);
  y2 = c0.center.y + (dy * a / d);

  /* Determine the distance from point 2 to either of the
   * intersection points.
   */
  h = sqrt((c0.radius * c0.radius) - (a * a));

  /* Now determine the offsets of the intersection points from
   * point 2.
   */
  rx = -dy * (h / d);
  ry = dx * (h / d);

  /* Determine the absolute intersection points. */
  p1.x = x2 + rx;
  p2.x = x2 - rx;
  p1.y = y2 + ry;
  p2.y = y2 - ry;

  return 1;
}

bool SelfLocator::isInsideCarpet(const Vector2<double> &p) const
{
  return fieldBoundary->isInside(p);
}
