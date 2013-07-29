/**
 * @file AccEvaluator.cpp
 *
 * Implements the module AccEvaluator.
 *
 * @author Andreas Seekircher <aseek@cs.miami.edu>
 * @author Sam Abeyruwan <saminda@cs.miami.edu>
 */
#include "AccEvaluator.h"

//#include <cstdio>

MAKE_MODULE(AccEvaluator)

/**
 * Creates UprightVec and FallState (copied from old code)
 */
void AccEvaluator::execute()
{
  Vector3<double> acc(theSensorData->acc);
  acc.x *= -1; //when acc by gravity points back, robot falls to the front
  acc.y *= -1;

  //sometimes the server sends NaN...
  if (!(acc.x > -10000 && acc.x < 10000))
    acc.x = 0;
  if (!(acc.y > -10000 && acc.y < 10000))
    acc.y = 0;
  if (!(acc.z > -10000 && acc.z < 10000))
    acc.z = 0;

  vec += (acc - vec) * 0.05;

  if (vec.abs() < 5)
  {
    angleX = 0;
    angleY = 0;
    orientation = ORIENTATION_STANDING;
    return;
  }

  angleX = -atan2(vec.y, vec.z);
  angleY = -atan2(vec.x, vec.z);

  //debug.drawing.line("AccEvaluator.vec", 0,0,0, vec.x/5, vec.y/5, vec.z/5, 255,255,255, 10);

  const float thresholdFalling = 0.8;
  const float thresholdLying = 1.2;

  ORIENTATION prevOrientation = orientation;
  if (theFrameInfo->time_ms - timeLastFall > 3000.0)
    orientation = ORIENTATION_STANDING;
  if (vec.abs() > 6.0)
  {
    if (fabs(angleX) > thresholdFalling && fabs(angleX) > fabs(angleY))
    {
      if (fabs(angleX) > thresholdLying)
        orientation = ORIENTATION_LYING_SIDEWAYS;
      else
        orientation = ORIENTATION_FALLING_SIDEWAYS;
    }
    if (angleY > thresholdFalling)
    {
      if (angleY > thresholdLying)
        orientation = ORIENTATION_LYING_BACK;
      else
        orientation = ORIENTATION_FALLING_BACK;
    }
    if (angleY < -thresholdFalling)
    {
      if (angleY < -thresholdLying)
        orientation = ORIENTATION_LYING_FORWARD;
      else
        orientation = ORIENTATION_FALLING_FORWARD;
    }
  }
  if (orientation != ORIENTATION_STANDING && prevOrientation == ORIENTATION_STANDING)
    timeLastFall = theFrameInfo->time_ms;

}

void AccEvaluator::update(FallState& theFallState)
{
  //log << "orientation = " << orientation << std::endl;
  theFallState.orientation = orientation;
  theFallState.fallen = (orientation != ORIENTATION_STANDING);
}

void AccEvaluator::update(UprightVec& theUprightVec)
{
  theUprightVec.vec = vec;
}

void AccEvaluator::update(Odometry& theOdometry)
{
  const long time = theFrameInfo->time_ms;
  double timeSinceLastFrame = (double) (time - timePrev) / 1000.0;
  if (timeSinceLastFrame > 0.1)
    timeSinceLastFrame = 0.02; //something is wrong, use default timestep
  timePrev = time;

  odometryRotation += theSensorData->gyro.z * timeSinceLastFrame;

  theOdometry.pose = theWalkingEngineOutput->odometry;
  theOdometry.pose.rotation = odometryRotation;
}

void AccEvaluator::update(TorsoPose& theTorsoPose)
{
  //create rough torso pose with the values available in this module
  // theTorsoPose.translation = Vector3<double>(0, 0,
  //    (orientation == ORIENTATION_STANDING ? 0.35 : 0.05));

  //create rotation from the upright vector  
  const double angleY = atan2(vec.x, vec.z);
  //theTorsoPose.rotation = RotationMatrix::fromRotationY(angleY);
  const Vector3<double> upRot = RotationMatrix::fromRotationY(-angleY) * vec;
  const double angleX = atan2(upRot.y, upRot.z);
  //theTorsoPose.rotation.rotateX(angleX);
  theTorsoPose.rotation = RotationMatrix::fromRotationY(angleY);
  theTorsoPose.rotation.rotateX(angleX);

  //get the ankle joint positions relative to torso from forward kinematics
  const Pose3D& ankleLeft = theFKModel->partPositions[RobotModel::footLeft];
  const Pose3D& ankleRight = theFKModel->partPositions[RobotModel::footRight];
  const Vector3<double> footOffset(0,0,0); //TODO simspark footheight offset?

  const Vector3<double> footLeft = theTorsoPose.rotation * (ankleLeft * footOffset);
  const Vector3<double> footRight = theTorsoPose.rotation * (ankleRight * footOffset);

  //get position between feet with z from lower foot
  Vector3<double> feetMiddle = (footLeft + footRight) * 0.5;
  feetMiddle.z = std::min(footLeft.z, footRight.z);

  theTorsoPose.translation = feetMiddle * -1.0;

}

