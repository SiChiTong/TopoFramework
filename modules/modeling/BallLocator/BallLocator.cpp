#include <iostream>
#include <cmath>

#include "BallLocator.h"

using namespace std;

MAKE_MODULE(BallLocator)

///
BallTracker::BallTracker() :
    period(0.02), dd(0), dv(0)
{
  // x, u, w, z, v
  setDim(2, 0, 2, 1, 1);
}

void BallTracker::makeBaseA()
{

  A(1, 1) = 1.0;
  A(1, 2) = period;
  A(2, 1) = 0.0;
  A(2, 2) = 1.0;
}

void BallTracker::makeA()
{
  // nothing
}

void BallTracker::makeBaseW()
{
  W(1, 1) = 1.0;
  W(1, 2) = 0.0;
  W(2, 1) = 0.0;
  W(2, 2) = 1.0;
}

void BallTracker::makeBaseQ()
{
  Q(1, 1) = 0.01;
  Q(1, 2) = 0.0;
  Q(2, 1) = 0.0;
  Q(2, 2) = 0.01;
}

void BallTracker::makeQ()
{
  //cout << "ekf_test = " << "Mapper_ekf_ball_tracker_x::makeQ()" << endl;
}

void BallTracker::makeBaseH()
{
  H(1, 1) = 1.0;
}

void BallTracker::makeH()
{
  // nothing
}

void BallTracker::makeBaseV()
{
  V(1, 1) = 1.0;
}

void BallTracker::makeBaseR()
{
  R(1, 1) = 0.001;
}

void BallTracker::makeMeasure()
{
  z(1) = x(1);
}

void BallTracker::makeProcess()
{
  KalmanFloatVector x_(x.size());
  x_(1) = dd + x(2) * period;
  x_(2) = dv;
  x.swap(x_);
}

void BallTracker::setChange(float dd, float dv)
{
  this->dd = dd;
  this->dv = dv;
}
///

void BallLocator::init()
{
  static const float _PX0[] =
  { 0.0001, 0.0001, 0.0001, 0.0001 };
  static const float _PY0[] =
  { 0.0001, 0.0001, 0.0001, 0.0001 };
  KalmanFloatVector x(2), y(2);
  KalmanFloatMatrix PX0(2, 2, _PX0), PY0(2, 2, _PY0);
  x(1) = x(2) = y(1) = y(2) = 0;
  xTracker.init(x, PX0);
  yTracker.init(y, PY0);

  prevRotation = kalmanLastBallSeen = 0.;
  confidence = 1.;
}

void BallLocator::execute()
{
  // In degrees to be compatible with vision information
  float panAngle = theJointData->values[JID_HEAD_PAN].angle * 180. / M_PI;
  float tiltAngle = theJointData->values[JID_HEAD_TILT].angle * 180. / M_PI;

  // TODO: this should be null or state variable should be given to indicate
  // whether updated or not.
  Polar pol = theBallPercept->polar;

  if (theBallPercept->updated)
  {
    //ball seen
    float ballDist = std::max(0.08, sqrt(max(pol.distance * pol.distance - (0.47 * 0.47), 0.0)));
    Vector2<double> ballObs(ballDist, 0);
    ballObs = ballObs.rotate((pol.azimuth + panAngle) * M_PI / 180.);
    ballPosition += (ballObs - ballPosition); //* 0.5;
    timeBallLastSeen = theFrameInfo->time;
  }

  //ball can't be inside robot
  float minBallDist = 0.1;
  if (fabs(ballPosition.x) < minBallDist && fabs(ballPosition.y) < minBallDist)
  {
    float angleToBall = ballPosition.angleToVector(Vector2<double>(1, 0));
    while (angleToBall > M_PI)
      angleToBall -= M_PI * 2.0;
    while (angleToBall < -M_PI)
      angleToBall += M_PI * 2.0;
    if (fabs(angleToBall) < M_PI / 4.0)
      ballPosition.x = minBallDist;
    else if (fabs(angleToBall) > M_PI / (3.0 * 4.0))
      ballPosition.x = -minBallDist;
    else if (fabs(angleToBall) < 0)
      ballPosition.y = minBallDist;
    else if (fabs(angleToBall) > 0)
      ballPosition.y = -minBallDist;
  }

  Pose3D transHeadGivenTorso = Pose3D();
  transHeadGivenTorso.rotateZ(panAngle * M_PI / 180.);
  transHeadGivenTorso.rotateY(-tiltAngle * M_PI / 180.);

  Pose2D step = theOdometry->pose - prevOdometryForBall;
  stepTranslationX = step.translation.x;
  stepTranslationY = step.translation.y;
  Vector2<double> basisPosTrans(step.translation.x, step.translation.y);
  Vector2<double> basisPosChange(xTracker.getX()(1), yTracker.getX()(1));
  // ball can not be inside the robot
  if (fabs(basisPosChange.x) < minBallDist && fabs(basisPosChange.y) < minBallDist)
  {
    float angleToBall = basisPosChange.angleToVector(Vector2<double>(1, 0));
    while (angleToBall > M_PI)
      angleToBall -= M_PI * 2.0;
    while (angleToBall < -M_PI)
      angleToBall += M_PI * 2.0;
    if (fabs(angleToBall) < M_PI / 4.0)
      ballPosition.x = minBallDist;
    else if (fabs(angleToBall) > M_PI / (3.0 * 4.0))
      ballPosition.x = -minBallDist;
    else if (fabs(angleToBall) < 0)
      ballPosition.y = minBallDist;
    else if (fabs(angleToBall) > 0)
      ballPosition.y = -minBallDist;
  }
  Vector2<double> basisVelChange(xTracker.getX()(2), yTracker.getX()(2));
  basisPosChange = basisPosChange.rotate(-step.rotation) - basisPosTrans;
  basisVelChange = basisVelChange.rotate(-step.rotation);
  xTracker.setChange(basisPosChange.x, basisVelChange.x);
  yTracker.setChange(basisPosChange.y, basisVelChange.y);

  KalmanFloatVector ux(1), uy(1), zx(1), zy(1);

  // debug
  if (theBallPercept->updated)
  {
    Vector3<double> relativeBall = pol.toVector();
    Vector3<double> anPnt = transHeadGivenTorso * relativeBall;
    ux(1) = uy(1) = 0;
    zx(1) = anPnt[0];
    zy(1) = anPnt[1];
    xTracker.step(ux, zx);
    yTracker.step(uy, zy);
    confidence = 1.;
    kalmanLastBallSeen = theFrameInfo->time;
  }
  else
  {
    ux(1) = uy(1) = 0;
    if ((theFrameInfo->time - kalmanLastBallSeen) > 3000)
    {
      // if the ball is not seen for 3 seconds; the current heuristic of
      // the listen ball position is used
      zx(1) = ballPosition.x;
      zy(1) = ballPosition.y;
      xTracker.step(ux, zx);
      yTracker.step(uy, zy);
      confidence *= 0.9;
    }
    else
    {
      xTracker.timeUpdateStep(ux);
      yTracker.timeUpdateStep(uy);
      confidence *= 0.95;
    }
  }

  /////////////////
  kAbsBallPos = theLocalRobotPose->pose * Vector2<double>(xTracker.getX()(1), yTracker.getX()(1));
  kRelBallPos = Vector2<double>(xTracker.getX()(1), yTracker.getX()(1));

  prevRotation = theLocalRobotPose->pose.rotation;
  prevOdometryForBall = theOdometry->pose;
}

void BallLocator::update(Vector2<double>& relativePos, Vector2<double>& absPos, double& conf)
{
  relativePos = kRelBallPos;
  absPos = kAbsBallPos;
  conf = confidence;
}

void BallLocator::update(BallPos& theBallPos)
{
  update(theBallPos.relativePos, theBallPos.absPos, theBallPos.confidence);
}

void BallLocator::update(LocalBallPos& theLocalBallPos)
{
  update(theLocalBallPos.relativePos, theLocalBallPos.absPos, theLocalBallPos.confidence);
}

