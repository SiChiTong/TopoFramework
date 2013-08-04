/**
 * @file BallLocator.h
 *
 * This module provides the ball location using Extended Kalman Filter.
 * Please to Probabilistic Robotics for more information.
 *
 */
#ifndef BALLLOCATOR_H
#define BALLLOCATOR_H

#include "kernel/Template.h"

#include "representations/perception/BallPercept.h"
#include "representations/perception/JointData.h"
#include "representations/perception/FrameInfo.h"
#include "representations/modeling/TorsoPose.h"
#include "representations/modeling/Odometry.h"
#include "representations/modeling/RobotPose.h"
#include "representations/modeling/BallPos.h"

#include "utils/ekf/ekfilter.h"
#include "math/Pose3D.h"

MODULE(BallLocator)
  REQUIRES(BallPercept)
  REQUIRES(JointData)
  REQUIRES(FrameInfo)
  REQUIRES(TorsoPose)
  REQUIRES(Odometry)
  REQUIRES(LocalRobotPose)
//
  //PROVIDES(BallPos)
  PROVIDES(LocalBallPos)
END_MODULE

typedef Kalman::EKFilter<float, 1, false, true, false>::Vector KalmanFloatVector;
typedef Kalman::EKFilter<float, 1, false, true, false>::Matrix KalmanFloatMatrix;

class BallTracker: public Kalman::EKFilter<float, 1, false, true, false>
{
  public:
    BallTracker();
    virtual ~BallTracker()
    {
    }

  protected:
    void makeBaseA();
    void makeBaseH();
    void makeBaseV();
    void makeBaseR();
    void makeBaseW();
    void makeBaseQ();

    void makeQ();
    void makeA();
    void makeH();
    void makeProcess();
    void makeMeasure();

    float period, dd, dv;
  public:
    void setChange(float dd, float dv);
};



class BallLocator: public BallLocatorBase
{
  private:
    /**
     * We separately maps x and y axis relatively to the
     * robot camera position
     */
    BallTracker xTracker;
    BallTracker yTracker;

    Pose2D prevOdometry, prevOdometryForBall;
    Vector2<double> ballPosition;
    double timeBallLastSeen, prevRotation, kalmanLastBallSeen, stepTranslationX,
        stepTranslationY;

    // state variable
    Vector2<double> kRelBallPos, kAbsBallPos;

    double confidence;

  public:

    void init();
    void execute();
    void update(BallPos& theBallPos);
    void update(LocalBallPos& theLocalBallPos);

  private:
    bool preExecute();
    void update(Vector2<double>& rel, Vector2<double>& abs, double& conf);


};

#endif

