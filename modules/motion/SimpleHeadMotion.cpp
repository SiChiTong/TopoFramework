#include "SimpleHeadMotion.h"

MAKE_MODULE(SimpleHeadMotion)

void SimpleHeadMotion::update(HeadMotionRequest& theHeadMotionRequest)
{
  //do not move the head, if robot is not walking or standing
  if (!(theMotionRequest->motion == MotionRequest::WALK
      || (theMotionRequest->motion == MotionRequest::SPECIAL_ACTION
          && theMotionRequest->specialActionRequest == MotionRequest::SPECIAL_STAND)
      || theMotionRequest->motion == MotionRequest::STAND))
  {
    theHeadMotionRequest.pan = 0;
    theHeadMotionRequest.tilt = 0;
    return;
  }

  Vector2<double> ball_position = theBallPos->relativePos;
  float dist_to_ball = ball_position.abs();
  float pan_angle_to_ball = atan2(ball_position.y, ball_position.x);

  //t_ball is the time to look at the ball and depends on the ball distance
  unsigned int t = 1024 * 2 - 1;
  unsigned int t_ball = std::min(1600.0, 1000.0 * std::max(0.0f, 3.0f - dist_to_ball));

  if (theBallPercept->updated)
    ball_visibility += (1.0 - ball_visibility) * 0.02;
  else
    ball_visibility += (0.0 - ball_visibility) * 0.02;

  uint32_t time = theFrameInfo->time_ms;
  if ( //fabs(pan_angle_to_ball)<M_PI_2 &&
  ball_visibility > 0.01 && (time & t) < t_ball)
  {
    //look at ball
    theHeadMotionRequest.pan = pan_angle_to_ball;
    theHeadMotionRequest.tilt = -atan2(dist_to_ball, 0.45f);
  }
  else
  {
    //look around
    theHeadMotionRequest.pan = (90.0 / 180.0 * M_PI) * sin(time * 6.28 / 4000.0);
    theHeadMotionRequest.tilt = 0.3 * cos(time * 6.28 / 4000.0) - 0.15;
  }

}

