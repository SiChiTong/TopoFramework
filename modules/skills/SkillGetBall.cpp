#include "SkillGetBall.h"

MAKE_MODULE(SkillGetBall)

void SkillGetBall::update(SkillGetBallOutput& theSkillGetBallOutput)
{
  if (!(theSkillGetBallOutput.active = theSkillRequest->skill == SkillRequest::GETBALL))
    return;

  const Vector2<double> &target_point = theSkillRequest->target.translation;

  // extract ball and myself position
  const Vector2<double> &ball_position = theBallPos->absPos;
  const Vector2<double> &my_position = theRobotPose->pose.translation;
  Vector2<double> myself_to_ball = ball_position - my_position;

// ----------------------- FIND CLOSEST OPP for intercept and foot selection

//--------------------------------  
// from here GETBALL

//check if it is necessary to walk around ball
//Vector2d footpos = my_position + Vector2d(0,(left?0.04:-0.04)).rotate_rad(my_direction);
  float angle = (ball_position - my_position).angleToVector(target_point - ball_position);
  if (angle > M_PI)
    angle -= M_PI * 2.0;
  float walk_around_correction = 0;
  if (angle > M_PI_4)
    walk_around_correction = (angle - M_PI_4);
  if (angle < -M_PI_4)
    walk_around_correction = (angle + M_PI_4);
  float walk_around_correction_dist = 0;
  if (angle > M_PI_4 * 0.25)
    walk_around_correction_dist = (angle - M_PI_4 * 0.25);
  if (angle < -M_PI_4 * 0.25)
    walk_around_correction_dist = (angle + M_PI_4 * 0.25);

  //calculate angle to target position and position at ball
  Vector2<double> rel_ball_to_target_point(target_point - ball_position);
  float global_ang_ball_to_target_point = rel_ball_to_target_point.angle();

  Vector2<double> shoot_dist_vec = Vector2<double>(rel_ball_to_target_point).normalize(
      -(std::min(0.3, 0.2 * fabs(walk_around_correction_dist))));

  shoot_dist_vec.rotate(walk_around_correction);
  Vector2<double> move_to_pos = ball_position + shoot_dist_vec
      - Vector2<double>(theActiveFoot->offset).rotate(global_ang_ball_to_target_point);

//--------------- INTERCEPTION   just adds an offset

//------------------

  Pose2D move_target(global_ang_ball_to_target_point, move_to_pos);
  theSkillGetBallOutput.moveToRequest.setValues(move_target);
  //avoidObstacles = (myself_to_ball.length()>0.5);
}

