#include "SkillDribble.h"

MAKE_MODULE(SkillDribble)

void SkillDribble::init()
{
  ball_visibility = 0;
  dribbling = false;
}

void SkillDribble::update(SkillDribbleOutput& theSkillDribbleOutput)
{
  if (!(theSkillDribbleOutput.active = theSkillRequest->skill == SkillRequest::DRIBBLE))
    return;

  //if dribbling on, only use LocalBallPos
  const BallPosData* ballPos = (
      dribbling ? (BallPosData*) &(*theLocalBallPos) : (BallPosData*) &(*theBallPos));

  const Vector2<double> &target_point = theSkillRequest->target.translation;
  const Vector2<double> &ball_position = ballPos->absPos;
  const Vector2<double> &my_position = theRobotPose->pose.translation;
  double my_direction = theRobotPose->pose.rotation;
  Vector2<double> vec_orientation = Vector2<double>(1, 0).rotate(my_direction);

  //ball_visibility
  if (theBallPercept->updated)
    ball_visibility += (1.0 - ball_visibility) * 0.02;
  else
    ball_visibility += (0.0 - ball_visibility) * 0.02;

  //ball
  float angle_to_ball = ballPos->relativePos.angle();
  if (angle_to_ball > M_PI)
    angle_to_ball -= M_PI * 2.0;
  float dist_to_ball = ballPos->relativePos.abs();
  float ball_shift = sin(fabs(angle_to_ball)) * dist_to_ball;
  if (fabs(angle_to_ball) > 90 * M_PI / 180.0)
    ball_shift = 100;
  //float angle_to_ball_deg = angle_to_ball/M_PI * 180.0;

  //target
  float dist_to_target = (ball_position - target_point).abs();
  float angle_to_target_point_deg = (target_point - my_position).angleToVector(vec_orientation)
      * 180.0 / pi;
  if (angle_to_target_point_deg > 180)
    angle_to_target_point_deg -= 360;
  float target_shift = sin(fabs(angle_to_target_point_deg / 180.0 * M_PI)) * dist_to_target;
  if (fabs(angle_to_target_point_deg) > 90)
    target_shift = 100;

  //ball and target
  float angle_to_ball_and_target =
      normalize(
          (ball_position - theRobotPose->pose * Vector2<double>(0, theActiveFoot->offset.y)).angleToVector(
              target_point - ball_position));
  float angle_to_ball_and_target_deg = angle_to_ball_and_target / M_PI * 180.0;

  float foot_ball_shift = sin(angle_to_ball) * dist_to_ball - theActiveFoot->offset.y;

//log << "foot_ball_shift = " << foot_ball_shift << std::endl;
  //change state
  bool prev_dribbling = dribbling;
  if (dribbling
      && (false  // (/*fabs(angle_to_ball_deg) > 50 &&*/ ball_shift > 0.13)
      || dist_to_ball > config.getValue("stop_ballDist", 2.5)
          || (fabs(angle_to_ball_and_target_deg)
              > config.getValue("stop_angleToBallAndTargetDeg", 25.0)
              && target_shift > config.getValue("stop_targetShift", 0.5)) || ball_visibility < 0.01
      /*|| (opp_shift < 0.02 && distance_to_opponent < 4.0)*/))
    dribbling = false;
  else if (!dribbling
      && (/*fabs(angle_to_ball_deg) < 20 ||*/fabs(foot_ball_shift) < 0.05) //ball_shift < 0.05)
      && dist_to_ball < config.getValue("start_ballDist", 2.0)
      && (fabs(angle_to_ball_and_target_deg)
          < config.getValue("start_angleToBallAndTargetDeg", 20.0)
          || target_shift < config.getValue("start_targetShift", 0.4)) && ball_visibility > 0.1)
    dribbling = true;

  //the actual dribbling or positioning
  if (dribbling)
  {
    Vector2<double> footpos = theRobotPose->pose * Vector2<double>(0, theActiveFoot->offset.y);
    Vector2<double> v = (ball_position - footpos).rotate(-my_direction);
    //Vector2<double> v = (target_point-footpos).rotate(-my_direction);

    //reactive dribbling obstacle avoidance...
    float distance_to_opponent = 100.0f;
    float distance_opp_ball = 100.0f;
    float angle_to_opponent = 0;
    std::vector<OtherRobots::State>::const_iterator oppIter;
    for (oppIter = theOtherRobots->opponents.begin(); oppIter != theOtherRobots->opponents.end();
        ++oppIter)
    {
      const Vector2<double> &pos_2d = oppIter->pose.translation;
      float angle = (pos_2d - ball_position).angleToVector(
          Vector2<double>(1, 0).rotate(my_direction));
      angle = normalize(angle);
      angle *= 180.0 / pi;
      float dist = (pos_2d - ball_position).abs();
      if (dist < distance_opp_ball)
      {
        distance_opp_ball = dist;
        distance_to_opponent = (pos_2d - my_position).abs();
        angle_to_opponent = (angle <= 180 ? angle : angle - 360);
      }
    }
    if (dist_to_ball < 0.17 && distance_to_opponent < 1.6 && fabs(angle_to_opponent) < 80)
      v.rotate((angle_to_opponent + (angle_to_opponent > 0 ? -80 : 80)) * pi / 180.0);

    Vector2<double> tmp = v;
    tmp = theRobotPose->pose * (v + Vector2<double>(0, theActiveFoot->offset.y));
    drawing.line("SkillDribble.toTarget", footpos.x, footpos.y, 0, tmp.x, tmp.y, 0, 128, 255, 128,
        5);

    //theSkillDribbleOutput->motionRequest.motion = MotionRequest::WALK;
    //theSkillDribbleOutput->motionRequest.walkRequest = Pose2D(0, v.normalize(600));
    //theSkillDribbleOutput->moveToRequest.active = false;

    if (!prev_dribbling)
      v = my_position + v.normalize(0.09).rotate(my_direction); //hack :( ...
    else
      v = my_position + v.normalize(1.0).rotate(my_direction);
    Pose2D p(my_direction, v.x, v.y);

    theSkillDribbleOutput.moveToRequest.setValues(p, -1, (ballPos->relativePos.abs() > 0.3));
  }
  else
  {
    //walk to position behind ball

    const Vector2<double> &target_point = theSkillRequest->target.translation;

    Vector2<double> myself_to_ball = ball_position - my_position;

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

    Pose2D move_target(global_ang_ball_to_target_point, move_to_pos);
    theSkillDribbleOutput.moveToRequest.setValues(move_target, -1,
        (ballPos->relativePos.abs() > config.getValue("ignoreObstacleBallDist", 0.5)));
  }

}

