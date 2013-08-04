#include "SkillKick.h"

MAKE_MODULE(SkillKick)

void SkillKick::init()
{
  ball_visibility = 0;
  step_left_frames = 0;
  step_right_frames = 0;
  time_kick_started = 0;
  kickType = MotionRequest::KICK_LEFT;
  params = &(theSkillKickParameters->params.find(kickType)->second);
}

void SkillKick::update(SkillKickOutput& theSkillKickOutput)
{
  if (theBallPercept->updated)
    ball_visibility += (1.0 - ball_visibility) * 0.02;
  else
    ball_visibility += (0.0 - ball_visibility) * 0.02;

  // for stabilization
  if ((theFrameInfo->time_ms - time_kick_started) < 500)
  {
    theSkillKickOutput.active = true;
    theSkillKickOutput.motionRequest.motion = MotionRequest::STAND;
    return;
  }

  //check if kick skill requested
  if (!(theSkillKickOutput.active = theSkillRequest->skill == SkillRequest::KICK))
    return;

  double maxAngleError = theSkillRequest->kickAccuracy;
  selectKick(theSkillRequest->target.translation, maxAngleError);

  if (kickPossible(theSkillRequest->target.translation, maxAngleError))
  {
    printf("KICK %d\n", kickType);
    time_kick_started = theFrameInfo->time_ms;
    theSkillKickOutput.moveToRequest.active = false;
    theSkillKickOutput.motionRequest.motion = kickType;
    theSkillKickOutput.motionRequest.kickSpeed = theSkillRequest->kickSpeed;
  }
  else
  {
    theSkillKickOutput.moveToRequest.setValues(
        positioning(theSkillRequest->target.translation, maxAngleError), -1,
        (theBallPos->relativePos.abs() > config.getValue("ignoreObstacleBallDist", 0.5)));
  }

}

void SkillKick::selectKick(const Vector2<double> &target, double maxAngleError)
{
  const Vector2<double> &ball = theBallPos->absPos;
  double angleBallToTarget = (target - ball).angle();

  double bestAngle = fabs(
      normalize(theRobotPose->pose.rotation + params->angle - angleBallToTarget));

  double bestDist = (ball - theRobotPose->pose * params->foot).abs();

  for (std::map<MotionRequest::Motions, SkillKickParameters::Params>::const_iterator iter =
      theSkillKickParameters->params.begin(); iter != theSkillKickParameters->params.end(); iter++)
  {
    const SkillKickParameters::Params& p = iter->second;
    const Vector2<double> absFoot = theRobotPose->pose * p.foot;
    const double angle = fabs(normalize(theRobotPose->pose.rotation + p.angle - angleBallToTarget));
    const double dist = (ball - absFoot).abs();
    if (angle < bestAngle - 0.2 || (fabs(angle - bestAngle) < 0.2 && dist < bestDist - 0.02))
    {
      kickType = iter->first;
      bestAngle = angle;
      bestDist = dist;
    }
  }

  //kickType = MotionRequest::KICK_LEFT_45;
  params = &(theSkillKickParameters->params.find(kickType)->second);
}

Pose2D SkillKick::positioning(const Vector2<double> &target_point, double maxAngleError)
{
  //calculate angle to target position
  const Vector2<double> &ball_position = theBallPos->absPos;
  //const Vector2<double> &ball_position = theInterceptPosition->own;
  Vector2<double> rel_ball_to_target_point(target_point - ball_position);
  float global_ang_ball_to_target_point = rel_ball_to_target_point.angle();
  global_ang_ball_to_target_point = normalize(global_ang_ball_to_target_point - params->angle);

  //angle between foot-ball and ball-target
  float angle = normalize(
      (ball_position - theRobotPose->pose * params->foot).angleToVector(
          target_point - ball_position));

  //check if it is necessary to walk around ball
  float walk_around_correction = 0;
  if (angle > M_PI_4)
    walk_around_correction = (angle - M_PI_4);
  if (angle < -M_PI_4)
    walk_around_correction = (angle + M_PI_4);
  float walk_around_correction_dist = 0;
  if (angle > params->distAngleStart)
    walk_around_correction_dist = (angle - params->distAngleStart);
  if (angle < -params->distAngleStart)
    walk_around_correction_dist = (angle + params->distAngleStart);

  Vector2<double> shoot_dist_vec = Vector2<double>(rel_ball_to_target_point).normalize(
      -params->dist
          + std::max(-params->distMax,
              -params->distAngleFactor * fabs(walk_around_correction_dist)));

  //if current orientation withing max angle error, do not turn
  double r = theRobotPose->pose.rotation;
  if (fabs(global_ang_ball_to_target_point - r) < maxAngleError)
    global_ang_ball_to_target_point = r;

  //position to walk to
  shoot_dist_vec.rotate(walk_around_correction);
  Vector2<double> move_to_pos = ball_position + shoot_dist_vec
      - Vector2<double>(params->foot).rotate(global_ang_ball_to_target_point);
  Pose2D move_target(global_ang_ball_to_target_point, move_to_pos);

#ifdef DEBUG_MODE
  Vector2<double> tmp = ball_position + shoot_dist_vec;
  drawing.circle("SkillKick.positioning",
      tmp.x,tmp.y, 0.03, 200,200,200, 2);
  drawing.pose("SkillKick.positioning",
      move_target, 0.15, 3, 200,200,200);
#endif

  return move_target;
}

bool SkillKick::kickPossible(const Vector2<double> &target, double maxAngleError)
{

  step_left_frames = 1;
  step_right_frames = 1;

  //make sure ball was seen and get ball position
  if (ball_visibility < 0.2)
    return false;
  Vector2<double> ball_relative = theBallPos->relativePos;

  //minimum time between kicks
  if ((theFrameInfo->time_ms - time_kick_started) > 500)
  {
    //check ball position
    double footDist = (params->foot - ball_relative).abs();
    double angleError = (theRobotPose->pose.rotation + params->angle)
        - (target - theBallPos->absPos).angle();
//printf("maxAngleError %f    angleError %f\n", maxAngleError, angleError);
    //double angleError = (target-absFoot)
    //                      .angleToVector(theBallPos->absPos-absFoot);
    angleError = normalize(angleError);

    const Vector2<double> absFoot = theRobotPose->pose * params->foot;
    drawing.line("SkillKick.footToTarget", absFoot.x, absFoot.y, 0, target.x, target.y, 0, 255, 255,
        255, 5);
    /*const Vector2<double> angleVec = theRobotPose->pose * Vector2<double>(4.0,0).rotate(params->angle);
     debug.drawing.line("SkillKick.aim",
     theRobotPose->pose.translation.x, theRobotPose->pose.translation.y,0,
     angleVec.x, angleVec.y,0, 255,255,255, 5);*/

    //log << footDist << "   " << angleError << std::endl;
    if (footDist > params->footDistMax || fabs(angleError) > maxAngleError)
    {
      return false;
    }

    //right moment for kick ?
    if ((params->kickWithLeft && step_left_frames == 1)
        || (!params->kickWithLeft && step_right_frames == 1))
    {
      return true;
    }
  }

  return false;
}

