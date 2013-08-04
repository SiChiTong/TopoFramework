#include "SkillMoveToPos.h"

MAKE_MODULE(SkillMoveToPos)

void SkillMoveToPos::init()
{
  keepForwardOrientation = config.getValue("keepForwardOrientation", true);
}

void SkillMoveToPos::update(SkillMoveToPosOutput& theSkillMoveToPosOutput)
{
  if (!(theSkillMoveToPosOutput.active = theSafeWalkDirection->active))
  {
    destination_turn = false;
    walk = false;
    walk_stopped = true;
    return;
  }

  float max_walk_speed = config.getValue("max_walk_speed", 1000.0f);
  float min_walk_speed = config.getValue("min_walk_speed", 50.0f);
  float avoid_danger_speed = config.getValue("avoid_danger_speed", 300.0f);
  float min_turn_ang_to_goal = config.getValue("min_turn_ang_to_goal", 3.0f);
  float max_turn_speed_stand = config.getValue("max_turn_speed_stand", 0.5f);
  float max_turn_speed_walk = config.getValue("max_turn_speed_walk", 0.0f);
  float turn_dest_rotation_dist = config.getValue("turn_dest_rotation_dist", 2.0f);
  float break_distance = config.getValue("break_distance", 0.5f);
  float turn_p_value = config.getValue("turn_p_value", 0.02f);

  float stop_distance = 0.1; // TODO from MoveToRequest ?

  //------------------------------------------------------------------------

  // get the position of robot and ball
  const Vector2<double> &my_position = theRobotPose->pose.translation;
  float my_direction = theRobotPose->pose.rotation * (180 / M_PI);

  Pose2D absolute_destination = theSafeWalkDirection->moveToRequest.target;
  absolute_destination.rotation *= 180.0 / pi;
  const Vector2<double> destination_point = absolute_destination.translation;

  // get distance to destination point
  Vector2<double> myself_to_destination(destination_point - my_position);
  float distance_to_destination = myself_to_destination.abs();
  //float angle_to_destination = atan2(myself_to_destination.y, 
  //           myself_to_destination.x)*(180.0/M_PI);

  // get difference between my direction and desired position
  float ang_diff_destination = atan2(myself_to_destination.y, myself_to_destination.x)
      * (180.0 / M_PI) - my_direction;

  if (keepForwardOrientation)
  {
    //if destination away from opp goal, prefer backwards walk
    if (backwards && (myself_to_destination.x > 1.0 || distance_to_destination > 6.0))
      backwards = false;
    if (!backwards && (myself_to_destination.x < -1.0 && distance_to_destination < 5.0))
      backwards = true;
    if (backwards)
      ang_diff_destination += 180.0;
  }

  while (ang_diff_destination > 180.0)
    ang_diff_destination -= 360.0;
  while (ang_diff_destination < -180.0)
    ang_diff_destination += 360.0;

  // get difference between my direction and desired rotation
  float ang_diff_destination_rotation = absolute_destination.rotation - my_direction;
  // normalize angle
  while (ang_diff_destination_rotation > 180.0)
    ang_diff_destination_rotation -= 360.0;
  while (ang_diff_destination_rotation < -180.0)
    ang_diff_destination_rotation += 360.0;

  // already standing at destination position and facing the destination rotation ? 
  /*if((distance_to_destination < stop_distance) &&
   (fabs(ang_diff_destination_rotation) < min_turn_ang_to_goal) )
   {
   //STAND
   theSkillMoveToPosOutput->motionRequest.motion=MotionRequest::SPECIAL_ACTION;
   theSkillMoveToPosOutput->motionRequest.specialActionRequest
   = MotionRequest::STAND;
   log << "STAND" << std::endl;
   return;
   }*/

  // initial walk_vector, rotation and speed
  Vector2<double> walk_vector;
  if (!theSafeWalkDirection->moveToRequest.avoidObstacles
      || (destination_point - my_position).abs() < 0.1)
    walk_vector = destination_point - my_position;
  else
    walk_vector = theSafeWalkDirection->vec;

  float walk_rotation = 0.0;
  float walk_speed = 0.0;
  bool lower_speed = false;

  //------------------------------------------------------------------------

  if (destination_turn && distance_to_destination > stop_distance + 0.2)
    destination_turn = false;
  if (!destination_turn && distance_to_destination < stop_distance)
    destination_turn = true;

  float ang_diff = (destination_turn ? ang_diff_destination_rotation : ang_diff_destination);
  if (!walk && (fabs(ang_diff) < 20 || fabs(ang_diff) > 160 || distance_to_destination < 1.5)
      && (distance_to_destination > stop_distance + 0.2 || fabs(ang_diff) < min_turn_ang_to_goal))
    walk = true;
  if (walk
      && ((fabs(ang_diff) > 50 && fabs(ang_diff) < 130
          && (distance_to_destination > 1.5
              || fabs(ang_diff_destination_rotation - ang_diff_destination) < 20))
          || (distance_to_destination < stop_distance && fabs(ang_diff) > min_turn_ang_to_goal + 2)))
  {
    walk = false;
    walk_stopped = false;
  }

  // calculate walk speed
  if (!walk)
    walk_speed = 0.0;
  else if (lower_speed)
    walk_speed = avoid_danger_speed;
  else if (distance_to_destination > break_distance)
    walk_speed = max_walk_speed;
  else
    walk_speed = min_walk_speed
        + (distance_to_destination / break_distance) * (max_walk_speed - min_walk_speed);

  // calculate walk rotation
  // towards the destination point and then the destination rotation
  float max_turn_speed = ((walk || !walk_stopped) ? max_turn_speed_walk : max_turn_speed_stand);
  float angle_diff = (destination_turn ? ang_diff_destination_rotation : ang_diff_destination);

  //if(theForceData->updatedLeft && theForceData->updatedRight
  //   && theForceData->vectorLeft.z  > 15 && theForceData->vectorLeft.z  < 70
  //   && theForceData->vectorRight.z > 15 && theForceData->vectorRight.z < 70)
  if (theWalkingEngineOutput->currentSpeed.translation.abs() < 10)
    walk_stopped = true;

  if (walk || walk_stopped)
  {
    //if( fabs(ang_diff_destination_rotation) > min_turn_ang_to_goal )
    walk_rotation = std::min(max_turn_speed, std::max(-max_turn_speed, angle_diff * turn_p_value));
  }

  // set new walk vector scale
  walk_vector.normalize(walk_speed);
  walk_vector.rotate(-my_direction / 180.0 * pi);

  //---------------------------------------------

  //drawings
  Vector2<double> tmp_wv = walk_vector / 1000.0;
  tmp_wv.rotate(my_direction / 180.0 * pi);
  if (tmp_wv.abs() > 1.0)
    tmp_wv.normalize(1.0);
  if (walk_speed > 0)
  {
    drawing.line("SkillMoveToPos.walkVector", my_position.x, my_position.y, 0,
        my_position.x + tmp_wv.x, my_position.y + tmp_wv.y, 0, 128, 128, 255, 3);
  }
  tmp_wv = Vector2<double>(0.5, 0);
  Vector2<double> tmp_wv_rotated = tmp_wv;
  tmp_wv_rotated.rotate(walk_rotation * 2.0);
  if (walk_rotation != 0)
  {
    drawing.line("SkillMoveToPos.walkRotation", my_position.x + tmp_wv.x, my_position.y + tmp_wv.y,
        0, my_position.x + tmp_wv_rotated.x, my_position.y + tmp_wv_rotated.y, 0, 128, 128, 255, 3);
  }

  // set output MotionRequest
  theSkillMoveToPosOutput.motionRequest.motion = MotionRequest::WALK;
  theSkillMoveToPosOutput.motionRequest.walkRequest = Pose2D(walk_rotation, walk_vector.x,
      walk_vector.y);
  // avoid small walk requests in y-direction to get
  // a stable forward walking
  //(fabs(walk_vector.y) > 5.0) ? walk_vector.y : 0.0);
}

