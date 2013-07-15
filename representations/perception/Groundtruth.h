/**
 * @file RobotPose.h
 * Contains the Groundtruth representation.
 */
#ifndef GROUNDTRUTH_H
#define GROUNDTRUTH_H

#include "kernel/Framework.h"
#include "math/Pose2D.h"
#include "math/Pose3D.h"
#include "math/Vector2.h"
#include "math/Vector3.h"

#include "representations/modeling/OtherRobots.h"

REPRESENTATION(Groundtruth)

/** 
 * @class Groundtruth
 * The representation that stores all groundtruth information.
 */
class Groundtruth: public GroundtruthBase
{
  public:

    /** Robot pose. */
    Pose2D myself;
    /** Robot position z. */
    double myselfZ;
    /** Robot upright vector. */
    Vector3<double> myselfUp;
    /** Robot forward vector. */
    Vector3<double> myselfForward;

    /** 2D ball position. */
    Vector2<double> ball;
    /** 3D ball position. */
    Vector3<double> ball3D;
    /** Other robots. */
    OtherRobotsData otherRobots;

    /** When no groundtruth is received, this is false. */
    bool updated;

    /** Draw ground truth. */
    /*
     void draw() const
     {
     if(!updated)
     return;

     int r=50, g=50, b=50;
     debug.drawing.pose("Groundtruth.myself", myself, 0.2, 2, r,g,b);
     debug.drawing.circle("Groundtruth.ball", ball.x, ball.y, 0.04, 2, r,g,b);

     if(myselfUp.abs() > 0.001)
     {
     Vector3<double> v1(myself.translation.x, myself.translation.y, myselfZ);
     Vector3<double> v2 = v1 +
     (RotationMatrix::fromRotationZ(myself.rotation)*myselfUp) * 0.5;
     debug.drawing.line("Groundtruth.uprightVec",
     v1.x,v1.y,v1.z,v2.x,v2.y,v2.z, r,g,b, 2);
     }
     if(myselfForward.abs() > 0.001)
     {
     Vector3<double> v1(myself.translation.x, myself.translation.y, myselfZ);
     Vector3<double> v2 = v1 +
     (RotationMatrix::fromRotationZ(myself.rotation)*myselfForward)*0.25;
     debug.drawing.line("Groundtruth.forwardVec",
     v1.x,v1.y,v1.z,v2.x,v2.y,v2.z, r,g,b, 2);
     }
     }
     */

    /** Torso transformation in field coordinate system. **/
    Pose3D torso;
    /** Torso transformation relative to the Pose2D of the robot. **/
    Pose3D torsoRelative;

    /** Calculate torso pose using the up and forward vectors. */
    void calculateTorsoTransformation()
    {
      torso.translation.x = myself.translation.x;
      torso.translation.y = myself.translation.y;
      torso.translation.z = myselfZ;
      torsoRelative = Vector3<double>(0, 0, myselfZ);
      if (myselfUp.abs() < 0.001 || myselfForward.abs() < 0.001)
      {
        //if up or forward vector is missing, just rotate around z axis
        torso.rotation = RotationMatrix::fromRotationZ(myself.rotation);
        return;
      }
      torso.rotation = RotationMatrix::fromRotationZ(myself.rotation);
      const double angleY = -atan2(myselfForward.z, myselfForward.x);
      torso.rotation.rotateY(angleY);
      const Vector3<double> upRot = RotationMatrix::fromRotationY(-angleY) * myselfUp;
      const double angleX = -atan2(upRot.y, upRot.z);
      torso.rotation.rotateX(angleX);
      torsoRelative.rotation = RotationMatrix::fromRotationY(angleY);
      torsoRelative.rotation.rotateX(angleX);
    }
};

#endif

