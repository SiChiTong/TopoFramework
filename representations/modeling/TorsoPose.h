/**
 * @file TorsoPose.h
 * Contains the TorsoPose representation.
 */
#ifndef TORSOPOSE_H
#define TORSOPOSE_H

#include "kernel/Framework.h"
#include "math/Pose3D.h"
#include <sstream>

REPRESENTATION(TorsoPose)

/** Estimated transformation from the ground to the torso center. */
class TorsoPose: public TorsoPoseBase, public Pose3D
{
  public:

    TorsoPose() :
        valid(false)
    {
    }

    bool valid;

    Pose3D offset; /**< The estimated offset (including odometry) from last torso matrix to this one. (relative to the torso) */

    /** Draws the torso pose. */
    void draw() const
    {
      Vector3<double> v1(this->translation);
      Vector3<double> v2(*this * Vector3<double>(1, 0, 0));
      drawing.line("TorsoPose", v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, 255, 0, 0, 5);
      Vector3<double> v3(*this * Vector3<double>(0, 0, 1));
      drawing.line("TorsoPose", v1.x, v1.y, v1.z, v3.x, v3.y, v3.z, 0, 0, 255, 5);
      std::stringstream ss;
      //ss << translation.z;
      //ss << rotation.getXAngle() << ":" << rotation.getYAngle();
      drawing.annotation("TorsoPose", ss.str(), 0, 0, 0.8, 255, 255, 255);
    }

};

#endif

