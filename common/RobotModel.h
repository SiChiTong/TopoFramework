/** 
 * @file RobotModel.h
 * Base class for robot models.
 * 
 * @author Andreas <aseek@cs.miami.edu>
 * 
 */

#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include <vector>
#include "math/Vector3.h"
#include "math/Pose3D.h"
#include "common/JointValues.h"

/**
 * @class RoboModelBase
 * Base class for forward kinematics and CoM.
 * The definition of the body parts has to be in a derived class in the
 * method initBodyParts.
 */
class RobotModelBase
{
  public:

    enum Limb  // Limbs of the Nao, can be used as index in partPositions array
    {
      torso,
      neck,
      head,
      shoulderLeft,
      bicepsLeft,
      elbowLeft,
      foreArmLeft,
      shoulderRight,
      bicepsRight,
      elbowRight,
      foreArmRight,
      pelvisLeft,
      hipLeft,
      thighLeft,
      tibiaLeft,
      ankleLeft,
      footLeft,
      pelvisRight,
      hipRight,
      thighRight,
      tibiaRight,
      ankleRight,
      footRight,
      numOfLimbs
    };

    enum Shape
    {
      CYLINDER, SPHERE, BOX
    };
    class Bodypart
    {
      public:
        int jointIndex;
        char name[32];
        Vector3<double> translation;
        double mass;
        Vector3<double> anchor;
        Vector3<double> axis;
        std::vector<Bodypart*> children;
        Bodypart *parent;

        Shape shape;
        double shape_param1;
        double shape_param2;
        double shape_param3;

        Bodypart(const char* n);
        Bodypart(int index, const char* n, Bodypart* parent, Vector3<double> translation,
            double mass, Vector3<double> anchor, Vector3<double> axis, Shape shape,
            double shape_param1, double shape_param2, double shape_param3);
        ~Bodypart();
    };

    RobotModelBase();
    virtual ~RobotModelBase();

    virtual void init() = 0;

    /** Method to update values, when the joint angles are changed. */
    void update(double *jointAngles);
    /** Method to update values, when the joint angles are changed. */
    void update(const JointValues &jointValues);

    int getNumberOfJoints()
    {
      return numberOfJoints;
    }

    /** Information calculated using the model and joint angles. */
    Pose3D *jointPositions;
    Pose3D *partPositions;
    Vector3<double> com;
    double mass;

  protected:

    /** Tree with body parts. */
    Bodypart* root;
    Pose3D origin;
    // if true, partpositions at joints, not at the CoM that is given by anchor
    bool partOriginsAtJoint;

    /** Joint angles. */
    double jointAnglesBuffer[NUM_JOINT_ID];
    double *jointAngles;
    int numberOfJoints;

    /** Recursive method for updateModel. */
    void updateModelRec(Bodypart *part, Pose3D t);
};

// fixMe: take care of the other models. Sam Abeyruwan
#include "SimSparkNaoModel.h"
typedef SimSparkNaoModel RobotModel;
#endif
