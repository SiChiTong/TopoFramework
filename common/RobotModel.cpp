/** 
 * @file RobotModel.cpp
 * Implementation of the class RobotModel.
 * 
 * @author Andreas <aseek@cs.miami.edu>
 * 
 */
#include "RobotModel.h"

#include <iostream>
#include <stdio.h>
#include <cstring>

RobotModelBase::RobotModelBase()
{
  root = NULL;
  partPositions = NULL;
  jointPositions = NULL;
  numberOfJoints = 0;
  jointAngles = NULL;
  partOriginsAtJoint = false;
  mass = 0.0f;
}

RobotModelBase::~RobotModelBase()
{
  if (partPositions != NULL)
    delete[] partPositions;
  if (jointPositions != NULL)
    delete[] jointPositions;
  if (root != NULL)
    delete root;
  root = NULL;
}

void RobotModelBase::update(const JointValues &jointValues)
{
  for (int i = 0; i < NUM_JOINT_ID; i++)
    jointAnglesBuffer[i] = jointValues[i].angle;
  update(jointAnglesBuffer);
}

void RobotModelBase::update(double *jointAngles)
{
  this->jointAngles = jointAngles;

  //reset all values
  mass = 0;
  com = Vector3<double>(0, 0, 0);

  //go through tree with all body parts
  updateModelRec(root, origin);

  com *= 1.0 / mass;

  this->jointAngles = NULL;
}

void RobotModelBase::updateModelRec(Bodypart *part, Pose3D t)
{
  //transform to body part position
  t.translate(part->translation);

  //if there is a joint on this body part (only not on root)
  if (part->jointIndex >= 0)
  {
    t.translate(part->anchor);               //go to joint position
    jointPositions[part->jointIndex] = t;    //set joint position
    t.rotate(RotationMatrix(part->axis, jointAngles[part->jointIndex]));
    t.translate(part->anchor * -1.0);        //rotate part and go back to center
  }

  if (partOriginsAtJoint)
    (partPositions[part->jointIndex + 1] = t).translate(part->anchor);
  else
    partPositions[part->jointIndex + 1] = t;

  com += t.translation * part->mass;              //calculate center of mass
  mass += part->mass;

  //continue with body parts mounted on this part
  for (unsigned int i = 0; i < part->children.size(); i++)
    updateModelRec(part->children[i], t);
}

//--------------------------------------------------------

RobotModelBase::Bodypart::Bodypart(const char* n) :
    jointIndex(-1), mass(0.0f), parent(0), shape(BOX), shape_param1(-1), shape_param2(-1), shape_param3(
        -1)
{
  strcpy(name, n);
}

RobotModelBase::Bodypart::Bodypart(int index, const char* n, Bodypart* parent,
    Vector3<double> translation, double mass, Vector3<double> anchor, Vector3<double> axis,
    Shape shape, double shape_param1, double shape_param2, double shape_param3) :
    jointIndex(index), translation(translation), mass(mass), anchor(anchor), axis(axis), parent(
        parent), shape(shape), shape_param1(shape_param1), shape_param2(shape_param2), shape_param3(
        shape_param3)
{
  strcpy(name, n);
  if (parent != NULL)
    parent->children.push_back(this);
}

RobotModelBase::Bodypart::~Bodypart()
{
  for (unsigned int i = 0; i < children.size(); i++)
    delete children[i];
}

