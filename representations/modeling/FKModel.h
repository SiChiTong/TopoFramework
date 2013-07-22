/**
 * @file FKModel.h
 * Contains the RobotModel
 */
#ifndef FKMODEL_H
#define FKMODEL_H

#include "kernel/Framework.h"
#include "common/JointValues.h"
#include "common/RobotModel.h"


REPRESENTATION(FKModel)

/** 
 * @class FKModel
 * The representation that stores the RobotModel.
 * 
 * @author Andreas Seekircher <aseek@cs.miami.edu>
 */
class FKModel: public FKModelBase, public RobotModel
{
  public:

    FKModel()
    {
      //init model
      init();
      if (getNumberOfJoints() != NUM_JOINT_ID)
      {
        printf("ERROR model joints != NUM_JOINT_ID !!!\n");
        exit(1);
      }
    }

    /** Draws the model vector vec. */
    void draw() const
    {
      //draw body part positions
      /*for(int i=0; i<NUM_JOINT_ID+1; i++)
       {
       Vector3<double> p = //theGroundtruth->torsoRelative *
       (model.partPositions[i] * Vector3<double>(0,0,0));
       debug.drawing.point("bodypart", p.x, p.y, p.z, 10, 255,100,100);
       }
       //draw joint positions
       for(int i=0; i<NUM_JOINT_ID; i++)
       {
       Vector3<double> p = //theGroundtruth->torsoRelative *
       (model.jointPositions[i] * Vector3<double>(0,0,0));
       debug.drawing.point("jointpos", p.x, p.y, p.z, 10, 100,255,255);
       }

       //draw CoM
       Vector3<double> p = //theGroundtruth->torsoRelative *
       model.com;
       debug.drawing.point("CoM", p.x, p.y, p.z, 12, 0,0,0);*/
    }

};


#endif

