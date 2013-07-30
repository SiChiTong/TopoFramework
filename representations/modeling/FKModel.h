/**
 * @file FKModel.h
 * Contains the RobotModel
 */
#ifndef FKMODEL_H
#define FKMODEL_H

#include "kernel/Template.h"
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
};


#endif

