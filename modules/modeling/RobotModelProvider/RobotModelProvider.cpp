/**
 * @file RobotModelProvider.cpp
 *
 * Implements the module RobotModelProvider.
 *
 * @author Andreas Seekircher <aseek@cs.miami.edu>
 */
#include "RobotModelProvider.h"

MAKE_MODULE(RobotModelProvider)


void RobotModelProvider::update(FKModel& theFKModel)
{
  //update model
  theFKModel.update(theJointData->values);
}

