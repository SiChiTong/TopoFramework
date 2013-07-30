/**
 * @file RobotModelProvider.h
 *
 * Defines the module RobotModelProvider.
 *
 * @author Andreas Seekircher <aseek@cs.miami.edu>
 */

#ifndef ROBOTMODELPROVIDER_H
#define ROBOTMODELPROVIDER_H

#include "kernel/Template.h"

#include "representations/perception/JointData.h"
#include "representations/modeling/FKModel.h"

MODULE(RobotModelProvider)
  REQUIRES(JointData)
  PROVIDES(FKModel)
END_MODULE

/**
 * @class RobotModelProvider 
 * 
 */
class RobotModelProvider : public RobotModelProviderBase
{
  public:
    void update(FKModel& theFKModel);
    
  private:
    double jointAngles[NUM_JOINT_ID];
  
};


#endif

