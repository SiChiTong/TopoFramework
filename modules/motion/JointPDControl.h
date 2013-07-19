#ifndef JOINTPDCONTROL_H
#define JOINTPDCONTROL_H

#include "kernel/Framework.h"

#include "representations/perception/FrameInfo.h"
#include "representations/perception/JointData.h"
#include "representations/motion/JointRequest.h"
#include "representations/motion/JointRequestWithSpeeds.h"


MODULE(JointPDControl)
  REQUIRES(FrameInfo)
  REQUIRES(JointData)
  REQUIRES(JointRequest)  
  PROVIDES(JointRequestWithSpeeds)
END_MODULE


class JointPDControl : public JointPDControlBase
{
  public:
    
    void update(JointRequestWithSpeeds& theJointRequestWithSpeeds);
  
  private:
  
    double lastTimestamp;
    float prevError[NUM_JOINT_ID];
    
  
};


#endif

