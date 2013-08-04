#ifndef KICKMOTION_H
#define KICKMOTION_H

#include "kernel/Template.h"


#include "representations/perception/FrameInfo.h"
#include "representations/motion/MotionRequest.h"
#include "representations/motion/KickMotionOutput.h"


MODULE(KickMotion)
  REQUIRES(FrameInfo)  
  REQUIRES(MotionRequest)  
  PROVIDES(KickMotionOutput)
END_MODULE


class KickMotion : public KickMotionBase
{
  public:

    void init();
    
    void update(KickMotionOutput& theKickMotionOutput);
  
  private:
  
    long time_kick_started;
    
    MotionRequest::Motions kick_type;
    double kickSpeed;
  
};


#endif

