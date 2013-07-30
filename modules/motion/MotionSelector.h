#ifndef MOTIONSELECTOR_H
#define MOTIONSELECTOR_H

#include "kernel/Template.h"

#include "representations/motion/MotionRequest.h"
#include "representations/motion/MotionSelection.h"

MODULE(MotionSelector)
  REQUIRES(MotionRequest)
  PROVIDES(MotionSelection)
END_MODULE


class MotionSelector : public MotionSelectorBase
{
  public:
    
    void update(MotionSelection& theMotionSelection);
  
};


#endif

