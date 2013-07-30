#ifndef SimSparkLinePercept_H
#define SimSparkLinePercept_H

#include "kernel/Template.h"
REPRESENTATION(SimSparkLinePercept)

#include <vector>
#include "math/Polar.h"


class SimSparkLinePercept : public SimSparkLinePerceptBase
{
  public:
        
    std::vector<std::pair<Polar, Polar> > lines;

};


#endif

