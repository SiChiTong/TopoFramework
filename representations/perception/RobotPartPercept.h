#ifndef ROBOTPARTPERCEPT_H
#define ROBOTPARTPERCEPT_H

#include <vector>

#include "kernel/Framework.h"
#include "math/Polar.h"

#undef HEAD // for mac users with random compatibility issues :)
REPRESENTATION(RobotPartPercept)

class PartPercept
{
  public:
    enum ROBOT_PART
    {
      HEAD, RIGHT_ARM, LEFT_ARM, RIGHT_FOOT, LEFT_FOOT
    };

    int unum;
    ROBOT_PART type;
    Polar polar;
};

inline PartPercept::ROBOT_PART& operator++(PartPercept::ROBOT_PART& part)
{
  return part = PartPercept::ROBOT_PART(part + 1);
}

class RobotPartPercept: public RobotPartPerceptBase
{
  public:

    std::vector<PartPercept> teammates;
    std::vector<PartPercept> opponents;

};

#endif

