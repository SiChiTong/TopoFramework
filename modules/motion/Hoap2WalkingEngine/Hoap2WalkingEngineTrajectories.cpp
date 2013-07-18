/**
* @file Hoap2WalkingEngineTrajectories.cpp
* @author Cord Niehaus
*/
#include "Hoap2WalkingEngineTrajectories.h"

/**
 *  creates the ComplexTrajectory
 */
ComplexTrajectory* ComplexTrajectory::create(uint32_t numberOfSections, 
                                             Vector2<double>* maxPoints,  
                                             Shape* sectionShape,
                                             double phaseShift)
{
  return new ComplexTrajectory(numberOfSections, maxPoints, sectionShape, phaseShift);
}

/**
 *  returns the name of a given shape
 */
const char* ComplexTrajectory::getShapeName(ComplexTrajectory::Shape shape)
{
  switch(shape)
  {
    case linear: 
      return "linear";
    case halfSineClose: 
      return "halfSineClose";
    case halfSineOpen: 
      return "halfSineOpen";
    case sine: 
      return "sine";
    case quarterCircleClose: 
      return "quarterCircleClose";
    case quarterCircleOpen: 
      return "quarterCircleOpen";
    case arcSineClose: 
      return "arcSineClose";
    case arcSineOpen: 
      return "arcSineOpen";
    case sigmoid: 
      return "sigmoid";
    default: 
      return "unknown";
  }
}

/**
 * calculates the current amplitude of the trajectory
 */
double ComplexTrajectory::getAmplitude(double phase, 
                                       Vector2<double> startPoint, 
                                       Vector2<double> endPoint, 
                                       Shape currentShape) const
{
  // normalize the x and y values between start and end points 
  double xDiff     = endPoint.x - startPoint.x;
  double yDiff     = endPoint.y - startPoint.y;
  double normXDiff = (xDiff == 0) ? 0 : ((phase - startPoint.x) / xDiff);
  double fctVal    = 0.0;

  switch(currentShape)
  {
    case linear:
    {
      // simply linear between 0 and 1
      fctVal = normXDiff;
      break; 
    }
    case halfSineClose:
    {
      // sinus function between pi/2 and pi
      fctVal = -sin(pi/2 + (normXDiff * pi/2)) + 1;
      break;
    }
    case halfSineOpen:
    {
      // sinus function between 0 and pi/2 
      fctVal = sin(normXDiff * pi/2);
      break;  
    }
    case sine: 
    {
      // sinus function between -pi/2 and pi/2
      fctVal = (sin(-pi/2 + (normXDiff * pi)) + 1) / 2;
      break; 
    }
    case quarterCircleClose:
    {
      // circle segment from 90 to 180 degree
      fctVal = sin(acos(normXDiff-1));
      break;
    }
    case quarterCircleOpen:
    {
      // circle segment from 0 to 90 degree
      fctVal = sin(-acos(normXDiff)) + 1;
      break;
    }
    case arcSineClose:
    {
      // arc sinus from 0 to 1
      fctVal = asin(normXDiff) / (pi/2);
      break;
    }
    case arcSineOpen:
    {
      // arc sinus from 1 to 0
      fctVal = 1 - (asin(1 - normXDiff) / (pi/2));
      break;
    }
    case sigmoid:
    {
      // the factor 12.5 is pure empirical  
      fctVal = 1/(1 + pow(e, -12.5*(normXDiff - 0.5)));
      break;
    }
    default: return 0.0;
  }
  // calc current trajectory value
  return startPoint.y + (fctVal * yDiff);
}

/**
 * The method calculates the position in the trajectory for a certain point according to the phase.
 */
double ComplexTrajectory::getPosition(double phase) const
{
  // add phase shift to phase
  phase += phaseShift;
  // normalize phase if phase value out of the range [0,1[
  // value too high
  if(phase >= 1.0)
    phase -= 1.0;
  // value too low
  if(phase < 0.0)
    phase += 1.0;
  
  // get current section within the complex trajectory by comparing
  // the current phase with the given maxpoints 
  uint32_t currentSection = 0;
  for(uint32_t i = 0; i < numberOfSections; i++)
  {
    if(phase <= maxPoints[i+1].x &&
       phase >= maxPoints[i].x)
    {
      currentSection = i;
      break;
    }
  }

  // set the start and end points as well as the shape according to the current section
  Vector2<double> startPoint   = maxPoints[currentSection];
  Vector2<double> endPoint     = maxPoints[currentSection+1];
  Shape           currentShape = sectionShape[currentSection];

  return getAmplitude(phase, startPoint, endPoint, currentShape);
}
