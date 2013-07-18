/**
 * @file Hoap2WalkingEngineTrajectories.h
 * @author Cord Niehaus
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>
#include "math/Common.h"
#include "math/Vector2.h"

class ComplexTrajectory
{
public:
/**
 * The available shapes of trajectories.
 */
enum Shape
{ 
  linear,
  halfSineClose, 
  halfSineOpen, 
  sine,
  quarterCircleClose,
  quarterCircleOpen,
  arcSineClose,
  arcSineOpen,
  sigmoid,
  numberOfShapes
};

private:
  uint32_t         numberOfSections;
  Vector2<double>* maxPoints;
  Shape*           sectionShape;
  double           phaseShift;

  /**
  * The method calculates the amplitude in the range [-1..1].
  * @param phase The phase in the trajectory in the range of startPoint to endPoint.
  * @return The amplitude.
  */
  double getAmplitude(double phase, 
                      Vector2<double> startPoint, 
                      Vector2<double> endPoint, 
                      Shape currentShape) const;

public:
  /**
   * Constructor.
   */
  ComplexTrajectory(uint32_t numberOfSections, 
                    Vector2<double>* maxPoints,  
                    Shape* sectionShape, 
                    double phaseShift)
    : numberOfSections(numberOfSections),
      maxPoints(maxPoints),
      sectionShape(sectionShape),
      phaseShift(phaseShift)
  {}

  /**
   * destructor, so objects of derived classes will be freed correctly.
   */
  ~ComplexTrajectory() {}

  /**
   * The method creates a new complex trajectory.
   * @param numberOfSections number of sections.
   * @param maxPoints maximum points during phase.
   * @param sectionShape shapes of trajectory during the section.
   * @param phaseShift shift of trajectory compared to the walk cycle.
   * @return The new trajectory. It is dynamically allocated and has to be freed by the caller.
   */
  static ComplexTrajectory* create(uint32_t numberOfSections, 
                                   Vector2<double>* maxPoints, 
                                   Shape* sectionShape, 
                                   double phaseShift);

  /**
   * The method returns a name for each shape.
   * @param shape The shape.
   * @return The name of the shape.
   */
  static const char* getShapeName(Shape shape);

  /**
   * The method calculates the position in the trajectory according 
   * to the current phase.
   * @param phase The current phase of the trajectory in the range of [0..1[.
   * @return The current trajectory value according to phase.
   */
  double getPosition(double phase) const;
};

#endif 
