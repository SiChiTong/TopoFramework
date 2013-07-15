/*
 * Line.h
 *
 *  Created on: Jun 14, 2012
 *      Author: sam
 */

#ifndef LINE_H_
#define LINE_H_
#include <cmath>
#include "Vector2.h"
#include "Pose3D.h"

class Line
{
  public:
    Vector2<double> base;
    Vector2<double> direction;

    Line()
    {
    }

    Line(const Vector2<>& base, const Vector2<>& direction) :
        base(base), direction(direction)
    {
    }

    Line(const Vector2<int>& base, const Vector2<>& direction) :
        direction(direction)
    {
      this->base.x = (float) base.x;
      this->base.y = (float) base.y;
    }

    Line(const Vector2<int>& base, const Vector2<int>& direction)
    {
      this->base.x = (float) base.x;
      this->base.y = (float) base.y;
      this->direction.x = (float) direction.x;
      this->direction.y = (float) direction.y;
    }

    Line(const Pose2D& base, float length = 1.f)
    {
      this->base = base.translation;
      this->direction =
          (Pose2D(base.rotation) + Pose2D(Vector2<>(length, 0))).translation;
    }

    Line(float baseX, float baseY, float directionX, float directionY)
    {
      base.x = baseX;
      base.y = baseY;
      direction.x = directionX;
      direction.y = directionY;
    }

    void normalizeDirection()
    {
      float distance = sqrt(sqr(direction.x) + sqr(direction.y));
      if (distance > 0)
      {
        direction.x = direction.x / distance;
        direction.y = direction.y / distance;
      }
    }

    static bool getIntersectionOfLines(const Line& line1, const Line& line2, Vector2<int>& intersection)
    {
      if(line1.direction.y* line2.direction.x == line1.direction.x * line2.direction.y)
      {
        return false;
      }

      intersection.x =
        line1.base.x +
        line1.direction.x *
        (
          line1.base.y * line2.direction.x -
          line2.base.y * line2.direction.x +
          (-line1.base.x + line2.base.x) * line2.direction.y
        )
        /
        ((-line1.direction.y) * line2.direction.x + line1.direction.x * line2.direction.y);

      intersection.y =
        line1.base.y +
        line1.direction.y *
        (
          -line1.base.y * line2.direction.x +
          line2.base.y * line2.direction.x +
          (line1.base.x - line2.base.x) * line2.direction.y
        )
        /
        (line1.direction.y * line2.direction.x - line1.direction.x * line2.direction.y);

        return true;
      }
    };

#endif /* LINE_H_ */
