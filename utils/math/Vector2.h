/**
 * @file Vector2.h
 * Contains template class Vector2 of type V
 *
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 */

#ifndef __Vector2_h__
#define __Vector2_h__

#include <math.h>
#include "Common.h"

/** This class represents a 2-vector */
template <class V = double> class Vector2
{
public:

  /** The vector values */
  V x,y;
  
  /** Default constructor. */
  Vector2<V>():x(V()),y(V())
  {
  }
  /** Default constructor. */
  Vector2<V>(V x, V y):x(x),y(y)
  {
  }

  /** Assignment operator
  *\param other The other vector that is assigned to this one
  *\return A reference to this object after the assignment.
  */
  Vector2<V>& operator=(const Vector2<V>& other)
  {
    x=other.x;y=other.y;
    return *this;
  }

  /** Copy constructor
  *\param other The other vector that is copied to this one
  */
  Vector2<V>(const Vector2<V>& other) {*this = other;}

  /** Addition of another vector to this one.
  *\param other The other vector that will be added to this one
  *\return A reference to this object after the calculation.
  */
  Vector2<V>& operator+=(const Vector2<V>& other)
  {
    x += other.x;
    y += other.y;
    return *this;
  }

  /** Substraction of this vector from another one.
  *\param other The other vector this one will be substracted from
  *\return A reference to this object after the calculation.
  */
  Vector2<V>& operator-=(const Vector2<V>& other)
  {
    x -= other.x;
    y -= other.y;
    return *this;
  }

  /** Multiplication of this vector by a factor.
  *\param factor The factor this vector is multiplied by
  *\return A reference to this object after the calculation.
  */
  Vector2<V>& operator*=(const V& factor)
  {
    x *= factor;
    y *= factor;
    return *this;
  }

  /** Division of this vector by a factor.
  *\param factor The factor this vector is divided by
  *\return A reference to this object after the calculation.
  */
  Vector2<V>& operator/=(const V& factor)
  {
    if (factor == 0) return *this;
    x /= factor;
    y /= factor;
    return *this;
  }

  /** Addition of another vector to this one.
  *\param other The other vector that will be added to this one
  *\return A new object that contains the result of the calculation.
  */
  Vector2<V> operator+(const Vector2<V>& other) const
    {return Vector2<V>(*this) += other;}

  /** Subtraction of another vector to this one.
  *\param other The other vector that will be added to this one
  *\return A new object that contains the result of the calculation.
  */
  Vector2<V> operator-(const Vector2<V>& other) const
    {return Vector2<V>(*this) -= other;}

  /** Negation of this vector.
  *\return A new object that contains the result of the calculation.
  */
  Vector2<V> operator-() const
    {return Vector2<V>() -= *this;}

  /** Inner product of this vector and another one.
  *\param other The other vector this one will be multiplied by
  *\return The inner product.
  */
  V operator*(const Vector2<V>& other) const
  {
    return x * other.x + y * other.y;
  }

  /** Multiplication of this vector by a factor.
  *\param factor The factor this vector is multiplied by
  *\return A new object that contains the result of the calculation.
  */
  Vector2<V> operator*(const V& factor) const
    {return Vector2<V>(*this) *= factor;}

  /** Division of this vector by a factor.
  *
  *\param factor The factor this vector is divided by
  *\return A new object that contains the result of the calculation.
  */
  Vector2<V> operator/(const V& factor) const
    {return Vector2<V>(*this) /= factor;}

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one
  *\return Whether the two vectors are equal.
  */
  bool operator==(const Vector2<V>& other) const
  {
    return (x==other.x && y==other.y);
  }

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one.
  *\return Whether the two vectors are unequal.
  */
  bool operator!=(const Vector2<V>& other) const
    {return !(*this == other);}

  /** Calculation of the length of this vector.
  *\return The length.
  */
  V abs() const
  {return (V) sqrt(((double)x)*x+((double)y)*y);}

  /** Calculation of the square length of this vector.
  *\return length*length.
  */
  V squareAbs() const
  {return (x*x) + (y*y);}

  /** normalize this vector.
  *\param len The length, the vector should be normalized to, default=1.
  *\return the normalized vector.
  */
  Vector2<V> normalize(V len)
  {
    if (abs() == 0) return *this;
    return *this = (*this * len) / abs();
  }

  /** normalize this vector.
  *\return the normalized vector.
  */
  Vector2<V> normalize()
  {
    if (abs() == 0) return *this;
    return *this /= abs();
  }

  /** transpose this vector.
  *\return the transposed vector.
  */
  Vector2<V> transpose()
  { V buffer = x;
    x = y;
    y = buffer;
    return *this;
  }

  /** the vector is rotated left by 90 degrees.
  *\return the rotated vector.
  */
  Vector2<V> rotateLeft()
  { V buffer = -y;
    y = x;
    x = buffer;
    return *this;
  }

  /** the vector is rotated right by 90 degrees.
  *\return the rotated vector.
  */
  Vector2<V> rotateRight()
  { V buffer = -x;
    x = y;
    y = buffer;
    return *this;
  }

  /** the vector is rotated by alpha.
  *\return the rotated vector.
  */
  Vector2<V> rotate(double alpha)
  { double buffer = x;
    double a = cos(alpha);
    double b = sin(alpha);
    x = (V)(a*(double)x - b*(double)y);
    y = (V)(b*buffer + a*(double)y);
    return *this;
  }

  /**
  * array-like member access.
  * \param i index of coordinate
  * \return reference to x or y
  */
  V& operator[](int i)
  {
    return  (&x)[i];
  }
  
  /** Calculation of the angle of this vector */
  double angle() const
  {return atan2((double)y,(double)x);}
  



  /**
   * Returns the signed distance of this vector to another one.
   * Here, this vector can be seen as a point and the
   * other vectors a directed line. This
   * method is useful in order to find out if a point is to the left
   * or right of a (directed) line. If the result is greater than 0
   * the point lies on the right. If it is smaller than 0 it is on the
   * left.
   */
  V getSignedDistance(const Vector2<V>& p1,
                          const Vector2<V>& p2) const
  {
      Vector2<V> directedLine(p2.x - p1.x, p2.y - p1.y);
      Vector2<V> normalizedDirectedLine = directedLine.normalize(1);
      Vector2<V> originToThis = p1 - *this;
      return ((normalizedDirectedLine.x * originToThis.y)
              - (normalizedDirectedLine.y * originToThis.x));
  }

  /** cosinus of the arc between this and the given vector */
  float cos_alpha(const Vector2<V>& vector) const
  {
      return (*(this) * vector /
          (this->abs() * vector.abs()));
  }

  /**
  * Computes the angle between this vector and other. The angle value
  * is between 0 and 2*PI growing counter clockwise.
  * \param other The other vector.
  * \return The angle in radian.
  */
  float angleToVector(const Vector2<V>& other) const
  {
      V signed_distance = this->getSignedDistance(Vector2<V>(0,0), other);
      float alpha = -acos(this->cos_alpha(other));
      if (signed_distance < 0.0f)
      {
          alpha = -alpha;
      }
      else
      {
          alpha += pi2;
      }
      return alpha;
  }

};

#endif // __Vector2_h__
