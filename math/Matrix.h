/** 
 * \file Matrix.h
 * Template classes for various 3x3 matrices.
 *
 * \author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * \author <a href="mailto:thomas.kindler@gmx.de">Thomas Kindler</a> 
 * \author Max Risler
 *
 * Contains template class Matrix2x2 of type V
 *
 * @author <a href="mailto:Kai_Engel@gmx.de">Kai Engel</a>
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 *
 * Also contains:
 * Contains template class Matrix of type V and size mxn
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 * @author Colin Graf
 *
 * Borrowed from B-Human code base with modifications.
 * (Sam Abeyruwan)
 */

#ifndef __Matrix_h__
#define __Matrix_h__

#include "Vector2.h"
#include "Vector3.h"
#include <limits>

/** This class represents a 2x2-matrix */

template<class V = float> class Matrix2x2
{
  public:
    /** The columns of the matrix */
    Vector2<V> c[2];

    /** Default constructor. */
    Matrix2x2<V>()
    {
      c[0] = Vector2<V>(1, 0);
      c[1] = Vector2<V>(0, 1);
    }

    /**
     * Anti-lateral thinking constructor.
     */
    Matrix2x2<V>(const V& a11, const V& a12, const V& a21, const V& a22)
    {
      c[0].x = a11;
      c[1].x = a12;
      c[0].y = a21;
      c[1].y = a22;
    }

    //! Constructor
    /*!
     \param c0 the first column of the matrix.
     \param c1 the second column of the matrix.
     */
    Matrix2x2<V>(const Vector2<V>& c0, const Vector2<V>& c1)
    {
      c[0] = c0;
      c[1] = c1;
    }

    //! Assignment operator
    /*!
     \param other The other matrix that is assigned to this one
     \return A reference to this object after the assignment.
     */
    Matrix2x2<V>& operator=(const Matrix2x2<V>& other)
    {
      c[0] = other.c[0];
      c[1] = other.c[1];
      return *this;
    }

    //! Copy constructor
    /*!
     \param other The other matrix that is copied to this one
     */
    Matrix2x2<V>(const Matrix2x2<V>& other)
    {
      *this = other;
    }

    /**
     * Array-like member access.
     * \param  i index
     * \return reference to column
     */
    Vector2<V>& operator[](int i)
    {
      return c[i];
    }

    /**
     * const array-like member access.
     * \param  i index
     * \return reference to column
     */
    const Vector2<V>& operator[](int i) const
    {
      return c[i];
    }

    //! Multiplication of this matrix by vector.
    /*!
     \param vector The vector this one is multiplied by
     \return A reference to a new vector containing the result
     of the calculation.
     */
    Vector2<V> operator*(const Vector2<V>& vector) const
    {
      return (c[0] * vector.x + c[1] * vector.y);
    }

    //! Multiplication of this matrix by another matrix.
    /*!
     \param other The other matrix this one is multiplied by
     \return An object containing the result of the calculation.
     */
    Matrix2x2<V> operator*(const Matrix2x2<V>& other) const
    {
      Matrix2x2<V> returnMatrix;
      returnMatrix.c[0].x = c[0].x * other.c[0].x + c[1].x * other.c[0].y;
      returnMatrix.c[0].y = c[0].y * other.c[0].x + c[1].y * other.c[0].y;
      returnMatrix.c[1].x = c[0].x * other.c[1].x + c[1].x * other.c[1].y;
      returnMatrix.c[1].y = c[0].y * other.c[1].x + c[1].y * other.c[1].y;
      return returnMatrix;
    }

    //! Multiplication of this matrix by another matrix.
    /*!
     \param other The other matrix this one is multiplied by
     \return A reference this object after the calculation.
     */
    Matrix2x2<V> operator*=(const Matrix2x2<V>& other)
    {
      return *this = *this * other;
    }

    //! Multiplication of this matrix by a factor.
    /*!
     \param factor The factor this matrix is multiplied by
     \return A reference to this object after the calculation.
     */
    Matrix2x2<V>& operator*=(const V& factor)
    {
      c[0] *= factor;
      c[1] *= factor;
      return *this;
    }

    //! Division of this matrix by a factor.
    /*!
     \param factor The factor this matrix is divided by
     \return A reference to this object after the calculation.
     */
    Matrix2x2<V>& operator/=(const V& factor)
    {
      c[0] /= factor;
      c[1] /= factor;
      return *this;
    }

    //! Multiplication of this matrix by a factor.
    /*!
     \param factor The factor this matrix is multiplied by
     \return A new object that contains the result of the calculation.
     */
    Matrix2x2<V> operator*(const V& factor) const
    {
      return Matrix2x2<V>(*this) *= factor;
    }

    //! Division of this matrix by a factor.
    /*!
     \param factor The factor this matrix is divided by
     \return A new object that contains the result of the calculation.
     */
    Matrix2x2<V> operator/(const V& factor) const
    {
      return Matrix2x2<V>(*this) /= factor;
    }

    //! Computes the sum of two matrices
    /*!
     \param other Another matrix
     \return The sum
     */
    Matrix2x2<V> operator+(const Matrix2x2<V>& other) const
    {
      return Matrix2x2<V>(
          Vector2<V>(c[0].x + other.c[0].x, c[0].y + other.c[0].y),
          Vector2<V>(c[1].x + other.c[1].x, c[1].y + other.c[1].y));
    }

    //! Computes the difference of two matrices
    /*!
     \param other Another matrix
     \return The difference
     */
    Matrix2x2<V> operator-(const Matrix2x2<V>& other) const
    {
      return Matrix2x2<V>(
          Vector2<V>(c[0].x - other.c[0].x, c[0].y - other.c[0].y),
          Vector2<V>(c[1].x - other.c[1].x, c[1].y - other.c[1].y));
    }

    /**
     * Adds another matrix.
     *
     * \param  other  The other matrix that is added to this one
     * \return        A reference to this object after the calculation.
     */
    Matrix2x2<V>& operator+=(const Matrix2x2<V>& other)
    {
      c[0] += other.c[0];
      c[1] += other.c[1];
      return *this;
    }

    /**
     * Subtracts another matrix.
     *
     * \param  other  The other matrix that is subtracted from this one
     * \return        A reference to this object after the calculation.
     */
    Matrix2x2<V>& operator-=(const Matrix2x2<V>& other)
    {
      c[0] -= other.c[0];
      c[1] -= other.c[1];
      return *this;
    }

    //! Computes an inverted matrix.
    /*!
     \return An inverted matrix.
     */
    Matrix2x2<V> invert() const
    {
      V factor(det());
      if (abs(factor) < std::numeric_limits<V>::min())
        factor = std::numeric_limits<V>::min();
      else
        factor = 1.f / factor;
      return Matrix2x2<V>(Vector2<V>(factor * c[1].y, -factor * c[0].y),
          Vector2<V>(-factor * c[1].x, factor * c[0].x));
    }

    //! Comparison of another matrix with this one.
    /*!
     \param other The other matrix that will be compared to this one
     \return Whether the two matrices are equal.
     */
    bool operator==(const Matrix2x2<V>& other) const
    {
      return (c[0] == other.c[0] && c[1] == other.c[1]);
    }

    //! Comparison of another matrix with this one.
    /*!
     \param other The other matrix that will be compared to this one
     \return Whether the two matrixs are unequal.
     */
    bool operator!=(const Matrix2x2<V>& other) const
    {
      return !(*this == other);
    }

    /*! Transpose the matrix
     \return A new object containing transposed matrix
     */
    Matrix2x2<V> transpose() const
    {
      return Matrix2x2<V>(Vector2<V>(c[0].x, c[1].x),
          Vector2<V>(c[0].y, c[1].y));
    }

    //! Calculation of the determinant of this matrix.
    /*!
     \return The determinant.
     */
    V det() const
    {
      return c[0].x * c[1].y - c[1].x * c[0].y;
    }

    /** Sums up the elements on the main diagonal
     * @return The sum
     */
    V trace() const
    {
      return c[0].x + c[1].y;
    }
};

/**
 * This class represents a 3x3-matrix 
 *
 */
template<class V = double> class Matrix3x3
{
  public:
    /**
     * The columns of the matrix
     */
    Vector3<V> c[3];
    
    /**
     * Default constructor.
     */
    Matrix3x3<V>()
    {
      c[0] = Vector3<V>(1, 0, 0);
      c[1] = Vector3<V>(0, 1, 0);
      c[2] = Vector3<V>(0, 0, 1);
    }

    /**
     * Constructor.
     *
     * \param  c0  the first column of the matrix.
     * \param  c1  the second column of the matrix.
     * \param  c2  the third column of the matrix.
     */
    Matrix3x3<V>(const Vector3<V>& c0, const Vector3<V>& c1,
        const Vector3<V>& c2)
    {
      c[0] = c0;
      c[1] = c1;
      c[2] = c2;
    }

    /**
     * Anti-lateral thinking constructor.
     */
    Matrix3x3<V>(const V& a11, const V& a12, const V& a13, const V& a21,
        const V& a22, const V& a23, const V& a31, const V& a32, const V& a33)
    {
      c[0].x = a11;
      c[1].x = a12;
      c[2].x = a13;
      c[0].y = a21;
      c[1].y = a22;
      c[2].y = a23;
      c[0].z = a31;
      c[1].z = a32;
      c[2].z = a33;
    }

    /**
     * Assignment operator.
     *
     * \param  other   The other matrix that is assigned to this one
     * \return         A reference to this object after the assignment.
     */
    Matrix3x3<V>& operator=(const Matrix3x3<V>& other)
    {
      c[0] = other.c[0];
      c[1] = other.c[1];
      c[2] = other.c[2];
      return *this;
    }

    /**
     * Copy constructor.
     *
     * \param other The other matrix that is copied to this one
     */
    Matrix3x3<V>(const Matrix3x3<V>& other)
    {
      *this = other;
    }

    /**
     * Adds this matrix with another matrix.
     *
     * \param  other  The matrix this one is added to
     * \return         A new matrix containing the result
     *                 of the calculation.
     */
    Matrix3x3<V> operator+(const Matrix3x3<V>& other) const
    {
      return Matrix3x3<V>((*this).c[0] + other.c[0], (*this).c[1] + other.c[1],
          (*this).c[2] + other.c[2]);
    }
    /**
     * Adds another matrix to this matrix.
     *
     * \param  other  The other matrix that is added to this one
     * \return        A reference this object after the calculation.
     */
    Matrix3x3<V>& operator+=(const Matrix3x3<V>& other)
    {
      (*this).c[0] += other.c[0];
      (*this).c[1] += other.c[1];
      (*this).c[2] += other.c[2];
      return *this;
    }

    /**
     * Compute difference of this matrix and another one
     *
     * \param  other  The matrix which is substracted from this one
     * \return         A new matrix containing the result
     *                 of the calculation.
     */
    Matrix3x3<V> operator-(const Matrix3x3<V>& other) const
    {
      return Matrix3x3<V>((*this).c[0] - other.c[0], (*this).c[1] - other.c[1],
          (*this).c[2] - other.c[2]);
    }
    /**
     * Substracts another matrix from this one
     *
     * \param  other  The other matrix that is substracted from this one
     * \return        A reference this object after the calculation.
     */
    Matrix3x3<V>& operator-=(const Matrix3x3<V>& other)
    {
      (*this).c[0] -= other.c[0];
      (*this).c[1] -= other.c[1];
      (*this).c[2] -= other.c[2];
      return *this;
    }

    /**
     * Multiplication of this matrix by vector.
     *
     * \param  vector  The vector this one is multiplied by
     * \return         A new vector containing the result
     *                 of the calculation.
     */
    Vector3<V> operator*(const Vector3<V>& vector) const
    {
      return (c[0] * vector.x + c[1] * vector.y + c[2] * vector.z);
    }

    /**
     * Multiplication of this matrix by another matrix.
     *
     * \param  other  The other matrix this one is multiplied by
     * \return        A new matrix containing the result
     *                of the calculation.
     */
    Matrix3x3<V> operator*(const Matrix3x3<V>& other) const
    {
      return Matrix3x3<V>((*this) * other.c[0], (*this) * other.c[1],
          (*this) * other.c[2]);
    }

    /**
     * Multiplication of this matrix by another matrix.
     *
     * \param  other  The other matrix this one is multiplied by
     * \return        A reference this object after the calculation.
     */
    Matrix3x3<V>& operator*=(const Matrix3x3<V>& other)
    {
      return *this = *this * other;
    }

    /**
     * Multiplication of this matrix by a factor.
     *
     * \param  factor  The factor this matrix is multiplied by
     * \return         A reference to this object after the calculation.
     */
    Matrix3x3<V>& operator*=(const V& factor)
    {
      c[0] *= factor;
      c[1] *= factor;
      c[2] *= factor;
      return *this;
    }

    /**
     * Division of this matrix by a factor.
     *
     * \param  factor  The factor this matrix is divided by
     * \return         A reference to this object after the calculation.
     */
    Matrix3x3<V>& operator/=(const V& factor)
    {
      *this *= 1 / factor;
      return *this;
    }

    /**
     * Multiplication of this matrix by a factor.
     *
     * \param  factor  The factor this matrix is multiplied by
     * \return         A new object that contains the result of the calculation.
     */
    Matrix3x3<V> operator*(const V& factor) const
    {
      return Matrix3x3<V>(*this) *= factor;
    }

    /**
     * Division of this matrix by a factor.
     *
     * \param  factor  The factor this matrix is divided by
     * \return         A new object that contains the result of the calculation.
     */
    Matrix3x3<V> operator/(const V& factor) const
    {
      return Matrix3x3<V>(*this) /= factor;
    }

    /**
     * Comparison of another matrix with this one.
     *
     * \param  other  The other matrix that will be compared to this one
     * \return        Whether the two matrices are equal.
     */
    bool operator==(const Matrix3x3<V>& other) const
    {
      return (c[0] == other.c[0] && c[1] == other.c[1] && c[2] == other.c[2]);
    }

    /**
     * Comparison of another matrix with this one.
     *
     * \param  other  The other matrix that will be compared to this one
     * \return        Whether the two matrixs are unequal.
     */
    bool operator!=(const Matrix3x3<V>& other) const
    {
      return !(*this == other);
    }

    /**
     * Array-like member access.
     * \param  i index
     * \return reference to column
     */
    Vector3<V>& operator[](int i)
    {
      return c[i];
    }

    /**
     * Transpose the matrix
     *
     * \return  A new object containing transposed matrix
     */
    Matrix3x3<V> transpose() const
    {
      return Matrix3x3<V>(Vector3<V>(c[0].x, c[1].x, c[2].x),
          Vector3<V>(c[0].y, c[1].y, c[2].y),
          Vector3<V>(c[0].z, c[1].z, c[2].z));
    }

    /**
     * Calculation of the determinant of this matrix.
     *
     * \return The determinant.
     */
    V det() const
    {
      return c[0].x * (c[1].y * c[2].z - c[1].z * c[2].y)
          + c[0].y * (c[1].z * c[2].x - c[1].x * c[2].z)
          + c[0].z * (c[1].x * c[2].y - c[1].y * c[2].x);
    }

    /**
     * Calculate determinant of 2x2 Submatrix
     * | a b |
     * | c d |
     *
     * \return  determinant.
     */
    static V det2(V a, V b, V c, V d)
    {
      return a * d - b * c;
    }

    /**
     * Calculate the adjoint of this matrix.
     *
     * \return the adjoint matrix.
     */
    Matrix3x3<V> adjoint() const
    {
      return Matrix3x3<V>(
          Vector3<V>(det2(c[1].y, c[2].y, c[1].z, c[2].z),
              det2(c[2].x, c[1].x, c[2].z, c[1].z),
              det2(c[1].x, c[2].x, c[1].y, c[2].y)),
          Vector3<V>(det2(c[2].y, c[0].y, c[2].z, c[0].z),
              det2(c[0].x, c[2].x, c[0].z, c[2].z),
              det2(c[2].x, c[0].x, c[2].y, c[0].y)),
          Vector3<V>(det2(c[0].y, c[1].y, c[0].z, c[1].z),
              det2(c[1].x, c[0].x, c[1].z, c[0].z),
              det2(c[0].x, c[1].x, c[0].y, c[1].y)));

    }

    /**
     * Calculate the inverse of this matrix.
     *
     * \return The inverse matrix
     */
    Matrix3x3<V> invert() const
    {
      return adjoint().transpose() / det();
    }
};

/**
 * Representation for 3x3 RotationMatrices
 */
class RotationMatrix: public Matrix3x3<double>
{
  public:
    /**
     * Default constructor.
     */
    RotationMatrix()
    {
    }

    /**
     * Constructor.
     *
     * \param  c0  the first column of the matrix.
     * \param  c1  the second column of the matrix.
     * \param  c2  the third column of the matrix.
     */
    RotationMatrix(const Vector3<double>& c0, const Vector3<double>& c1,
        const Vector3<double>& c2);

    /**
     * Assignment operator.
     *
     * \param  other  The other matrix that is assigned to this one
     * \return        A reference to this object after the assignment.
     */
    RotationMatrix& operator=(const Matrix3x3<double>& other)
    {
      c[0] = other.c[0];
      c[1] = other.c[1];
      c[2] = other.c[2];
      return *this;
    }

    /**
     * Copy constructor.
     *
     * \param  other  The other matrix that is copied to this one
     */
    RotationMatrix(const Matrix3x3<double>& other)
    {
      *this = other;
    }

    /**
     * RotationMatrix from RPY-angles.
     *   Roll  rotates along z axis,
     *   Pitch rotates along y axis,
     *   Yaw   rotates along x axis
     *
     *   R(roll,pitch,yaw) = R(z,roll)*R(y,pitch)*R(x,yaw)
     *
     * \see  "Robotik 1 Ausgabe Sommersemester 2001" by Prof. Dr. O. von Stryk
     * \attention  RPY-angles are not clearly defined!
     */
    RotationMatrix(const double& yaw, const double& pitch, const double& roll);

    /**
     * RotationMatrix from rotation around any axis.
     * \param axis The axis.
     * \param angle The angle to rotate around the axis.
     */
    RotationMatrix(const Vector3<double>& axis, const double& angle);

    /**
     * RotationMatrix from rotation around any axis with an angle given as the length of the axis.
     * \param axis The axis.
     */
    RotationMatrix(const Vector3<double>& axis);

    /**
     * Invert the matrix.
     *
     * \note: Inverted rotation matrix is transposed matrix.
     */
    RotationMatrix invert() const
    {
      return transpose();
    }

    /**
     * Rotation around the x-axis.
     *
     * \param   angle  The angle this pose will be rotated by
     * \return  A reference to this object after the calculation.
     */
    RotationMatrix& rotateX(const double angle);

    /**
     * Rotation around the y-axis.
     *
     * \param   angle  The angle this pose will be rotated by
     * \return  A reference to this object after the calculation.
     */
    RotationMatrix& rotateY(const double angle);

    /**
     * Rotation around the z-axis.
     *
     * \param   angle  The angle this pose will be rotated by
     * \return  A reference to this object after the calculation.
     */
    RotationMatrix& rotateZ(const double angle);

    /**
     * Get the x-angle of a RotationMatrix.
     *
     * \return  The angle around the x-axis between the original
     *          and the rotated z-axis projected on the y-z-plane
     */
    double getXAngle() const;

    /**
     * Get the y-angle of a RotationMatrix.
     *
     * \return  The angle around the y-axis between the original
     *          and the rotated x-axis projected on the x-z-plane
     */
    double getYAngle() const;

    /**
     * Get the z-angle of a RotationMatrix.
     *
     * \return  The angle around the z-axis between the original
     *          and the rotated x-axis projected on the x-y-plane
     */
    double getZAngle() const;

    /**
     * Create and return a RotationMatrix, rotated around x-axis
     *
     * \param   angle
     * \return  rotated RotationMatrix
     */
    static RotationMatrix fromRotationX(const double angle)
    {
      return RotationMatrix().rotateX(angle);
    }

    /**
     * Create and return a RotationMatrix, rotated around y-axis
     *
     * \param   angle
     * \return  rotated RotationMatrix
     */
    static RotationMatrix fromRotationY(const double angle)
    {
      return RotationMatrix().rotateY(angle);
    }

    /**
     * Create and return a RotationMatrix, rotated around z-axis
     *
     * \param   angle
     * \return  rotated RotationMatrix
     */
    static RotationMatrix fromRotationZ(const double angle)
    {
      return RotationMatrix().rotateZ(angle);
    }

    /**
     * Get the euler axis and angle of a RotationMatrix.
     *
     * \param   axis  Will be set to the rotation axis
     * \param   angle Will be set to the rotation angle around axis
     */
    void getAngleAxis(Vector3<double>& axis, double& angle) const;
    
    /**
    * Converts the rotation matrix into the single vector format.
    * @return The rotation matrix as angleAxis.
    */
    Vector3<> getAngleAxis() const;    
};

#include "Vector.h"
#include <cassert>
#include "MVTools.h"

/** This class represents a mxn-matrix */
template<int m = 2, int n = 2, class V = float> class Matrix
{
  public:
    /** The columns of the matrix */
    Vector<m, V> c[n];

    /** Default constructor. */
    Matrix<m, n, V>()
    {
    }

    Matrix<m, n, V>(V v)
    {
      //ASSERT(m == n);
      const int mnm = n < m ?
          n : m;
      for (int i = 0; i < mnm; ++i)
        c[i][i] = v;
    }

    /** Constructor */
    Matrix<m, n, V>(const Vector<m, V>& c0, const Vector<m, V>& c1)
    {
      //ASSERT(n == 2);
      c[0] = c0;
      c[1] = c1;
    }

    /** Constructor */
    Matrix<m, n, V>(const Vector<m, V>& c0, const Vector<m, V>& c1
        , const Vector<m, V>& c2)
    {
      //ASSERT(n == 3);
      c[0] = c0;
      c[1] = c1;
      c[2] = c2;
    }

    /** Constructor */
    Matrix<m, n, V>(const Vector<m, V>& c0, const Vector<m, V>& c1
        , const Vector<m, V>& c2, const Vector<m, V>& c3)
    {
      //ASSERT(n == 4);
      c[0] = c0;
      c[1] = c1;
      c[2] = c2;
      c[3] = c3;
    }

    /** Constructor */
    /*
     Matrix<m, n, V>(const Vector<m, V>& c0, const Vector<m, V>& c1, const Vector<m, V>& c2, const Vector<m, V>& c3, ...)
     {
     c[0] = c0;
     c[1] = c1;
     c[2] = c2;
     c[3] = c3;
     va_list vl;
     va_start(vl, c3);
     for(int i = 4; i < n; ++i)
     (*this)[i] = va_arg(vl, const Vector<m, V>&);
     va_end(vl);
     }
     */

    /**
     * Assignment operator
     * @param other The other matrix that is assigned to this one
     * @return A reference to this object after the assignment.
     */
    Matrix<m, n, V>& operator=(const Matrix<m, n, V>& other)
    {
      for (int i = 0; i < n; ++i)
        c[i] = other.c[i];
      return *this;
    }

    /**
     * Array-like member access.
     * @param i index
     * @return reference to column
     */
    inline Vector<m, V>& operator[](int i)
    {
      return c[i];
    }

    /**
     * const array-like member access.
     * @param i index
     * @return reference to column
     */
    inline const Vector<m, V>& operator[](int i) const
    {
      return c[i];
    }

    /**
     * Multiplication of this matrix by vector.
     * @param vector The vector this one is multiplied by
     * @return A reference to a new vector containing the result of the calculation.
     */
    Vector<m, V> operator*(const Vector<n, V>& vector) const
    {
      Vector<m, V> result = c[0] * vector[0];
      for (int i = 1; i < n; ++i)
        result += c[i] * vector[i];
      return result;
    }

    /**
     * Multiplication of this matrix by another matrix.
     * @param other The other matrix this one is multiplied by
     * @return An object containing the result of the calculation.
     */
    template<int o> Matrix<m, o, V> operator*(
        const Matrix<n, o, V>& other) const
    {
      Matrix<m, o, V> result;
      for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
          for (int k = 0; k < o; ++k)
            result.c[k][j] += c[i][j] * other.c[k][i];
      return result;
    }

    /**
     * Multiplication of this matrix by another matrix.
     * @param other The other matrix this one is multiplied by
     * @return A reference this object after the calculation.
     */
    Matrix<n, n, V> operator*=(const Matrix<n, n, V>& other)
    {
      return *this = *this * other;
    }

    /**
     * Multiplication of this matrix by a factor.
     * @param factor The factor this matrix is multiplied by
     * @return A reference to this object after the calculation.
     */
    Matrix<m, n, V>& operator*=(const V& factor)
    {
      for (int i = 0; i < n; ++i)
        c[i] *= factor;
      return *this;
    }

    /**
     * Multiplication of this matrix by a factor.
     * @param factor The factor this matrix is multiplied by
     * @return A new object that contains the result of the calculation.
     */
    Matrix<m, n, V> operator*(const V& factor) const
    {
      return Matrix<m, n, V>(*this) *= factor;
    }

    /**
     * Division of this matrix by a factor.
     * @param factor The factor this matrix is divided by
     * @return A reference to this object after the calculation.
     */
    Matrix<m, n, V>& operator/=(const V& factor)
    {
      for (int i = 0; i < n; ++i)
        c[i] /= factor;
      return *this;
    }

    /**
     * Division of this matrix by a factor.
     * @param factor The factor this matrix is divided by
     * @return A new object that contains the result of the calculation.
     */
    Matrix<m, n, V> operator/(const V& factor) const
    {
      return Matrix<m, n, V>(*this) /= factor;
    }

    /**
     * Adds another matrix.
     * @param other The other matrix that is added to this one
     * @return A reference to this object after the calculation.
     */
    Matrix<m, n, V>& operator+=(const Matrix<m, n, V>& other)
    {
      for (int i = 0; i < n; ++i)
        c[i] += other.c[i];
      return *this;
    }

    /**
     * Computes the sum of two matrices
     * @param other Another matrix
     * @return The sum
     */
    Matrix<m, n, V> operator+(const Matrix<m, n, V>& other) const
    {
      return Matrix<m, n, V>(*this) += other;
    }

    /**
     * Subtracts another matrix.
     * @param other The other matrix that is subtracted from this one
     * @return A reference to this object after the calculation.
     */
    Matrix<m, n, V>& operator-=(const Matrix<m, n, V>& other)
    {
      for (int i = 0; i < n; ++i)
        c[i] -= other.c[i];
      return *this;
    }

    /**
     * Computes the difference of two matrices
     * @param other Another matrix
     * @return The difference
     */
    Matrix<m, n, V> operator-(const Matrix<m, n, V>& other) const
    {
      return Matrix<m, n, V>(*this) -= other;
    }

    /**
     * Comparison of another matrix with this one.
     * @param other The other matrix that will be compared to this one
     * @return Whether the two matrices are equal.
     */
    bool operator==(const Matrix<m, n, V>& other) const
    {
      for (int i = 0; i < n; ++i)
        if (c[i] != other.c[i])
          return false;
      return true;
    }

    /**
     * Comparison of another matrix with this one.
     * @param other The other matrix that will be compared to this one
     * @return Whether the two matrixs are unequal.
     */
    bool operator!=(const Matrix<m, n, V>& other) const
    {
      for (int i = 0; i < n; ++i)
        if (c[i] != other.c[i])
          return true;
      return false;
    }

    /**
     * Transpose the matrix
     * @return A new object containing transposed matrix
     */
    Matrix<n, m, V> transpose() const
    {
      Matrix<n, m, V> result;
      for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
          result.c[j][i] = c[i][j];
      return result;
    }

    /**
     * Calculation of the determinant of this matrix.
     * @return The determinant.
     */
    V det() const;

    /**
     * Calculate the adjoint of this matrix.
     * @return the adjoint matrix.
     */
    Matrix<m, n, V> adjoint() const;

    /**
     * Calculate the inverse of this matrix.
     * @return The inverse matrix
     */
    Matrix<m, n, V> invert() const;

    /**
     * Solves the system A*x=b where A is the actual matrix
     * @param b Vector b
     * @param x Solution x
     * @return Whether solving was possible
     */
    bool solve(const Vector<n, V>& b, Vector<n, V>& x) const;
};

// implementation
template<class V> static V det2(V a, V b, V c, V d)
{
  return a * d - b * c;
}

template<int m, int n, class V> V Matrix<m, n, V>::det() const
{
  if(m == 3 && n == 3)
  {
    return
      c[0].x * (c[1].y * c[2].z - c[1].z * c[2].y) +
      c[0].y * (c[1].z * c[2].x - c[1].x * c[2].z) +
      c[0].z * (c[1].x * c[2].y - c[1].y * c[2].x);
  }

  assert(false);
  // not implemented
  /*
   // create copy of actual matrix
   Matrix_nxn<T, N> m(*this);

   // initialize ranking vector
   Vector_n<int, N> ranking;
   size_t i;
   for (i = 0; i < N; ++i)
   ranking[i] = i;

   T z = T();
   bool bPositive = true;
   int c;
   int r;
   for (c = 0; c < (int)(N-1); ++c)
   {
   // find row containing highest value
   int maxRow = c;
   T maxValue = m[ranking[maxRow]][c];
   if (maxValue < z)
   maxValue = -maxValue;
   for (r = c+1; r < (int)N; ++r)
   {
   T value = m[ranking[r]][c];
   if (value < z)
   value = -value;
   if (value > maxValue)
   {
   maxRow = r;
   maxValue = value;
   }
   }

   // if maximum value zero --> determinant is zero
   if (MVTools::isNearZero(maxValue))
   return z;
   / *if (maxValue == z)
   return z;* /

   // swap rows in ranking
   if (c != maxRow)
   {
   int temp = ranking[c];
   ranking[c] = ranking[maxRow];
   ranking[maxRow] = temp;
   bPositive = !bPositive;
   }

   // process all following rows
   for (r = c+1; r < (int)N; ++r)
   {
   // calc factor for subtracting
   T factor = m[ranking[r]][c] / m[ranking[c]][c];
   if (MVTools::isNearInf(factor))
   {
   if (MVTools::isNearPosInf(factor))
   throw MVException(MVException::PosInfValue);
   else
   throw MVException(MVException::NegInfValue);
   }

   // change matrix
   m[ranking[r]][c] = T();
   for (int c2 = c+1; c2 < (int)N; ++c2)
   {
   m[ranking[r]][c2] -= factor*m[ranking[c]][c2];
   T sub;
   sub = factor*m[ranking[c]][c2];
   if (MVTools::isNearInf(sub))
   {
   if (MVTools::isNearPosInf(sub))
   throw MVException(MVException::PosInfValue);
   else
   throw MVException(MVException::NegInfValue);
   }
   }
   }
   }
   // if last entry of matrix zero --> determinant is zero
   if (MVTools::isNearZero(m[ranking[N-1]][N-1]))
   return z;
   / *if (m[ranking[N-1]][N-1] == z)
   return z;* /
   // matrix has triangle form
   // calculate determinant
   T res = m[ranking[0]][0];
   for (r = 1; r < (int)N; ++r)
   {
   res *= m[ranking[r]][r];
   if (MVTools::isNearInf(res))
   {
   if (MVTools::isNearPosInf(res))
   throw MVException(MVException::PosInfValue);
   else
   throw MVException(MVException::NegInfValue);
   }
   }
   if (!bPositive)
   res = -res;
   return res;

   */
  return V(0);
}

template<int m, int n, class V> Matrix<m, n, V> Matrix<m, n, V>::adjoint() const
{
  ASSERT(false);
  // not implemented
  return Matrix<m, n, V>();
}

template<int m, int n, class V> Matrix<m, n, V> Matrix<m, n, V>::invert() const
{
  Matrix<n, n, V> left(*this);
  Matrix<n, n, V> right(V(1)); // I
  Vector<n, int> ranking;

  for (int i = 0; i < n; ++i)
    ranking[i] = i;

  int r, r2, maxrow;
  for (r = 0; r < n - 1; ++r)
  {
    // find highest value
    V maxval = left[ranking[r]][r];
    maxrow = r;
    if (maxval < V(0))
      maxval = -maxval;
    for (r2 = r + 1; r2 < n; ++r2)
    {
      float val = left[ranking[r2]][r];
      if (val < V(0))
        val = -val;
      if (val > maxval)
      {
        maxval = val;
        maxrow = r2;
      }
    }
    // swap rows
    int temp = ranking[r];
    ranking[r] = ranking[maxrow];
    ranking[maxrow] = temp;

    if (MVTools::isNearZero(left[ranking[r]][r]))
    {
      if (MVTools::isNearNegZero(left[ranking[r]][r]))
        throw MVException(MVException::DivByNegZero);
      else
        throw MVException(MVException::DivByPosZero);
    }


    for (r2 = r + 1; r2 < n; ++r2)
    {
      // calc factor for subtracting
      assert(left[ranking[r]][r] != 0.f);
      V factor = left[ranking[r2]][r] / left[ranking[r]][r];

      if (MVTools::isNearInf(factor))
      {
        if (MVTools::isNearPosInf(factor))
          throw MVException(MVException::PosInfValue);
        else
          throw MVException(MVException::NegInfValue);
      }

      // change left matrix
      left[ranking[r2]] -= left[ranking[r]] * factor;

      // change right matrix
      right[ranking[r2]] -= right[ranking[r]] * factor;
    }
  }
  // matrix has triangle form
  // bring to diagonal form
  for (r = n - 1; r > 0; --r)
  {
    if (MVTools::isNearZero(left[ranking[r]][r]))
    {
      if (MVTools::isNearNegZero(left[ranking[r]][r]))
        throw MVException(MVException::DivByNegZero);
      else
        throw MVException(MVException::DivByPosZero);
    }

    for (r2 = r - 1; r2 >= 0; --r2)
    {
      assert(left[ranking[r]][r] != 0.f);
      V factor = left[ranking[r2]][r] / left[ranking[r]][r];

      if (MVTools::isNearInf(factor))
      {
        if (MVTools::isNearPosInf(factor))
          throw MVException(MVException::PosInfValue);
        else
          throw MVException(MVException::NegInfValue);
      }

      // change left matrix
      left[ranking[r2]] -= left[ranking[r]] * factor;

      // change right matrix
      right[ranking[r2]] -= right[ranking[r]] * factor;
    }
  }
  // matrix has diagonal form
  // set entries of left matrix to 1 and apply multiplication to right
  Matrix<n, n, V> res;
  for (r = 0; r < n; ++r)
  {
    res[r] = right[ranking[r]];
    assert(left[ranking[r]][r] != 0.f);

    if (MVTools::isNearZero(left[ranking[r]][r]))
    {
      if (MVTools::isNearNegZero(left[ranking[r]][r]))
        throw MVException(MVException::DivByNegZero);
      else
        throw MVException(MVException::DivByPosZero);
    }

    res[r] /= left[ranking[r]][r];
  }
  return res;
}

template<int m, int n, class V> bool Matrix<m, n, V>::solve(
    const Vector<n, V>& b2, Vector<n, V>& x) const
{

  // create copy of actual matrix
  Vector<n, V> b = b2;
  Matrix<n, n, V> matrix = *this;

  // initialize ranking vector
  Vector<n, int> ranking;
  for (int i = 0; i < n; ++i)
    ranking[i] = i;

  for (int c = 0; c < n - 1; ++c)
  {
    // find row containing highest value
    int maxRow = c;
    V maxValue = matrix[c][ranking[maxRow]];
    if (maxValue < V())
      maxValue = -maxValue;
    for (int r = c + 1; r < n; ++r)
    {
      V value = matrix[c][ranking[r]];
      if (value < V())
        value = -value;
      if (value > maxValue)
      {
        maxRow = r;
        maxValue = value;
      }
    }

    // if maximum value zero --> matrix is singular
    if (maxValue == V())
      return false;

    // swap rows in ranking
    int temp = ranking[c];
    ranking[c] = ranking[maxRow];
    ranking[maxRow] = temp;

    // process all following rows
    for (int r = c + 1; r < n; ++r)
    {
      // calc factor for subtracting
      V factor = matrix[c][ranking[r]] / matrix[c][ranking[c]];
      V sub = factor * b[ranking[c]];

      // change vector b
      b[ranking[r]] -= sub;

      // change matrix
      matrix[c][ranking[r]] = V();
      for (int c2 = c + 1; c2 < n; ++c2)
      {
        sub = factor * matrix[c2][ranking[c]];
        matrix[c2][ranking[r]] -= sub;
      }
    }
  }

  // if last entry of matrix zero --> matrix is singular
  if (matrix[n - 1][ranking[n - 1]] == V())
    return false;

  // matrix has triangle form
  // calculate solutions
  b[ranking[n - 1]] /= matrix[n - 1][ranking[n - 1]];
  for (int r = n - 2; r >= 0; --r)
  {
    V sum = V();
    for (int c = r + 1; c < n; ++c)
      sum += matrix[c][ranking[r]] * b[ranking[c]];
    b[ranking[r]] = (b[ranking[r]] - sum) / matrix[r][ranking[r]];
  }

  // create vector with correct order
  for (int r = 0; r < n; ++r)
    x[r] = b[ranking[r]];
  return true;
}
// typedefs
typedef Matrix<4, 4, double> Matrix4x4;
typedef Matrix<3, 4, double> Matrix3x4;

typedef Matrix<1, 1, float> Matrix1x1f;
typedef Matrix<2, 2, float> Matrix2x2f;
typedef Matrix<2, 1, float> Matrix2x1f;
typedef Matrix<1, 2, float> Matrix1x2f;
typedef Matrix<3, 3, float> Matrix3x3f;
typedef Matrix<3, 2, float> Matrix3x2f;
typedef Matrix<3, 1, float> Matrix3x1f;
typedef Matrix<2, 3, float> Matrix2x3f;
typedef Matrix<1, 3, float> Matrix1x3f;
typedef Matrix<4, 4, float> Matrix4x4f;
typedef Matrix<4, 3, float> Matrix4x3f;
typedef Matrix<4, 2, float> Matrix4x2f;
typedef Matrix<4, 1, float> Matrix4x1f;
typedef Matrix<3, 4, float> Matrix3x4f;
typedef Matrix<2, 4, float> Matrix2x4f;
typedef Matrix<1, 4, float> Matrix1x4f;

#endif // __Matrix_h__
