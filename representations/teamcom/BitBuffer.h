#ifndef BITBUFFER_H
#define BITBUFFER_H

#include "representations/rcss/FieldDimensions.h"

class BitBuffer
{
  protected:
    int size;
    int offset;
    bool externBuffer;
    unsigned char* buffer;
  public:
    BitBuffer(int bitSize);
    BitBuffer(unsigned char* buffer, int bitOffset, int bitSize);
    BitBuffer(BitBuffer &buffer, int bitOffset, int bitSize);
    ~BitBuffer();
    int getSize() const;
    unsigned char getBit(int index) const;
    void setBit(int index, unsigned char value);
    int getBits(int bitIndex, unsigned char *data, int n) const;
    int setBits(int bitIndex, const unsigned char *data, int n);
};

class BitBool: public BitBuffer
{
  public:
    BitBool();
    BitBool(unsigned char* buffer, int bitOffset);
    BitBool(BitBuffer &buffer, int bitOffset);
    const BitBool& operator=(const bool &v);
    operator bool() const;
};

class BitInt: public BitBuffer
{
  public:
    BitInt(int size);
    BitInt(unsigned char* buffer, int bitOffset, int size);
    BitInt(BitBuffer &buffer, int bitOffset, int size);
    const BitInt& operator=(const int &v);
    operator int() const;
};

class BitFloat: public BitBuffer
{
  private:
    int mantissa;
    int exponent;
    bool sign;
  public:
    BitFloat(int mantissa, int exponent, bool sign);
    BitFloat(unsigned char* buffer, int bitOffset, int mantissa, int exponent, bool sign);
    BitFloat(BitBuffer &buffer, int bitOffset, int mantissa, int exponent, bool sign);
    const BitFloat& operator=(const double &v);
    operator double() const;
};

class BitVector2: public BitBuffer
{
  private:
    double halfLength;
    double halfWidth;
  public:
    BitVector2(double halfLength, double halfWidth, unsigned char* buffer, int bitOffset, int size);
    BitVector2(const FieldDimensions *theFieldDimensions, int size);
    BitVector2(const FieldDimensions *theFieldDimensions, unsigned char* buffer, int bitOffset,
        int size);
    BitVector2(const FieldDimensions *theFieldDimensions, BitBuffer &buffer, int bitOffset,
        int size);
    const BitVector2& operator=(const Vector2<double> &v);
    operator Vector2<double>() const;
};

class BitPose2D: public BitBuffer
{
  private:
    double halfLength;
    double halfWidth;
    int transBits, rotBits;
  public:
    BitPose2D(const FieldDimensions *theFieldDimensions, int transBits, int rotBits);
    BitPose2D(const FieldDimensions *theFieldDimensions, unsigned char* buffer, int bitOffset,
        int transBits, int rotBits);
    BitPose2D(const FieldDimensions *theFieldDimensions, BitBuffer &buffer, int bitOffset,
        int transBits, int rotBits);
    const BitPose2D& operator=(const Pose2D &v);
    operator Pose2D() const;
};

#endif

