#include "BitBuffer.h"

#include <cmath>

BitBuffer::BitBuffer(int bitSize)
{
  offset = 0;
  size = bitSize;
  buffer = new unsigned char[(size - 1) / 8 + 1];
  externBuffer = false;
}

BitBuffer::BitBuffer(unsigned char* buffer, int bitOffset, int bitSize)
{
  offset = bitOffset;
  size = bitSize;
  this->buffer = buffer;
  externBuffer = true;
}

BitBuffer::BitBuffer(BitBuffer &buffer, int bitOffset, int bitSize)
{
  offset = buffer.offset + bitOffset;
  size = bitSize;
  this->buffer = buffer.buffer;
  if (offset >= 8)
  {
    this->buffer++;
    offset -= 8;
  }
  externBuffer = true;
}

BitBuffer::~BitBuffer()
{
  if (!externBuffer)
    delete[] buffer;
}

int BitBuffer::getSize() const
{
  return size;
}

unsigned char BitBuffer::getBit(int index) const
{
  const int pos = offset + index;
  const int bytePos = (pos >> 3);
  const int bitPos = (pos & 7);
  const unsigned char mask = (1 << bitPos);
  if ((buffer[bytePos] & mask) != 0)
    return 1;
  return 0;
}

void BitBuffer::setBit(int index, unsigned char value)
{
  const int pos = offset + index;
  const int bytePos = (pos >> 3);
  const int bitPos = (pos & 7);
  const unsigned char mask = (1 << bitPos);
  if (value == 0)
    buffer[bytePos] &= ~mask;
  else
    buffer[bytePos] |= mask;
}

int BitBuffer::getBits(int bitIndex, unsigned char *data, int n) const
{
  int i;
  unsigned int mask = 1;
  unsigned char* dst = data;
  for (i = 0; i < n && i < size - bitIndex; i++)
  {
    *dst &= ~mask;
    if (getBit(bitIndex + i) != 0)
      *dst |= mask;
    mask <<= 1;
    if (mask >= 256)
    {
      dst++;
      mask = 1;
    }
  }
  return i;
}

int BitBuffer::setBits(int bitIndex, const unsigned char *data, int n)
{
  int i;
  unsigned int mask = 1;
  const unsigned char* dst = data;
  for (i = 0; i < n && i < size - bitIndex; i++)
  {
    setBit(bitIndex + i, ((*dst & mask) != 0 ? 1 : 0));
    mask <<= 1;
    if (mask >= 256)
    {
      dst++;
      mask = 1;
    }
  }
  return i;
}

//----------------------------------------------------------------------

BitBool::BitBool() :
    BitBuffer(1)
{
}
BitBool::BitBool(unsigned char* buffer, int bitOffset) :
    BitBuffer(buffer, bitOffset, 1)
{
}
BitBool::BitBool(BitBuffer &buffer, int bitOffset) :
    BitBuffer(buffer, bitOffset, 1)
{
}
const BitBool& BitBool::operator=(const bool &v)
{
  setBit(0, (v ? 1 : 0));
  return *this;
}
BitBool::operator bool() const
{
  return (getBit(0) != 0);
}

//----------------------------------------------------------------------

BitInt::BitInt(int size) :
    BitBuffer(size)
{
}
BitInt::BitInt(unsigned char* buffer, int bitOffset, int size) :
    BitBuffer(buffer, bitOffset, size)
{
}
BitInt::BitInt(BitBuffer &buffer, int bitOffset, int size) :
    BitBuffer(buffer, bitOffset, size)
{
}
const BitInt& BitInt::operator=(const int &v)
{
  setBits(0, (const unsigned char*) &v, getSize());
  return *this;
}
BitInt::operator int() const
{
  int v = 0;
  getBits(0, (unsigned char*) &v, getSize());
  return v;
}

//----------------------------------------------------------------------

BitFloat::BitFloat(int mantissa, int exponent, bool sign) :
    BitBuffer(mantissa + exponent + (sign ? 1 : 0)), mantissa(mantissa), exponent(exponent), sign(
        sign)
{
}

BitFloat::BitFloat(unsigned char* buffer, int bitOffset, int mantissa, int exponent, bool sign) :
    BitBuffer(buffer, bitOffset, mantissa + exponent + (sign ? 1 : 0)), mantissa(mantissa), exponent(
        exponent), sign(sign)
{
}

BitFloat::BitFloat(BitBuffer &buffer, int bitOffset, int mantissa, int exponent, bool sign) :
    BitBuffer(buffer, bitOffset, mantissa + exponent + (sign ? 1 : 0)), mantissa(mantissa), exponent(
        exponent), sign(sign)
{
}

const BitFloat& BitFloat::operator=(const double &v)
{
  if (v == 0 || (!sign && v < 0))
  {
    int zero = 0;
    setBits(0, (unsigned char*) &zero, mantissa);
    setBits(mantissa, (unsigned char*) &zero, exponent);
    if (sign)
      setBits(mantissa + exponent, (unsigned char*) &zero, 1);
    return *this;
  }
  int m = 0;
  int e = 0;
  int s = 0;
  double value = v;
  //sign
  if (value < 0)
  {
    s = 1;
    value *= -1.0;
  }
  //find exponent
  double accuracy = 1.0 / (1 << (mantissa + 1));
  while (value >= 2.0 - accuracy)
  {
    value /= 2.0;
    e++;
  }
  while (value < 1.0 - 0.5 * accuracy)
  {
    value *= 2.0;
    e--;
  }
  m = (value - 1.0) * (1 << mantissa) + 0.5;
  e += (1 << (exponent - 1));
  if (e < 0 || (e == 0 && m == 0))
  {
    e = 0;
    m = 1; //smalles value, but not 0
  }
  if (e >= (1 << exponent))
  {
    e = (1 << exponent) - 1;
    m = (1 << mantissa) - 1; // largest value
  }
  setBits(0, (unsigned char*) &m, mantissa);
  setBits(mantissa, (unsigned char*) &e, exponent);
  if (sign)
    setBits(mantissa + exponent, (unsigned char*) &s, 1);
  return *this;
}

BitFloat::operator double() const
{
  int m = 0;
  int e = 0;
  int s = 0;
  getBits(0, (unsigned char*) &m, mantissa);
  getBits(mantissa, (unsigned char*) &e, exponent);
  if (sign)
    getBits(mantissa + exponent, (unsigned char*) &s, 1);
  if (m == 0 && e == 0 && s == 0)
    return 0;
  e -= (1 << (exponent - 1));
  double result = (1.0 + (double) m / (1 << mantissa)) * pow(2.0, e);
  return (s == 0 ? 1.0 : -1.0) * result;
}

//----------------------------------------------------------------------

BitVector2::BitVector2(const double halfLength, const double halfWidth, unsigned char* buffer,
    int bitOffset, int size) :
    BitBuffer(buffer, bitOffset, size), halfLength(halfLength), halfWidth(halfWidth)
{
}

BitVector2::BitVector2(const FieldDimensions *theFieldDimensions, int size) :
    BitBuffer(size), halfLength(theFieldDimensions->halfLength), halfWidth(
        theFieldDimensions->halfWidth)
{
}

BitVector2::BitVector2(const FieldDimensions *theFieldDimensions, unsigned char* buffer,
    int bitOffset, int size) :
    BitBuffer(buffer, bitOffset, size), halfLength(theFieldDimensions->halfLength), halfWidth(
        theFieldDimensions->halfWidth)
{
}

BitVector2::BitVector2(const FieldDimensions *theFieldDimensions, BitBuffer &buffer, int bitOffset,
    int size) :
    BitBuffer(buffer, bitOffset, size), halfLength(theFieldDimensions->halfLength), halfWidth(
        theFieldDimensions->halfWidth)
{
}

const BitVector2& BitVector2::operator=(const Vector2<double> &v)
{
  const double minX = -halfLength - 0.5;
  const double maxX = halfLength + 0.5;
  const double minY = -halfWidth - 0.5;
  const double maxY = halfWidth + 0.5;
  const double x = std::max(minX, std::min(maxX - 0.001, v.x)) - minX;
  const double y = std::max(minY, std::min(maxY - 0.001, v.y)) - minY;
  int p = offset;
  const int xBits = (getSize() + 1) / 2;
  const int yBits = getSize() - xBits;
  BitInt(buffer, p, xBits) = x / (maxX - minX) * (1 << xBits);
  p += xBits;
  BitInt(buffer, p, yBits) = y / (maxY - minY) * (1 << yBits);
  p += yBits;
  return *this;
}

BitVector2::operator Vector2<double>() const
{
  const double minX = -halfLength - 0.5;
  const double maxX = halfLength + 0.5;
  const double minY = -halfWidth - 0.5;
  const double maxY = halfWidth + 0.5;
  int p = offset;
  const int xBits = (getSize() + 1) / 2;
  const int yBits = getSize() - xBits;
  const int x = BitInt(buffer, p, xBits);
  p += xBits;
  const int y = BitInt(buffer, p, yBits);
  p += yBits;
  return Vector2<double>(((double) x + 0.5) / (1 << xBits) * (maxX - minX) + minX,
      ((double) y + 0.5) / (1 << yBits) * (maxY - minY) + minY);
}

//----------------------------------------------------------------------

BitPose2D::BitPose2D(const FieldDimensions *theFieldDimensions, int transBits, int rotBits) :
    BitBuffer(transBits + rotBits), halfLength(theFieldDimensions->halfLength), halfWidth(
        theFieldDimensions->halfWidth), transBits(transBits), rotBits(rotBits)
{
}

BitPose2D::BitPose2D(const FieldDimensions *theFieldDimensions, unsigned char* buffer,
    int bitOffset, int transBits, int rotBits) :
    BitBuffer(buffer, bitOffset, transBits + rotBits), halfLength(theFieldDimensions->halfLength), halfWidth(
        theFieldDimensions->halfWidth), transBits(transBits), rotBits(rotBits)
{
}

BitPose2D::BitPose2D(const FieldDimensions *theFieldDimensions, BitBuffer &buffer, int bitOffset,
    int transBits, int rotBits) :
    BitBuffer(buffer, bitOffset, transBits + rotBits), halfLength(theFieldDimensions->halfLength), halfWidth(
        theFieldDimensions->halfWidth), transBits(transBits), rotBits(rotBits)
{
}

const BitPose2D& BitPose2D::operator=(const Pose2D &v)
{
  const double r = std::max(0.0, std::min(pi2, v.rotation + pi));
  BitVector2(halfLength, halfWidth, buffer, offset, transBits) = v.translation;
  BitInt(buffer, offset + transBits, rotBits) = r / pi2 * (1 << rotBits);
  return *this;
}

BitPose2D::operator Pose2D() const
{
  Pose2D pose;
  pose.translation = BitVector2(halfLength, halfWidth, buffer, offset, transBits);
  const int r = BitInt(buffer, offset + transBits, rotBits);
  pose.rotation = ((double) r + 0.5) / (1 << rotBits) * pi2 - pi;
  return pose;
}

