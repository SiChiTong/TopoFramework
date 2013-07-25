/*
 * Drawing.h
 *
 *  Created on: Jul 23, 2013
 *      Author: sam
 */

#ifndef DRAWING_H_
#define DRAWING_H_

#include <iostream>
#include <vector>
#include <cstring>
#include <cstdio>

#include "math/Pose2D.h"
#include "utils/net/UDPSocket.hpp"

namespace ime
{

// Drawing commands
class DrawingCommand
{
  public:
    DrawingCommand(std::string name) :
        name(name)
    {
    }

    virtual ~DrawingCommand()
    {
    }

    const std::string& getName()
    {
      return name;
    }

    virtual int writeToBuf(unsigned char* buf) = 0;

  protected:
    std::string name;

    inline int writeFloatToBuf(unsigned char* buf, float value)
    {
      // floats are written as characters using 5 most significant digits to
      // avoid issues with different systems and floating-point representations
      char temp[20];
      sprintf(temp, "%6f", value);
      memcpy(buf, temp, 6);
      return 6;
    }

    inline int writeCharToBuf(unsigned char* buf, unsigned char value)
    {
      *buf = value;
      return 1;
    }

    inline int writeStringToBuf(unsigned char* buf, std::string* value)
    {
      // all strings are terminated by a 0 byte
      int written = value->copy((char*) buf, name.size(), 0);
      written += writeCharToBuf(buf + written, 0);
      return written;
    }
};

class SwapBufferCommand: public DrawingCommand
{
  public:
    SwapBufferCommand(std::string name) :
        DrawingCommand(name)
    {
    }

    inline int writeToBuf(unsigned char* buf)
    {
      int l = 0;
      l += writeCharToBuf(buf + l, 0);
      l += writeCharToBuf(buf + l, 0);
      l += writeStringToBuf(buf + l, &name);
      return l;
    }
};

class SphereCommand: public DrawingCommand
{
  public:
    SphereCommand(std::string name, float x, float y, float z, float radius, unsigned char r,
        unsigned char g, unsigned char b) :
        DrawingCommand(name), x(x), y(y), z(z), radius(radius), r(r), g(g), b(b)
    {
    }

    inline int writeToBuf(unsigned char* buf)
    {
      int l = 0;
      l += writeCharToBuf(buf + l, 1);
      l += writeCharToBuf(buf + l, 3);
      l += writeFloatToBuf(buf + l, x);
      l += writeFloatToBuf(buf + l, y);
      l += writeFloatToBuf(buf + l, z);
      l += writeFloatToBuf(buf + l, radius);
      l += writeCharToBuf(buf + l, r);
      l += writeCharToBuf(buf + l, g);
      l += writeCharToBuf(buf + l, b);
      l += writeStringToBuf(buf + l, &name);
      return l;
    }

  protected:
    float x, y, z, radius;
    unsigned char r, g, b;
};

class PointCommand: public DrawingCommand
{
  public:
    PointCommand(std::string name, float x, float y, float z, float size, unsigned char r,
        unsigned char g, unsigned char b) :
        DrawingCommand(name), x(x), y(y), z(z), size(size), r(r), g(g), b(b)
    {
    }

    inline int writeToBuf(unsigned char* buf)
    {
      int l = 0;
      l += writeCharToBuf(buf + l, 1);
      l += writeCharToBuf(buf + l, 2);
      l += writeFloatToBuf(buf + l, x);
      l += writeFloatToBuf(buf + l, y);
      l += writeFloatToBuf(buf + l, z);
      l += writeFloatToBuf(buf + l, size);
      l += writeCharToBuf(buf + l, r);
      l += writeCharToBuf(buf + l, g);
      l += writeCharToBuf(buf + l, b);
      l += writeStringToBuf(buf + l, &name);
      return l;
    }

  protected:
    float x, y, z, size;
    unsigned char r, g, b;
};

class LineCmd: public DrawingCommand
{
  public:
    LineCmd(std::string name, float x1, float y1, float z1, float x2, float y2, float z2,
        unsigned char r, unsigned char g, unsigned char b, float thickness) :
        DrawingCommand(name), x1(x1), y1(y1), z1(z1), x2(x2), y2(y2), z2(z2), r(r), g(g), b(b), thickness(
            thickness)
    {
    }

    inline int writeToBuf(unsigned char* buf)
    {
      int l = 0;
      l += writeCharToBuf(buf + l, 1);
      l += writeCharToBuf(buf + l, 1);
      l += writeFloatToBuf(buf + l, x1);
      l += writeFloatToBuf(buf + l, y1);
      l += writeFloatToBuf(buf + l, z1);
      l += writeFloatToBuf(buf + l, x2);
      l += writeFloatToBuf(buf + l, y2);
      l += writeFloatToBuf(buf + l, z2);
      l += writeFloatToBuf(buf + l, thickness);
      l += writeCharToBuf(buf + l, r);
      l += writeCharToBuf(buf + l, g);
      l += writeCharToBuf(buf + l, b);
      l += writeStringToBuf(buf + l, &name);
      return l;
    }

  protected:
    float x1, y1, z1, x2, y2, z2;
    unsigned char r, g, b;
    float thickness;
};

class CircleCommand: public DrawingCommand
{
  public:
    CircleCommand(std::string name, float x, float y, float radius, float thickness,
        unsigned char r, unsigned char g, unsigned char b) :
        DrawingCommand(name), x(x), y(y), radius(radius), thickness(thickness), r(r), g(g), b(b)
    {
    }

    inline int writeToBuf(unsigned char* buf)
    {
      int l = 0;
      l += writeCharToBuf(buf + l, 1);
      l += writeCharToBuf(buf + l, 0);
      l += writeFloatToBuf(buf + l, x);
      l += writeFloatToBuf(buf + l, y);
      l += writeFloatToBuf(buf + l, radius);
      l += writeFloatToBuf(buf + l, thickness);
      l += writeCharToBuf(buf + l, r);
      l += writeCharToBuf(buf + l, g);
      l += writeCharToBuf(buf + l, b);
      l += writeStringToBuf(buf + l, &name);
      return l;
    }

  protected:
    float x, y, radius, thickness;
    unsigned char r, g, b;
};

class AnnotationCommand: public DrawingCommand
{
  public:
    AnnotationCommand(std::string name, std::string text, float x, float y, float z,
        unsigned char r, unsigned char g, unsigned char b) :
        DrawingCommand(name), text(text), x(x), y(y), z(z), r(r), g(g), b(b)
    {
    }

    inline int writeToBuf(unsigned char* buf)
    {
      int l = 0;
      l += writeCharToBuf(buf + l, 2);
      l += writeCharToBuf(buf + l, 0);
      l += writeFloatToBuf(buf + l, x);
      l += writeFloatToBuf(buf + l, y);
      l += writeFloatToBuf(buf + l, z);
      l += writeCharToBuf(buf + l, r);
      l += writeCharToBuf(buf + l, g);
      l += writeCharToBuf(buf + l, b);
      l += writeStringToBuf(buf + l, &text);
      l += writeStringToBuf(buf + l, &name);
      return l;
    }

  protected:
    std::string text;
    float x, y, z;
    unsigned char r, g, b;
};

class Drawing
{
  protected:
    rcss::net::UDPSocket gSocket;
    rcss::net::Addr* sendAddr;
    bool enabled;
    std::string teamName;
    int agentNumber;
    std::string prefix;
    typedef std::vector<DrawingCommand*> DrawingCommandVector;
    DrawingCommandVector commandVector;
    enum
    {
      sendBufferSize = 512 // 512 is max packet size for roboviz (from download)
    };
    unsigned char sendBuffer[sendBufferSize];

    void swapBuffers(std::string name);

  public:
    static Drawing& getInstance();
    static void deleteInstance();
    void initInstance(const std::string& gHost, const int& gPort, const std::string& teamName,
        const int& agentNumber);
    void send();
    void clear();
    void done();
    // Drawing methods
    void circle(std::string name, float x, float y, float radius, float thickness, unsigned char r,
        unsigned char g, unsigned char b);
    void line(std::string name, float x1, float y1, float z1, float x2, float y2, float z2,
        unsigned char r, unsigned char g, unsigned char b, float thickness);
    void point(std::string name, float x, float y, float z, float size, unsigned char r,
        unsigned char g, unsigned char b);
    void sphere(std::string name, float x, float y, float z, float radius, unsigned char r,
        unsigned char g, unsigned char b);
    void annotation(std::string name, std::string text, float x, float y, float z, unsigned char r,
        unsigned char g, unsigned char b);
    void pose(const std::string &name, const Pose2D &pose, float radius, float thickness,
        unsigned char r, unsigned char g, unsigned char b);

  private:
    Drawing();
    ~Drawing();
    Drawing(Drawing const& that);
    Drawing& operator=(Drawing const& that);
    static Drawing* instance;
};

}  // namespace ime

#endif /* DRAWING_H_ */
