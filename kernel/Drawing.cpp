/*
 * Drawing.cpp
 *
 *  Created on: Jul 23, 2013
 *      Author: sam
 */

#include "Drawing.h"

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sstream>

using namespace ime;
using namespace rcss::net;

Drawing* Drawing::instance = 0;

Drawing::Drawing() :
    sendAddr(0), enabled(false), agentNumber(0)
{
}

Drawing::~Drawing()
{
  done();
  if (sendAddr)
    delete sendAddr;
  sendAddr = 0;
}

Drawing& Drawing::getInstance()
{
  if (!instance)
    instance = new Drawing;
  return *instance;
}

void Drawing::deleteInstance()
{
  if (instance)
  {
    delete instance;
  }
  instance = 0;
}

void Drawing::initInstance(const std::string& gHost, const int& gPort, const std::string& teamName,
    const int& agentNumber)
{
  if (!enabled)
  {
    std::cout << "enabled Drawing=" << gHost << " " << gPort << " " << teamName << " "
        << agentNumber << std::endl;
    sendAddr = new Addr(gPort, gHost);
    gSocket.setNonBlocking(true);
    enabled = true;

    // Set prefix
    std::ostringstream oss;
    oss << teamName << "." << agentNumber << ".";
    prefix = oss.str();

  }
}

void Drawing::swapBuffers(std::string name)
{
  commandVector.push_back(new SwapBufferCommand(name));
}

void Drawing::send()
{
  if (enabled)
  {
    const int maxDrawingSize = 255;  // TODO what is the max size of one drawing?
    swapBuffers(prefix);

    unsigned char *pBuffer = sendBuffer;
    long numBytes = 0;

    DrawingCommandVector::iterator iter = commandVector.begin();
    while (iter < commandVector.end())
    {
      while (iter < commandVector.end() && numBytes < sendBufferSize - maxDrawingSize)
      {
        // fixMe: side
        //write to buffer
        const long n = (*iter)->writeToBuf(pBuffer);
        if (n > maxDrawingSize)
          printf("WARNING: n=%ld! Increase maxDrawingSize in Drawing::send.", n);
        pBuffer += n;
        numBytes += n;
        iter++;
      }
      //send buffer
      int l = gSocket.send((const char*) sendBuffer, numBytes, *sendAddr, 0, Socket::DONT_CHECK);

      if (l != numBytes)
        printf("ERROR: could not send drawing UDP packet (packet size %ld, return %d)!\n", numBytes,
            l);
      numBytes = 0;
      pBuffer = sendBuffer;
    }
  }

  clear();
}

void Drawing::clear()
{
  // release resources
  for (DrawingCommandVector::iterator iter = commandVector.begin(); iter != commandVector.end();
      ++iter)
    delete *iter;
  commandVector.clear();
}

void Drawing::done()
{
  clear();
  if (enabled)
  {
    gSocket.close();
    enabled = false;
    std::cout << "closed connection: Drawing::done()" << "\n";
  }
}

// Drawing methods fixMe: remove from the release version
void Drawing::circle(std::string name, float x, float y, float radius, float thickness,
    unsigned char r, unsigned char g, unsigned char b)
{
  name = prefix + name;
  commandVector.push_back(new CircleCommand(name, x, y, radius, thickness, r, g, b));
}

void Drawing::line(std::string name, float x1, float y1, float z1, float x2, float y2, float z2,
    unsigned char r, unsigned char g, unsigned char b, float thickness)
{
  name = prefix + name;
  commandVector.push_back(new LineCmd(name, x1, y1, z1, x2, y2, z2, r, g, b, thickness));
}

void Drawing::point(std::string name, float x, float y, float z, float size, unsigned char r,
    unsigned char g, unsigned char b)
{
  name = prefix + name;
  commandVector.push_back(new PointCommand(name, x, y, z, size, r, g, b));
}

void Drawing::sphere(std::string name, float x, float y, float z, float radius, unsigned char r,
    unsigned char g, unsigned char b)
{
  name = prefix + name;
  commandVector.push_back(new SphereCommand(name, x, y, z, radius, r, g, b));
}

void Drawing::annotation(std::string name, std::string text, float x, float y, float z,
    unsigned char r, unsigned char g, unsigned char b)
{
  name = prefix + name;
  commandVector.push_back(new AnnotationCommand(name, text, x, y, z, r, g, b));\
}

void Drawing::pose(const std::string &name, const Pose2D &pose, float radius, float thickness,
    unsigned char r, unsigned char g, unsigned char b)
{
  const Vector2<double> pointForward = pose * Vector2<double>(2.0f * radius, 0);
  circle(name, pose.translation.x, pose.translation.y, radius, thickness, r, g, b);
  line(name, pose.translation.x, pose.translation.y, 0, pointForward.x, pointForward.y, 0, r, g, b,
      thickness);
}
