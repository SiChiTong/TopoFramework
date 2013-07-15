/*
 * Communication.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#include "Communication.h"

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using namespace ime;
using namespace rcss::net;
using std::cout;
using std::endl;
using std::cerr;

Communication* Communication::instance = 0;

Communication::Communication()
{
}

Communication::~Communication()
{
}

// private:
void Communication::done()
{
  gSocket.close();
  cout << "closed connection" << "\n";
}

bool Communication::selectInput()
{
  fd_set readfds;
  struct timeval tv =
  { 60, 0 };
  FD_ZERO(&readfds);
  FD_SET(gSocket.getFD(), &readfds);

  while (1)
  {
    switch (select(gSocket.getFD() + 1, &readfds, 0, 0, &tv))
    {
    case 1:
      return 1;
    case 0:
      cerr << "(SelectInput) select failed " << strerror(errno) << endl;
      abort();
      return 0;
    default:
      if (errno == EINTR)
        continue;
      cerr << "(SelectInput) select failed " << strerror(errno) << endl;
      abort();
      return 0;
    }
  }
  return 0;
}

// public:

void Communication::initInstance(const std::string& gHost, const int& gPort)
{
  cout << "connecting to TCP " << gHost << ":" << gPort << "\n";
  //cout << "connecting to UDP " << gHost << ":" << gPort << "\n";
  try
  {
    Addr local(INADDR_ANY, INADDR_ANY);
    gSocket.bind(local);
  } catch (BindErr& error)
  {
    cerr << "failed to bind socket with '" << error.what() << "'" << endl;
    gSocket.close();
    exit(EXIT_FAILURE);
  }

  try
  {
    Addr server(gPort, gHost);
    gSocket.connect(server);
  } catch (ConnectErr& error)
  {
    cerr << "connection failed with: '" << error.what() << "'" << endl;
    gSocket.close();
    exit(EXIT_FAILURE);
  }
}

void Communication::putMessage(const std::string& msg)
{
  if (msg.empty())
  {
    return;
  }

  // prefix the message with it's payload length
  unsigned int len = htonl(msg.size());
  std::string prefix((const char*) &len, sizeof(unsigned int));
  std::string str = prefix + msg;
  gSocket.send(str.data(), str.size());
}

bool Communication::getMessage(std::string& msg)
{
  static char buffer[16 * 1024];

  unsigned int bytesRead = 0;
  while (bytesRead < sizeof(unsigned int))
  {
    selectInput();
    int readResult = gSocket.recv(buffer + bytesRead, sizeof(unsigned int) - bytesRead);
    if (readResult < 0)
      continue;
    bytesRead += readResult;
  }

  //cerr << "buffer = |" << string(buffer+1) << "|\n";
  //cerr << "bytesRead = |" << bytesRead << "|\n";
  //cerr << "Size of buffer = |" << sizeof(buffer) << "|\n";
  //cerr << "buffer = |" << buffer << "|\n";
  //cerr << "buffer[5] = |" << buffer[5] << "|\n";
  //printf ("xxx-%s\n", buffer+5);

  // msg is prefixed with it's total length
  int msgLen = ntohl(*(unsigned int*) buffer);

  // cerr << "GM 6 / " << msgLen << " (bytesRead " << bytesRead << ")\n";
  if (sizeof(unsigned int) + msgLen > sizeof(buffer))
  {
    cerr << "message too long; aborting" << endl;
    abort();
  }

  // read remaining message segments
  int msgRead = bytesRead - sizeof(unsigned int);

  //cerr << "msgRead = |" << msgRead << "|\n";

  char *offset = buffer + bytesRead;

  while (msgRead < msgLen)
  {
    if (!selectInput())
    {
      return false;
    }

    int readLen = sizeof(buffer) - msgRead;
    if (readLen > msgLen - msgRead)
      readLen = msgLen - msgRead;

    int readResult = gSocket.recv(offset, readLen);
    if (readResult < 0)
      continue;
    msgRead += readResult;
    offset += readResult;
    //cerr << "msgRead = |" << msgRead << "|\n";
  }

  // zero terminate received data
  (*offset) = 0;

  msg = std::string(buffer + sizeof(unsigned int));

  // DEBUG
  //cout << msg << endl;

  return true;
}

Communication& Communication::getInstance()
{
  if (!instance)
    instance = new Communication;
  return *instance;
}

void Communication::deleteInstance()
{
  if (instance)
  {
    instance->done();
    delete instance;
  }
  instance = 0;
}
