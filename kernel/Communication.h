/*
 * Communication.h
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

// This class communicates with a given server using a TCP socket. This includes
// sending and receiving messages. In addition to this this also provides a singleton class
// send and receive messages any point in the execution cycle.

#include <iostream>
#include "utils/net/TCPSocket.hpp"
#include "utils/net/Exception.hpp"

namespace ime
{
class Communication
{
  protected:
    rcss::net::TCPSocket gSocket;
    bool connected;

  public:
    static Communication& getInstance();
    static void deleteInstance();
    void putMessage(const std::string& msg);
    bool getMessage(std::string& msg);
    void initInstance(const std::string& gHost, const int& gPort);

  private:
    void done();
    bool selectInput();

  private:
    Communication();
    ~Communication();
    Communication(Communication const&);
    Communication& operator=(Communication const&);
    static Communication* instance;
};
}  // namespace ime

#endif /* COMMUNICATION_H_ */
