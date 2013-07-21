/*
 * Log.h
 *
 *  Created on: Jul 19, 2013
 *      Author: sam
 */

#ifndef LOG_H_
#define LOG_H_

#include <sstream>
#include <iostream>

namespace ime
{
// This is a very simple Log class
class Log : public std::stringstream
{
  private:
    bool verbose;
  public:
    Log();
    virtual ~Log();
    void update();
    void setVerbose(const bool verbose);
};
}  // namespace ime

#endif /* LOG_H_ */
