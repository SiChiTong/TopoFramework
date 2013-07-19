/*
 * Log.h
 *
 *  Created on: Jul 19, 2013
 *      Author: sam
 */

#ifndef LOG_H_
#define LOG_H_

#include <sstream>

namespace ime
{
// This is a very simple Log class
class Log : public std::stringstream
{
  public:
    void update();
};
}  // namespace ime

#endif /* LOG_H_ */
