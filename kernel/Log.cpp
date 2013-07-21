/*
 * Log.cpp
 *
 *  Created on: Jul 19, 2013
 *      Author: sam
 */

#include "Log.h"

ime::Log::Log() :
    std::stringstream(), verbose(false)
{
}

ime::Log::~Log()
{
}

void ime::Log::update()
{
  // This is as simple as it gets ...
  if (verbose)
  {
    std::string str = this->str();
    if (str.length() > 0)
    {
      std::cout << this->str() << std::endl;
      // Clear
      this->str("");
      this->clear();
    }
  }
}

void ime::Log::setVerbose(const bool verbose)
{
  this->verbose = verbose;
}

