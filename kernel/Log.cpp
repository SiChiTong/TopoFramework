/*
 * Log.cpp
 *
 *  Created on: Jul 19, 2013
 *      Author: sam
 */

#include "Log.h"

void ime::Log::update()
{
  // This is as simple as it gets ...
  std::string str = this->str();
  if (str.length() > 0)
  {
    std::cout << this->str() << std::endl;
    // Clear
    this->str("");
    this->clear();
  }
}

