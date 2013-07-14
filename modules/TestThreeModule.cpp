/*
 * TestThreeModule.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: Saminda
 */

#include "TestThreeModule.h"

void TestThreeModule::init()
{
  std::cout << "TestThreeModule::init" << std::endl;
}

void TestThreeModule::execute()
{
  std::cout << "TestThreeModule::execute" << std::endl;
  std::cout << "theTestOneRepresentation->a=" << theTestOneRepresentation->a << std::endl;
  std::cout << "theTestTwoRepresentation->b=" << theTestTwoRepresentation->b << std::endl;
  std::cout << "theTestThreeRepresentation->c=" << theTestThreeRepresentation->c << std::endl;
}

MAKE_MODULE(TestThreeModule)

