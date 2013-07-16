/*
 * TestTwoModule.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: Saminda
 */

#include "TestTwoModule.h"

void TestTwoModule::init()
{
  //std::cout << "TestTwoModule::init" << std::endl;
}

void TestTwoModule::execute()
{
  //std::cout << "TestTwoModule::execute" << std::endl;
  //std::cout << "theTestOneRepresentation->a=" << theTestOneRepresentation->a << std::endl;
}

void TestTwoModule::update(TestTwoRepresentation& theTestTwoRepresentation)
{
  //std::cout << "TestTwoModule::TestTwoRepresentation" << std::endl;
  theTestTwoRepresentation.b = 20;
}
void TestTwoModule::update(TestThreeRepresentation& theTestThreeRepresentation)
{
  //std::cout << "TestTwoModule::TestThreeRepresentation" << std::endl;
  theTestThreeRepresentation.c = 30;
}

MAKE_MODULE(TestTwoModule)

