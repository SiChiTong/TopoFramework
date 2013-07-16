/*
 * TestOneModule.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: Saminda
 */

#include "TestOneModule.h"

void TestOneModule::init()
{
  //std::cout << "TestOneModule::init" << std::endl;
}

void TestOneModule::execute()
{
  //std::cout << "TestOneModule::execute" << std::endl;
}

void TestOneModule::update(TestOneRepresentation& theTestOneRepresentation)
{
  //std::cout << "TestOneModule::TestOneRepresentation" << std::endl;
  theTestOneRepresentation.a = 10;
}

MAKE_MODULE(TestOneModule)

