/*
 * TestFourModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: Saminda
 */

#ifndef TESTFOURMODULE_H_
#define TESTFOURMODULE_H_

#include "kernel/Template.h"

MODULE(TestFourModule)
END_MODULE
class TestFourModule : public TestFourModuleBase
{
  public:
    void init();
    void execute();
};

#endif /* TESTFOURMODULE_H_ */
