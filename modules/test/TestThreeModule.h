/*
 * TestThreeModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: Saminda
 */

#ifndef TESTTHREEMODULE_H_
#define TESTTHREEMODULE_H_

#include "kernel/Framework.h"
#include "representations/test/TestOneRepresentation.h"
#include "representations/test/TestTwoRepresentation.h"
#include "representations/test/TestThreeRepresentation.h"

MODULE(TestThreeModule)
REQUIRES(TestOneRepresentation)
REQUIRES(TestTwoRepresentation)
REQUIRES(TestThreeRepresentation)
END_MODULE
class TestThreeModule : public TestThreeModuleBase
{
  public:
    void init();
    void execute();
};

#endif /* TESTTHREEMODULE_H_ */
