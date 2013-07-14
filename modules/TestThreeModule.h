/*
 * TestThreeModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: Saminda
 */

#ifndef TESTTHREEMODULE_H_
#define TESTTHREEMODULE_H_

#include "Framework.h"
#include "representations/TestOneRepresentation.h"
#include "representations/TestTwoRepresentation.h"
#include "representations/TestThreeRepresentation.h"

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
