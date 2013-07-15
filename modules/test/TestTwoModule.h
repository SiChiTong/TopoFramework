/*
 * TestTwoModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: Saminda
 */

#ifndef TESTTWOMODULE_H_
#define TESTTWOMODULE_H_

#include "Framework.h"
#include "representations/test/TestOneRepresentation.h"
#include "representations/test/TestTwoRepresentation.h"
#include "representations/test/TestThreeRepresentation.h"

MODULE(TestTwoModule)
REQUIRES(TestOneRepresentation)
PROVIDES(TestTwoRepresentation)
PROVIDES(TestThreeRepresentation)
END_MODULE
class TestTwoModule: public TestTwoModuleBase
{
  public:
    void init();
    void execute();
    void update(TestTwoRepresentation& theTestTwoRepresentation);
    void update(TestThreeRepresentation& theTestThreeRepresentation);
};

#endif /* TESTTWOMODULE_H_ */
