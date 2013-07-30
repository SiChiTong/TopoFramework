/*
 * TestOneModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: Saminda
 */

#ifndef TESTONEMODULE_H_
#define TESTONEMODULE_H_

#include "kernel/Template.h"
#include "representations/test/TestOneRepresentation.h"
#include "representations/test/TestTwoRepresentation.h"
#include "representations/test/TestThreeRepresentation.h"

MODULE(TestOneModule)
  PROVIDES(TestOneRepresentation)
  USES(TestTwoRepresentation)
  USES(TestThreeRepresentation)
END_MODULE
class TestOneModule : public TestOneModuleBase
{
  public:
    void init();
    void execute();
    void update(TestOneRepresentation& theTestOneRepresentation);
};

#endif /* TESTONEMODULE_H_ */
