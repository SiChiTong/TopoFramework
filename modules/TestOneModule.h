/*
 * TestOneModule.h
 *
 *  Created on: Jul 14, 2013
 *      Author: Saminda
 */

#ifndef TESTONEMODULE_H_
#define TESTONEMODULE_H_

#include "Framework.h"
#include "representations/TestOneRepresentation.h"

MODULE(TestOneModule)
PROVIDES(TestOneRepresentation)
END_MODULE
class TestOneModule : public TestOneModuleBase
{
  public:
    void init();
    void execute();
    void update(TestOneRepresentation& theTestOneRepresentation);
};

#endif /* TESTONEMODULE_H_ */
