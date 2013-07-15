/*
 * TestThreeRepresentation.h
 *
 *  Created on: Jul 14, 2013
 *      Author: Saminda
 */

#ifndef TESTTHREEREPRESENTATION_H_
#define TESTTHREEREPRESENTATION_H_

#include "Framework.h"

REPRESENTATION(TestThreeRepresentation)
class TestThreeRepresentation: public TestThreeRepresentationBase
{
  public:
    int c;
    TestThreeRepresentation() : c(0) {}
};

#endif /* TESTTHREEREPRESENTATION_H_ */
