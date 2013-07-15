/*
 * TestTwoRepresentation.h
 *
 *  Created on: Jul 14, 2013
 *      Author: Saminda
 */

#ifndef TESTTWOREPRESENTATION_H_
#define TESTTWOREPRESENTATION_H_

#include "Framework.h"

REPRESENTATION(TestTwoRepresentation)
class TestTwoRepresentation: public TestTwoRepresentationBase
{
  public:
    int b;
    TestTwoRepresentation() : b(0) {}
};

#endif /* TESTTWOREPRESENTATION_H_ */
