/*
 * TestOneRepresentation.h
 *
 *  Created on: Jul 14, 2013
 *      Author: Saminda
 */

#ifndef TESTONEREPRESENTATION_H_
#define TESTONEREPRESENTATION_H_

#include "kernel/Template.h"

REPRESENTATION(TestOneRepresentation)
class TestOneRepresentation : public TestOneRepresentationBase
{
  public:
    int a;
    TestOneRepresentation() : a(0) {}
};

#endif /* TESTONEREPRESENTATION_H_ */
