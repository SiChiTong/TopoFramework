//============================================================================
// Name        : PhD.cpp
// Author      : Sam Abeyruwan
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C, Ansi-style
//============================================================================

#include <stdio.h>
#include <stdlib.h>

#include "Framework.h"

REPRESENTATION(TestOneRepresentation)
class TestOneRepresentation : public TestOneRepresentationBase
{
  public:
    int a;
    TestOneRepresentation() : a(0) {}
};

REPRESENTATION(TestTwoRepresentation)
class TestTwoRepresentation: public TestTwoRepresentationBase
{
  public:
    int b;
    TestTwoRepresentation() : b(0) {}
};

REPRESENTATION(TestThreeRepresentation)
class TestThreeRepresentation: public TestThreeRepresentationBase
{
  public:
    int c;
    TestThreeRepresentation() : c(0) {}
};



MODULE(TestOneModule)
PROVIDES(TestOneRepresentation)
END_MODULE
class TestOneModule : public TestOneModuleBase
{
  public:
    void init()
    {
      std::cout << "TestOneModule::init" << std::endl;
    }

    void execute()
    {
      std::cout << "TestOneModule::execute" << std::endl;
    }

    void update(TestOneRepresentation& theTestOneRepresentation)
    {
      std::cout << "TestOneModule::TestOneRepresentation" << std::endl;
      theTestOneRepresentation.a = 10;
    }
};

MODULE(TestTwoModule)
REQUIRES(TestOneRepresentation)
PROVIDES(TestTwoRepresentation)
PROVIDES(TestThreeRepresentation)
END_MODULE
class TestTwoModule : public TestTwoModuleBase
{
  public:

    void init()
    {
      std::cout << "TestTwoModule::init" << std::endl;
    }

    void execute()
    {
      std::cout << "TestTwoModule::execute" << std::endl;
      std::cout << "theTestOneRepresentation->a=" << theTestOneRepresentation->a << std::endl;
    }

    void update(TestTwoRepresentation& theTestTwoRepresentation)
    {
      std::cout << "TestTwoModule::TestTwoRepresentation" << std::endl;
      theTestTwoRepresentation.b = 20;
    }
    void update(TestThreeRepresentation& theTestThreeRepresentation)
    {
      std::cout << "TestTwoModule::TestThreeRepresentation" << std::endl;
      theTestThreeRepresentation.c = 30;
    }
};

MODULE(TestThreeModule)
REQUIRES(TestOneRepresentation)
REQUIRES(TestTwoRepresentation)
REQUIRES(TestThreeRepresentation)
END_MODULE
class TestThreeModule : public TestThreeModuleBase
{
  public:
    void init()
    {
      std::cout << "TestThreeModule::init" << std::endl;
    }

    void execute()
    {
      std::cout << "TestThreeModule::execute" << std::endl;
      std::cout << "theTestOneRepresentation->a=" << theTestOneRepresentation->a << std::endl;
      std::cout << "theTestTwoRepresentation->b=" << theTestTwoRepresentation->b << std::endl;
      std::cout << "theTestThreeRepresentation->c=" << theTestThreeRepresentation->c << std::endl;
    }

};

MODULE(TestFourModule)
END_MODULE
class TestFourModule : public TestFourModuleBase
{
  public:
    void init()
    {
      std::cout << "TestFourModule::init" << std::endl;
    }

    void execute()
    {
      std::cout << "TestFourModule::execute" << std::endl;
    }

};


MAKE_MODULE(TestOneModule)
MAKE_MODULE(TestTwoModule)
MAKE_MODULE(TestThreeModule)
MAKE_MODULE(TestFourModule)




int main(void)
{
  std::cout << "*** starts " << std::endl;
  ime::Graph& graph = ime::Graph::getInstance();
  graph.computeGraph();
  graph.topoSort();
  graph.callGraphOutput();
  std::cout << graph << std::endl;
  ime::Graph::deleteInstance();
  std::cout << "*** ends   " << std::endl;
  return EXIT_SUCCESS;
}
