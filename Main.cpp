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
