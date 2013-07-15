//============================================================================
// Name        : Main.cpp
// Author      : Sam Abeyruwan
// Version     :
// Copyright   :
// Description : Main class of the TopoFramework. This is an illustration.
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
