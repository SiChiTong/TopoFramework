//============================================================================
// Name        : Main.cpp
// Author      : Sam Abeyruwan
// Version     :
// Copyright   :
// Description : Main class of the TopoFramework. This is an illustration.
//============================================================================

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include "Framework.h"

// bool to indicate whether to continue the agent mainloop
static bool gLoop = true;

// SIGINT handler prototype
extern "C" void handler(int sig)
{
  if (sig == SIGINT)
    gLoop = false;
}

int main(int argc, char** argv)
{
  std::cout << "*** starts " << std::endl;
  ime::Graph& graph = ime::Graph::getInstance();
  graph.computeGraph();
  graph.topoSort();
  std::cout << graph << std::endl;

  while (gLoop)
  {
    graph.callGraphOutput();
  }

  ime::Graph::deleteInstance();
  std::cout << "*** ends   " << std::endl;
  return EXIT_SUCCESS;
}
