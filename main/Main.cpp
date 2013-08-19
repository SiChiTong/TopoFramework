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
#include <string.h>
#include <cassert>
#include "kernel/Framework.h"
#include "kernel/Config.h"

// bool to indicate whether to continue the agent mainloop
static bool gLoop = true;
static int gLoopCount = 0;

// SIGINT handler prototype
void signal_callback_handler(int signum)
{
  if (signum == SIGINT)
  {
    std::cout << "signal=" << signum << ":" << (++gLoopCount) << std::endl;
    gLoop = false;
    if (gLoopCount > 5)
    {
      std::cout << "Force exit()" << std::endl;
      exit(EXIT_FAILURE);
    }
  }
}

void topoSignal()
{
// Register signal and signal handler
  signal(SIGINT, signal_callback_handler);
}

void topoLoop()
{
  ime::Graph& graph = ime::Graph::getInstance();
  graph.graphOutputAllocateAndUpdate();
  while (gLoop)
  {
    graph.graphOutputUpdate();
    graph.graphDrawing();
  }
  graph.graphOutputRelease();
}

int main(int argc, char** argv)
{
  std::cout << "*** starts " << std::endl;
  // fixMe: TODO: parse the arguments
  ime::Graph& graph = ime::Graph::getInstance();

  graph.computeGraph();
  graph.topoSort();
  std::cout << graph << std::endl;
  topoSignal();
  topoLoop();

  ime::Graph::deleteInstance();

  std::cout << "*** ends   " << std::endl;
  return EXIT_SUCCESS;
}
