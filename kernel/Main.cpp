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
#include "kernel/Framework.h"

#include "utils/sexpr/sexp.h"
#include "utils/sexpr/sexp_ops.h"
#include "kernel/Config.h"
#include <string.h>

// bool to indicate whether to continue the agent mainloop
static bool gLoop = true;

// SIGINT handler prototype
void signal_callback_handler(int signum)
{
  if (signum == SIGINT)
  {
    std::cout << "signal=" << signum << std::endl;
    gLoop = false;
  }
}

void sexprTest()
{
  const char* msg =
      "(time (now 104.87))(GS (t 0.00) (pm BeforeKickOff))(GYR (n torso)(rt 0.24 -0.05 0.02))(ACC (n torso) (a -0.01 0.05 9.80))(HJ (n hj1)(ax -0.00))(HJ (n hj2) (ax -0.00))(See (G2R (pol 20.11 -18.92 0.84))(G1R (pol 19.53 -13.04 0.90)) (F1R (pol 19.08 4.58 -1.54)) (F2R (pol 22.73 -33.49 -1.47)) (B (pol 10.12 -33.09 -2.94)) (L (pol 15.13 -55.78 -2.03) (pol 8.67 10.24 -3.34)) (L (pol 22.78 -33.20 -1.23)(pol 19.05 4.32 -1.76)) (L (pol 19.08 4.57 -1.55) (pol 1.81 60.14 -17.11)) (L (pol 22.77 -33.23 -1.26) (pol 14.49 -59.60 -1.79)) (L (pol 17.56 -11.77 -1.83) (pol 18.76 -23.38 -1.60)) (L (pol 17.58 -11.67 -1.74) (pol 19.35 -10.53 -1.53)) (L (pol 18.71 -23.82 -1.97)(pol 20.43 -21.36 -1.45)) (L (pol 11.68 -28.23 -2.73) (pol 10.93 -23.90 -2.69)) (L (pol 10.91 -24.22 -2.95) (pol 9.84 -22.59 -3.02))(L (pol 9.84 -22.64 -3.06) (pol 8.81 -25.74 -3.68)) (L (pol 8.83 -25.33 -3.34) (pol 8.35 -32.24 -3.68)) (L (pol 8.35 -32.20 -3.64)(pol 8.69 -39.32 -3.48)) (L (pol 8.68 -39.59 -3.71) (pol 9.63 -43.18 -3.37)) (L (pol 9.65 -42.85 -3.10) (pol 10.75 -42.17 -2.80)) (L (pol 10.75 -42.28 -2.89) (pol 11.61 -38.36 -2.50)) (L (pol 11.62 -38.15 -2.33) (pol 11.94 -33.38 -2.58)) (L (pol 11.94 -33.31 -2.52) (pol 11.70 -28.03 -2.56)))(HJ (n raj1) (ax -0.00))(HJ (n raj2) (ax 0.00))(HJ (n raj3) (ax 0.00))(HJ (n raj4) (ax 0.00))(HJ (n laj1) (ax -0.01))(HJ (n laj2) (ax 0.00))(HJ (n laj3) (ax -0.00))(HJ (n laj4)(ax -0.00))(HJ (n rlj1) (ax 0.01))(HJ (n rlj2) (ax 0.00))(HJ (n rlj3) (ax 0.01))(HJ (n rlj4) (ax -0.00))(HJ (n rlj5) (ax 0.00))(FRP (n rf) (c -0.02 -0.00 -0.02) (f -0.02 -0.17 22.52))(HJ (n rlj6) (ax -0.00))(HJ (n llj1) (ax -0.01))(HJ (n llj2) (ax 0.01))(HJ (n llj3)(ax 0.00))(HJ (n llj4) (ax -0.00))(HJ (n llj5) (ax 0.00))(FRP (n lf)(c 0.02 -0.01 -0.01) (f -0.08 -0.20 22.63))(HJ (n llj6) (ax 0.00))";

  size_t len = strlen(msg);
  sexp_t *sx = 0;
  pcont_t *cc = 0;

  if (len > 0)
  {
    char buf[4096];
    cc = init_continuation((char*) msg);
    sx = (sexp_t *) iparse_sexp((char*) msg, len, cc);
    while (sx != NULL)
    {
      print_sexp(buf, 4096, sx);
      fprintf(stderr, "\n\n%s\n", buf);
      destroy_sexp(sx);
      sexp_cleanup();
      sx = (sexp_t *) iparse_sexp((char*) msg, len, cc);
    }

    destroy_continuation(cc);
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
  graph.graphOutputAllocate();
  while (gLoop)
  {
    graph.graphOutputUpdate();
  }
  graph.graphOutputRelease();
}

int main(int argc, char** argv)
{
  std::cout << "*** starts " << std::endl;

  ime::Graph& graph = ime::Graph::getInstance();
  graph.computeGraph();
  graph.topoSort();
  //std::cout << graph << std::endl;
  topoSignal();
  topoLoop();

  ime::Graph::deleteInstance();

  std::cout << "*** ends   " << std::endl;
  return EXIT_SUCCESS;
}
