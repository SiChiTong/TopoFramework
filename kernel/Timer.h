/*
 * Timer.h
 *
 *  Created on: Aug 3, 2013
 *      Author: sam
 *
 *  A simple, high resolution ( 1 micro-second accuracy) timer that work on both Windows and Unix
 *  systems.
 */

#ifndef TIMER_H_
#define TIMER_H_

#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

namespace ime
{
class Timer
{
  public:
    Timer();
    ~Timer();

    void start();
    void stop();
    double getElapsedTime();
    double getElapsedTimeInSec();
    double getElapsedTimeInMilliSec();
    double getElapsedTimeInMicroSec();

  protected:

  private:
    double startTimeInMicroSec;
    double endTimeInMicroSec;
    int stopped;
#ifdef WIN32
    LARGE_INTEGER frequency;                    // ticks per second
    LARGE_INTEGER startCount;//
    LARGE_INTEGER endCount;//
#else
    timeval startCount;                         //
    timeval endCount;                           //
#endif
};
}  // namespace ime

#endif /* TIMER_H_ */
