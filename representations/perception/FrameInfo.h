#ifndef FRAMEINFO_H
#define FRAMEINFO_H

#include "kernel/Template.h"

REPRESENTATION(FrameInfo)

class FrameInfo: public FrameInfoBase
{
  public:

    FrameInfo() :
        time(-1), time_ms(0), frameNumber(0), cycleTime(0.01)
    {
    }

    double time; /**< Time since agent was started in seconds. */
    uint32_t time_ms; /**< Time since agent was started in milliseconds. */
    uint32_t frameNumber; /**< Frame counter. */

    double cycleTime; /**< Length of one cycle in seconds. */ //TODO set cycleTime somewhere else?

    /**
     * The method returns the time difference between a given time stamp and the
     * current frame time.
     * @param timeStamp A time stamp, usually in the past.
     * @return The number of ms passed since the given time stamp.
     */
    int getTimeSince(unsigned timeStamp) const
    {
      return int(time_ms - timeStamp);
    }
};

#endif

