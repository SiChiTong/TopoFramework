#ifndef BLOCKPOSITION_H
#define BLOCKPOSITION_H

#include "kernel/Template.h"
#include "math/Vector2.h"

REPRESENTATION(BlockPosition)
class BlockPosition: public BlockPositionBase
{
  public:

    bool valid; /*< If no opponent found to block, this is false. */
    Vector2<double> pos; /* Go to this pos to block opponent. */
    BlockPosition() :
        valid(false)
    {
    }
};

#endif

