/*
 * DrawingModule.h
 *
 *  Created on: Jul 23, 2013
 *      Author: sam
 */

#ifndef DRAWINGMODULE_H_
#define DRAWINGMODULE_H_

#include "kernel/Framework.h"
#include "kernel/Drawing.h"
#include "representations/perception/PlayerInfo.h"

MODULE(DrawingModule)
  REQUIRES(PlayerInfo)
END_MODULE

class DrawingModule : public DrawingModuleBase
{
  public:
    DrawingModule();
    void execute();

  private:
    bool connected;
};

MAKE_MODULE(DrawingModule)

#endif /* DRAWINGMODULE_H_ */
