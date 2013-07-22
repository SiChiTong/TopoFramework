/** 
 * @file SimSparkNaoModel.h
 * 
 * SimSparkNaoModel extends RobotModel to define the Nao model used in
 * SimSpark.
 * 
 * @author Andreas <aseek@cs.miami.edu>
 * 
 */

#ifndef SIMSPARKNAOMODEL_H
#define SIMSPARKNAOMODEL_H

class SimSparkNaoModel;
#include "RobotModel.h"

/**
 * @class SimSparkNaoModel
 * RobotModel for the Nao in SimSpark.
 */
class SimSparkNaoModel : public RobotModelBase
{
  public:
  
    /** 
     * This method creates all body parts of the SimSpark Nao.
     * It sets BodyPart *root and int numberOfJoints from the base class.
     */
    void init(); 


};

#endif
