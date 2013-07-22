/** 
 * @file RobotModel.cpp
 * Implementation of the class SimSparkNaoModel.
 * 
 * @author Andreas <aseek@cs.miami.edu>
 * 
 */
#include "SimSparkNaoModel.h"
#include "math/Common.h"

void SimSparkNaoModel::init()
{  
  // These are the values from the SimSpark documentation.
  
  //create Simspark Nao model
  Bodypart *torso = new Bodypart(-1, "torso", NULL, Vector3<double>(0,0,0), 1.2171, Vector3<double>(0,0,0), Vector3<double>(0,0,0), BOX,0.1,0.1,0.18);
  int i = 0;
  //head
  Bodypart *neck = new Bodypart(i++, "neck", torso, Vector3<double>(0,0,0.09), 0.05, Vector3<double>(0,0,0), Vector3<double>(0,0,1), CYLINDER,0.08,0.015,0);
  (void)           new Bodypart(i++, "head", neck, Vector3<double>(0,0,0.065), 0.35, Vector3<double>(0,0,-0.005), Vector3<double>(1,0,0), SPHERE,0.065,0,0);
  //arm left
  Bodypart *shoulderL = new Bodypart(i++, "shoulderL", torso, Vector3<double>(-0.098,0,0.075), 0.07, Vector3<double>(0,0,0), Vector3<double>(1,0,0), SPHERE,0.01,0,0);
  Bodypart *upperArmL = new Bodypart(i++, "upperArmL", shoulderL, Vector3<double>(-0.01,0.02,0), 0.15, Vector3<double>( 0.01,-0.02,0), Vector3<double>(0,0,1), BOX,0.07,0.08,0.06);
  Bodypart *elbowL = new Bodypart(i++, "elbowL", upperArmL, Vector3<double>( 0.01,0.07,0.009), 0.035, Vector3<double>(0,0,0), Vector3<double>(0,1,0), SPHERE,0.01,0,0);
  (void)             new Bodypart(i++, "lowerarmL", elbowL, Vector3<double>(0,0.05,0), 0.2, Vector3<double>(0,-0.05,0), Vector3<double>(0,0,1), BOX,0.05,0.11,0.05);
  //arm right
  Bodypart *shoulderR = new Bodypart(i++, "shoulderR", torso, Vector3<double>( 0.098,0,0.075), 0.07, Vector3<double>(0,0,0), Vector3<double>(1,0,0), SPHERE,0.01,0,0);
  Bodypart *upperArmR = new Bodypart(i++, "upperArmR", shoulderR, Vector3<double>( 0.01,0.02,0), 0.15, Vector3<double>(-0.01,-0.02,0), Vector3<double>(0,0,1), BOX,0.07,0.08,0.06);
  Bodypart *elbowR = new Bodypart(i++, "elbowR", upperArmR, Vector3<double>(-0.01,0.07,0.009), 0.035, Vector3<double>(0,0,0), Vector3<double>(0,1,0), SPHERE,0.01,0,0);
  (void)             new Bodypart(i++, "lowerarmR", elbowR, Vector3<double>(0,0.05,0), 0.2, Vector3<double>(0,-0.05,0), Vector3<double>(0,0,1), BOX,0.05,0.11,0.05);
  //leg left
  Bodypart *hip1L = new Bodypart(i++, "hip1L", torso, Vector3<double>(-0.055,-0.01,-0.115), 0.09, Vector3<double>(0,0,0), Vector3<double>(-0.7071,0,-0.7071), SPHERE,0.01,0,0);
  Bodypart *hip2L = new Bodypart(i++, "hip2L", hip1L, Vector3<double>(0,0,0), 0.125, Vector3<double>(0,0,0), Vector3<double>(0,1,0), SPHERE,0.01,0,0);
  Bodypart *thighL = new Bodypart(i++, "thighL", hip2L, Vector3<double>(0,0.01,-0.04), 0.275, Vector3<double>(0,-0.01,0.04), Vector3<double>(1,0,0), BOX,0.07,0.07,0.14);
  Bodypart *shankL = new Bodypart(i++, "shankL", thighL, Vector3<double>(0,0.005,-0.125), 0.225, Vector3<double>(0,-0.01,0.045), Vector3<double>(1,0,0), BOX,0.08,0.07,0.11);
  Bodypart *ankleL = new Bodypart(i++, "ankleL", shankL, Vector3<double>(0,-0.01,-0.055), 0.125, Vector3<double>(0,0,0), Vector3<double>(1,0,0), SPHERE,0.01,0,0);
  (void)             new Bodypart(i++, "footL", ankleL, Vector3<double>(0,0.03,-0.035), 0.2, Vector3<double>(0,-0.03,0.035), Vector3<double>(0,1,0), BOX,0.08,0.16,0.03);
  //leg right
  Bodypart *hip1R = new Bodypart(i++, "hip1R", torso, Vector3<double>( 0.055,-0.01,-0.115), 0.09, Vector3<double>(0,0,0), Vector3<double>(-0.7071,0, 0.7071), SPHERE,0.01,0,0);
  Bodypart *hip2R = new Bodypart(i++, "hip2R", hip1R, Vector3<double>(0,0,0), 0.125, Vector3<double>(0,0,0), Vector3<double>(0,1,0), SPHERE,0.01,0,0);
  Bodypart *thighR = new Bodypart(i++, "thighR", hip2R, Vector3<double>(0,0.01,-0.04), 0.275, Vector3<double>(0,-0.01,0.04), Vector3<double>(1,0,0), BOX,0.07,0.07,0.14);
  Bodypart *shankR = new Bodypart(i++, "shankR", thighR, Vector3<double>(0,0.005,-0.125), 0.225, Vector3<double>(0,-0.01,0.045), Vector3<double>(1,0,0), BOX,0.08,0.07,0.11);
  Bodypart *ankleR = new Bodypart(i++, "ankleR", shankR, Vector3<double>(0,-0.01,-0.055), 0.125, Vector3<double>(0,0,0), Vector3<double>(1,0,0), SPHERE,0.01,0,0);
  (void)             new Bodypart(i++, "footR", ankleR, Vector3<double>(0,0.03,-0.035), 0.2, Vector3<double>(0,-0.03,0.035), Vector3<double>(0,1,0), BOX,0.08,0.16,0.03);

  root = torso;      
  numberOfJoints = i;

  jointPositions = new Pose3D[numberOfJoints];
  partPositions = new Pose3D[numberOfJoints+1];
    
  //turn the model by 90 degrees
  origin = Pose3D(RotationMatrix(0,0,-pi_2), Vector3<double>(0, 0, 0));
}

