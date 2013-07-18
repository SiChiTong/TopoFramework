
#include "Hoap2WalkingEngineParameters.h"
#include "kernel/Config.h"
#include <string>

using namespace std;


// ---- WalkStepParameters --- 
WalkStepParameters::WalkStepParameters( void )
  : config(NULL)
{
  params[stepDuration] = 2000;
  params[stepLength] = 40.0;
  params[stepOriginX] = 0.0;
  params[stepOriginY] = 50.0;
  params[stepOriginZ] = -260.0;
  params[stepRearFrontRatio] = 1.0;
  params[doubleSupport] = 0.0;
  params[stepHeightAir] = 30.0;
  params[stepHeightPickAir] = 0.5;
  params[stepHeightLengthAir] = 0.0;
  params[stepHeightGround] = 0.0;
  params[stepHeightPickGround] = 0.5;
  params[stepHeightLengthGround] = 0.0;
  params[bodyShiftOrigin] = 0.0;
  params[bodyShiftFootDiff] = 0.0;
  params[bodyShiftPause] = 0.0;
  params[bodyShiftPhaseShift] = 0.0;
  params[bodyTiltOrigin] = 0.0;
  params[bodyTiltScale] = -0.004;
  params[bodyTiltPhaseShift] = 0.0;
  params[bodyTiltBackForthRatio] = 0.1;
  params[armTiltScale] = 1.0;
  params[armRollOrigin] = -1.0;
  stepHeight[0] = NULL;
}

WalkStepParameters::~WalkStepParameters( void )
{ 
  if(stepHeight[0] != NULL)
  {
    delete   this->stepHeight[0];
    delete   this->stepHeight[1];
    delete   this->stepX[0];
    delete   this->stepX[1];
    delete   this->bodyShift;
    delete   this->bodyTilt;
    delete   this->testTrajectory;
    delete   this->rotation[0];
    delete   this->rotation[1];
  }
}

/*
WalkStepParameters& WalkStepParameters::operator=(const WalkStepParameters& other)
{
  memcpy((char*) &params[0], (const char*) &other.params[0], 
         (char*) &stepHeight[0] - (char*) &params[0]);
  init();
  return *this;
}
*/ 

void WalkStepParameters::createStepHeightTrajectory()
{
  /** step height trajectories values */
  stepHeight_maxPoints[0] = Vector2<double>(0.0, 0.0);
  stepHeight_maxPoints[1] = Vector2<double>(params[doubleSupport], 0.0);
  stepHeight_maxPoints[2] = Vector2<double>(min(0.5, max(params[doubleSupport], 
							 params[doubleSupport]+
							 ((params[stepHeightPickAir]-
							  (params[stepHeightLengthAir]/2))* 
							  (0.5-params[doubleSupport])))), 
					    params[stepHeightAir]);
  stepHeight_maxPoints[3] = Vector2<double>(min(0.5, max(params[doubleSupport], 
							 params[doubleSupport]+
							 ((params[stepHeightPickAir]+
							  (params[stepHeightLengthAir]/2))* 
							  (0.5-params[doubleSupport])))), 
					    params[stepHeightAir]);
  stepHeight_maxPoints[4] = Vector2<double>(0.5, 0.0);
  stepHeight_maxPoints[5] = Vector2<double>(0.5+params[doubleSupport], 0.0);
  stepHeight_maxPoints[6] = Vector2<double>(min(1.0, max(0.5+params[doubleSupport], 
							 0.5+params[doubleSupport]+
							 (params[stepHeightPickGround]-
							  params[stepHeightLengthGround]/2)* 
							 (0.5-params[doubleSupport]))), 
					    params[stepHeightGround]);
  stepHeight_maxPoints[7] = Vector2<double>(min(1.0, max(0.5+params[doubleSupport], 
							 0.5+params[doubleSupport]+
							 (params[stepHeightPickGround]+
							  params[stepHeightLengthGround]/2)* 
							 (0.5-params[doubleSupport]))), 
					    params[stepHeightGround]);
  stepHeight_maxPoints[8] = Vector2<double>(1.0, 0.0);

  // using arcsin functions 
  stepHeight_phaseShape[0] = (ComplexTrajectory::Shape)0;
  stepHeight_phaseShape[1] = (ComplexTrajectory::Shape)7;
  stepHeight_phaseShape[2] = (ComplexTrajectory::Shape)0;
  stepHeight_phaseShape[3] = (ComplexTrajectory::Shape)6;
  stepHeight_phaseShape[4] = (ComplexTrajectory::Shape)0;
  stepHeight_phaseShape[5] = (ComplexTrajectory::Shape)7;
  stepHeight_phaseShape[6] = (ComplexTrajectory::Shape)0;
  stepHeight_phaseShape[7] = (ComplexTrajectory::Shape)6;

  /** create stepHeight trajectories */
  stepHeight[0] = ComplexTrajectory::create(8, 
					    stepHeight_maxPoints, 
					    stepHeight_phaseShape, 
					    params[doubleSupport]/2);
  stepHeight[1] = ComplexTrajectory::create(8, 
					    stepHeight_maxPoints, 
					    stepHeight_phaseShape, 
					    0.5+(params[doubleSupport]/2));
}

void WalkStepParameters::createStepXTrajectory()
{
  /** stepX trajectories values */
  stepX_maxPoints[0] = Vector2<double>(0.0, -1.0);
  stepX_maxPoints[1] = Vector2<double>(0.5 - params[doubleSupport], 1.0);
  stepX_maxPoints[2] = Vector2<double>(1.0, -1.0);
                        
  stepX_phaseShape[0] = (ComplexTrajectory::Shape)0;
  stepX_phaseShape[1] = (ComplexTrajectory::Shape)0;

  /** create stepX trajectories */
  stepX[0] = ComplexTrajectory::create(2, 
				       stepX_maxPoints, 
				       stepX_phaseShape, 
				       -params[doubleSupport]/2);
  stepX[1] = ComplexTrajectory::create(2, 
				       stepX_maxPoints, 
				       stepX_phaseShape, 
				       0.5-(params[doubleSupport]/2));
}

void WalkStepParameters::createBodyShiftTrajectory()
{
  /** body shift trajectory values */
  bodyShift_maxPoints[0] = Vector2<double>(0.0, 1.0);
  bodyShift_maxPoints[1] = Vector2<double>(params[bodyShiftPause], 1.0);
  bodyShift_maxPoints[2] = Vector2<double>(0.5, -1.0);
  bodyShift_maxPoints[3] = Vector2<double>(0.5+params[bodyShiftPause], -1.0);
  bodyShift_maxPoints[4] = Vector2<double>(1.0, 1.0);

  bodyShift_phaseShape[0] = (ComplexTrajectory::Shape)0; 
  bodyShift_phaseShape[1] = (ComplexTrajectory::Shape)8; 
  bodyShift_phaseShape[2] = (ComplexTrajectory::Shape)0; 
  bodyShift_phaseShape[3] = (ComplexTrajectory::Shape)8; 
 
  /** create bodyShift trajectory */
  bodyShift = ComplexTrajectory::create(4, 
                                        bodyShift_maxPoints, 
                                        bodyShift_phaseShape, 
                                        params[bodyShiftPhaseShift] + 
                                        params[bodyShiftPause]/2 + 0.25);
}

void WalkStepParameters::createBodyTiltTrajectory()
{
  /** body tilt trajectory values */
  bodyTilt_maxPoints[0] = Vector2<double>(0.0, 
					  params[bodyTiltScale]);
  bodyTilt_maxPoints[1] = Vector2<double>(0.25 - 
					  params[bodyTiltBackForthRatio], 
					  -params[bodyTiltScale]);
  bodyTilt_maxPoints[2] = Vector2<double>(0.5, 
					  params[bodyTiltScale]);
  bodyTilt_maxPoints[3] = Vector2<double>(0.75 - 
					  params[bodyTiltBackForthRatio], 
					  -params[bodyTiltScale]);
  bodyTilt_maxPoints[4] = Vector2<double>(1.0, 
					  params[bodyTiltScale]);
  
  bodyTilt_phaseShape[0] = (ComplexTrajectory::Shape)3; 
  bodyTilt_phaseShape[1] = (ComplexTrajectory::Shape)3; 
  bodyTilt_phaseShape[2] = (ComplexTrajectory::Shape)3; 
  bodyTilt_phaseShape[3] = (ComplexTrajectory::Shape)3; 
  
  /** create bodyTilt trajectory */
  bodyTilt = ComplexTrajectory::create(4, 
                                       bodyTilt_maxPoints, 
                                       bodyTilt_phaseShape, 
                                       params[bodyTiltPhaseShift] + 
                                       params[doubleSupport]/2);
}

void WalkStepParameters::createRotationTrajectory()
{
  /** rotation trajectory values */
  rotation_maxPoints[0] = Vector2<double>(0.0, 1.0);
  rotation_maxPoints[1] = Vector2<double>(params[doubleSupport], 1.0);
  rotation_maxPoints[2] = Vector2<double>(0.5, -1.0);
  rotation_maxPoints[3] = Vector2<double>(0.5 + params[doubleSupport], -1.0);
  rotation_maxPoints[4] = Vector2<double>(1.0, 1.0);

  rotation_phaseShape[0] = (ComplexTrajectory::Shape)0; 
  rotation_phaseShape[1] = (ComplexTrajectory::Shape)0; 
  rotation_phaseShape[2] = (ComplexTrajectory::Shape)0; 
  rotation_phaseShape[3] = (ComplexTrajectory::Shape)0; 
 
  /** create rotation trajectory */
  rotation[0] = ComplexTrajectory::create(4, 
					  rotation_maxPoints, 
					  rotation_phaseShape, 
					  params[doubleSupport]/2);
  rotation[1] = ComplexTrajectory::create(4, 
					  rotation_maxPoints, 
					  rotation_phaseShape, 
					  params[doubleSupport]/2 + 0.5);
}

void WalkStepParameters::createTestTrajectory()
{
  // test complex trajectory values
  testTrajectory_maxPoints[0]  = Vector2<double>(      0.0,   0.0);
  testTrajectory_maxPoints[1]  = Vector2<double>( 1.0/18.0,   2.0);
  testTrajectory_maxPoints[2]  = Vector2<double>( 2.0/18.0,  -2.0);
  testTrajectory_maxPoints[3]  = Vector2<double>( 3.0/18.0,   3.0);
  testTrajectory_maxPoints[4]  = Vector2<double>( 4.0/18.0,  -3.0);
  testTrajectory_maxPoints[5]  = Vector2<double>( 5.0/18.0,   2.0);
  testTrajectory_maxPoints[6]  = Vector2<double>( 6.0/18.0,  -2.0);
  testTrajectory_maxPoints[7]  = Vector2<double>( 7.0/18.0,   3.0);
  testTrajectory_maxPoints[8]  = Vector2<double>( 8.0/18.0,  -3.0);
  testTrajectory_maxPoints[9]  = Vector2<double>( 9.0/18.0,   2.0);
  testTrajectory_maxPoints[10] = Vector2<double>(10.0/18.0,  -2.0);
  testTrajectory_maxPoints[11] = Vector2<double>(11.0/18.0,   3.0);
  testTrajectory_maxPoints[12] = Vector2<double>(12.0/18.0,  -3.0);
  testTrajectory_maxPoints[13] = Vector2<double>(13.0/18.0,   2.0);
  testTrajectory_maxPoints[14] = Vector2<double>(14.0/18.0,  -2.0);
  testTrajectory_maxPoints[15] = Vector2<double>(15.0/18.0,   3.0);
  testTrajectory_maxPoints[16] = Vector2<double>(16.0/18.0,  -3.0);
  testTrajectory_maxPoints[17] = Vector2<double>(17.0/18.0,   2.0);
  testTrajectory_maxPoints[18] = Vector2<double>(      1.0,   0.0);

  testTrajectory_phaseShape[0] = (ComplexTrajectory::Shape)0; 
  testTrajectory_phaseShape[1] = (ComplexTrajectory::Shape)0; 
  testTrajectory_phaseShape[2] = (ComplexTrajectory::Shape)1; 
  testTrajectory_phaseShape[3] = (ComplexTrajectory::Shape)1; 
  testTrajectory_phaseShape[4] = (ComplexTrajectory::Shape)2; 
  testTrajectory_phaseShape[5] = (ComplexTrajectory::Shape)2; 
  testTrajectory_phaseShape[6] = (ComplexTrajectory::Shape)3; 
  testTrajectory_phaseShape[7] = (ComplexTrajectory::Shape)3; 
  testTrajectory_phaseShape[8] = (ComplexTrajectory::Shape)4; 
  testTrajectory_phaseShape[9] = (ComplexTrajectory::Shape)4; 
  testTrajectory_phaseShape[10] = (ComplexTrajectory::Shape)5; 
  testTrajectory_phaseShape[11] = (ComplexTrajectory::Shape)5; 
  testTrajectory_phaseShape[12] = (ComplexTrajectory::Shape)6; 
  testTrajectory_phaseShape[13] = (ComplexTrajectory::Shape)6; 
  testTrajectory_phaseShape[14] = (ComplexTrajectory::Shape)7; 
  testTrajectory_phaseShape[15] = (ComplexTrajectory::Shape)7; 
  testTrajectory_phaseShape[16] = (ComplexTrajectory::Shape)8; 
  testTrajectory_phaseShape[17] = (ComplexTrajectory::Shape)8; 
  
  // create test complex trajectory 
  testTrajectory = ComplexTrajectory::create(18, 
                                             testTrajectory_maxPoints,  
                                             testTrajectory_phaseShape, 
                                             0.0);
}

void WalkStepParameters::init( void )
{
  // Read step parameters values from configuration
  this->params[stepDuration] = config->getValue("STEP_DURATION", 320.0);
  this->params[stepLength] = config->getValue("STEP_LENGTH",1500.0);
  this->params[stepOriginX] = config->getValue("STEP_ORIGIN_X",0.0);            
  this->params[stepOriginY] = config->getValue("STEP_ORIGIN_Y",45.0);      
  this->params[stepOriginZ] = config->getValue("STEP_ORIGIN_Z",-222.807);     
  this->params[stepRearFrontRatio] = config->getValue("STEP_REAR_FRONT_RATIO",0.898353);
  this->params[doubleSupport] = config->getValue("DOUBLE_SUPPORT",0.0);
  this->params[stepHeightAir] = config->getValue("STEP_HEIGHT_AIR",25.0);   
  this->params[stepHeightPickAir] = config->getValue("STEP_HEIGHT_PICK_AIR",0.5);
  this->params[stepHeightLengthAir] = config->getValue("STEP_HEIGHT_LENGTH_AIR",0.372755);    
  this->params[stepHeightGround] = config->getValue("STEP_HEIGHT_GROUND",0.5);
  this->params[stepHeightPickGround] = config->getValue("STEP_HEIGHT_PICK_GROUND",0.0); 
  this->params[stepHeightLengthGround] = config->getValue("STEP_HEIGHT_LENGTH_GROUND",0.0);
  this->params[bodyShiftOrigin] = config->getValue("BODY_SHIFT_ORIGIN",0.0); 
  this->params[bodyShiftFootDiff] = config->getValue("BODY_SHIFT_FOOT_DIFF",-39.6322);
  this->params[bodyShiftPause] = config->getValue("BODY_SHIFT_PAUSE",0.27474);   
  this->params[bodyShiftPhaseShift] = config->getValue("BODY_SHIFT_PHASE_SHIFT",0.0652143);  
  this->params[bodyTiltOrigin] = config->getValue("BODY_TILT_ORIGIN",0.03);
  this->params[bodyTiltScale] = config->getValue("BODY_TILT_SCALE",0.0);
  this->params[bodyTiltPhaseShift] = config->getValue("BODY_TILT_PHASE_SHIFT",0.0);
  this->params[bodyTiltBackForthRatio] = config->getValue("BODY_TILT_BACK_FORTH_RATIO",0.0);
  this->params[armTiltScale] = config->getValue("ARM_TILT_SCALE",3.88838);
  this->params[armRollOrigin] = config->getValue("ARM_ROLL_ORIGIN",-0.2);

  // Do internal trajectories initialization
  createStepHeightTrajectory();
  createStepXTrajectory();
  createBodyShiftTrajectory();
  createBodyTiltTrajectory();
  createRotationTrajectory();
  createTestTrajectory();
}



double WalkStepParameters::getParameterValueByNumber(uint32_t number)
{
  return params[number];
}

void WalkStepParameters::setParameterValueByNumber(uint32_t number, 
						   double value)
{
  params[number] = value;
}

