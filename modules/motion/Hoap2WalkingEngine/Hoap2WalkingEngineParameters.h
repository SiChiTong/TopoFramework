// ------------------ Robocup3D ------------------
// -- Center for Computing Technologies - (TZI) --
// ----- Virtual Werder 3D - Agent Framework -----
// --------------- Third Revision ----------------

/**
* @file Hoap2WalkingEngineParameters.h implements more abstract walk parameters
* WalkBehaviourParameters includes parameters for behaviour related issues 
* WalkStepParameters should also be used as walk optimization parameters
* @author Cord Niehaus
* @author Andreas Seekircher
*/

#ifndef HOAP2WALKINGENGINEPARAMETERS_H
#define HOAP2WALKINGENGINEPARAMETERS_H

#include "Hoap2WalkingEngineTrajectories.h"
#include "kernel/Config.h"
#include "math/Vector2.h"
#include "math/Vector3.h"
#include "math/Pose2D.h"



class WalkStepParameters
{
public:

  void init();

  enum StepParameters
  {
    /**< Duration of a step in ms. */
    stepDuration,     
    /**< Step Length used during optimization */
    stepLength,             
    /**< Origin X of the position of the left foot. */
    stepOriginX,            
    /**< Origin Y of the position of the left foot. */
    stepOriginY,       
    /**< Origin Z of the position of the left foot. */
    stepOriginZ,       
    /**< Ratio between Rear and Front of step origin */
    stepRearFrontRatio,
    /**< Double support length*/
    doubleSupport,    
    /**< positive step height during air phase */
    stepHeightAir,      
    /**< step height pick begin (0 - 1 of the stepHeightAirPhase)*/
    stepHeightPickAir,    
    /**< step height pick length (0 - 1 of the stepHeightAirPhase) */
    stepHeightLengthAir,    
    /**< negative step height during ground phase */
    stepHeightGround, 
    /**< step height pick begin (0 - 1 of the stepHeightGroundPhase)*/
    stepHeightPickGround,   
    /**< step height pick length (0 - 1 of the stepHeightGroundPhase) */
    stepHeightLengthGround,
    /**< origin of the body shift swing */
    bodyShiftOrigin,    
    /**< difference of scale of body shift from foot position */
    bodyShiftFootDiff,
    /**< pause in sine extrema positions */
    bodyShiftPause,       
    /**< phase shift of body shift curve */
    bodyShiftPhaseShift,    
    /**< origin of the body tilt swing */
    bodyTiltOrigin,   
    /**< scale of body tilt */
    bodyTiltScale,   
    /**< phase shift of body tilt curve */
    bodyTiltPhaseShift,
    /**< ratio between back and forth duration of bodytilt */
    bodyTiltBackForthRatio, 
    /**< arms will move synchrone to the leg on the other side, 
         only the scale is changeable */
    armTiltScale, 
    /**< simply how far should the robot holds his hands from his hips */
    armRollOrigin,          
    numOfStepParameters
  };

  double params[numOfStepParameters]; /**< The step parameter set. */

  /** The method returns the value of a walk parameter with a given number. */
  double getParameterValueByNumber(uint32_t number);
  /** The method sets the value of a walk parameter with a given number */
  void   setParameterValueByNumber(uint32_t number, double value);

private:

  /** values for stepHeight trajectories */
  Vector2<double> stepHeight_maxPoints[9];
  ComplexTrajectory::Shape stepHeight_phaseShape[8];

  /** values for stepX trajectories */
  Vector2<double> stepX_maxPoints[3];
  ComplexTrajectory::Shape stepX_phaseShape[2];

  /** values for bodyShift trajectory */
  Vector2<double> bodyShift_maxPoints[5];
  ComplexTrajectory::Shape bodyShift_phaseShape[4];

  /** values for bodyTilt trajectory */
  Vector2<double> bodyTilt_maxPoints[5];
  ComplexTrajectory::Shape bodyTilt_phaseShape[4];
  
  /** values for rotation */
  Vector2<double> rotation_maxPoints[5];
  ComplexTrajectory::Shape rotation_phaseShape[4];

  /** values for test complex trajectory */
  Vector2<double> testTrajectory_maxPoints[19];
  ComplexTrajectory::Shape testTrajectory_phaseShape[18];

  /** The methodes creates all used trajectories within the init method*/
  void createStepHeightTrajectory();
  void createStepXTrajectory();
  void createBodyShiftTrajectory();
  void createBodyTiltTrajectory();
  void createRotationTrajectory();
  void createTestTrajectory();

public:
  // these must be the last attributes!
  ComplexTrajectory* stepHeight[2],
                   * stepX[2],
                   * bodyShift,
                   * bodyTilt,
                   * rotation[2],
                   * testTrajectory;
 
  /** Standard Constructor */
  WalkStepParameters( void );

  virtual ~WalkStepParameters( void );
  
  ime::Config *config;

};

#endif //HOAP2WALKINGENGINEPARAMETERS_H
