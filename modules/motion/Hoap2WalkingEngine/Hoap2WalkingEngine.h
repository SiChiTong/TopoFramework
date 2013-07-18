#ifndef WALKINGENGINE_H
#define WALKINGENGINE_H

#include "kernel/Framework.h"
#include "math/Vector3.h"

#include "representations/perception/FrameInfo.h"
#include "representations/modeling/UprightVec.h"
#include "representations/modeling/Odometry.h"
#include "representations/motion/MotionRequest.h"
#include "representations/motion/WalkingEngineOutput.h"
#include "Hoap2WalkingEngineParameters.h"

#include "representations/motion/SpecialActionsOutput.h"
#include "representations/motion/SpecialMotionsOutput.h"


MODULE(Hoap2WalkingEngine)
  REQUIRES(SpecialActionsOutput)  //Only needed to see if a special action is
  REQUIRES(SpecialMotionsOutput)  //still active. Then currentSpeed is set to 0.
                                  //If the speed is measured using sensors
                                  //this can be deleted.
  REQUIRES(FrameInfo)
  REQUIRES(UprightVec)
  REQUIRES(Odometry)
  REQUIRES(MotionRequest)
  PROVIDES(WalkingEngineOutput)
END_MODULE


class Hoap2WalkingEngine : public Hoap2WalkingEngineBase
{
  public:
    
    Hoap2WalkingEngine();
    ~Hoap2WalkingEngine();
    
    void init();
    void execute();
    void update(WalkingEngineOutput& theWalkingEngineOutput);



    /**
    * The function determines whether it is allowed to switch to another motion.
    * @return Is it the right moment to exit the walking engine? 
    *         True is always returned if currently no walk is performed.
    */
    bool isLeavingPossible( void );

  private:

    WalkStepParameters* stepParameters;      
    double jointAngles[NUM_JOINT_ID];

    Pose2D odometryData;
    long odometryTimestamp;

   /**
    * The method calculates the joint angles for a certain leg.
    * @param position The desired position of the foot sole.
    * @param bodyTilt The tilt angle of the body.
    * @param bodyShift The side shift of the body center in mm.
    * @param joint The index of the first joint of this leg in the 
    *        class JointAngles.
    */
    void calcLegJoints(const Vector3<double>& position, 
           double bodyTilt, 
           double bodyShift,  
                       JOINT_ID joint);
    
    /**
    * The method calculates the joint angles for a certain arm.
    * @param side which arm side need to the calculated
    * @param joint first arm joint to set 
    */
    void calcArmJoints(uint32_t side, JOINT_ID joint);

    /**
    * The method clips the step sizes to their maximum limits.
    * @param xSize The steps size in x direction.
    * @param ySize The step size in y direction.
    * @param rotSize The step size in x direction for rotation.
    */
    void limitStepSizes(double& xSize, double& ySize, double& rotSize);

    /**
     * This method corrects the step length in x-direction 
     * during rotation, in way that the outer foot moves more 
     * than the inner foot.
     * @param leftXSize the steps size of the left foot in x direction.
     * @param rightXSize the steps size of the right foot in x direction.
     */
    void correctStepSizeForRotation(double& leftXSize, double& rightXSize);


    /** The current walking speed. */
    Pose2D currentSpeed;

    /** The current position in walk phase. */
    double phase; 
    double lastPhase;
    
    /** The current foot positions. */
    Vector3<double> footPositions[2]; 

    /** The current foot rotation angles */
    double footRotationAngles[2];          
    
    /** The current step sizes for rotation */
    double xSizeRotLeft, xSizeRotRight;
    
    // current bodyShift and bodyTilt for calculation
    double bodyShift, bodyTilt;
   
    /** The previous motion type performed. */
    MotionRequest::Motions lastMotionType; 

    /** The last time */ 
    uint32_t lastTime;

    /** The time when the motion type was switched to walking. */ 
    uint32_t enterTime;

    /** The time when the first motion cycle began after 
        motion type was switched to walking. */  
    uint32_t startTime;

    /** The last time when the robot entered a state in which 
        leaving walking would be possible. */ 
    uint32_t exitTime;
    
    
    /**< The motion request that is currently executed. */
    Pose2D executedWalkRequest; 
    /**< If true, the motion is stable, leading to a valid camera matrix. */
    bool motionIsStable; 
    /**< The position inside the step of the walking engine slides from 0 to 1 during one step. */
    double positionInWalkCycle; 


};


#endif

