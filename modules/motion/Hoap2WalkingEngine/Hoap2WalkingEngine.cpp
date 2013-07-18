#include "Hoap2WalkingEngine.h"

MAKE_MODULE(Hoap2WalkingEngine)

using namespace std;



void Hoap2WalkingEngine::update(WalkingEngineOutput& theWalkingEngineOutput)
{
  theWalkingEngineOutput.active =
      (theMotionRequest->motion == MotionRequest::WALK);
  for(int i=2; i<NUM_JOINT_ID; i++)
    theWalkingEngineOutput.values[i].angle = jointAngles[i];
    
  //odometry (only translation)
  theWalkingEngineOutput.odometry = odometryData;
  theWalkingEngineOutput.odometryTimestamp = odometryTimestamp;
  
  theWalkingEngineOutput.currentSpeed = currentSpeed;
}



Hoap2WalkingEngine::Hoap2WalkingEngine()
: stepParameters(NULL), phase(0),
  lastMotionType(MotionRequest::NUM_MOTIONS),
  exitTime(0)
{
}

Hoap2WalkingEngine::~Hoap2WalkingEngine()
{
  if(stepParameters != NULL)
    delete stepParameters;
}

void Hoap2WalkingEngine::init()
{
  stepParameters = new WalkStepParameters();
  stepParameters->config = &config;
  stepParameters->init();
  
  // init footPositions ---> 0 = left and 1 = right
  this->footPositions[0].x = this->footPositions[1].x = 
    stepParameters->params[WalkStepParameters::stepOriginX];
  this->footPositions[0].y = this->footPositions[1].y = 
    stepParameters->params[WalkStepParameters::stepOriginY];
  this->footPositions[0].z = this->footPositions[1].z = 
    stepParameters->params[WalkStepParameters::stepOriginZ];
  this->footPositions[0].y *= -1;

  // init walk cycle phase
  this->phase = 0.0;

  //this->lastTime = this->extract_time->get_time_ms();
}
  
void Hoap2WalkingEngine::execute()
{
  if(theMotionRequest->motion == MotionRequest::WALK)
  {
    //log << "walk request, walking active" << std::endl;
    // entering the walking engine for the first time
    if((lastMotionType != MotionRequest::WALK) ||
       ((theFrameInfo->time_ms - this->lastTime) > 100))
    {
      //log << "reset times and speed" << std::endl;
      // the phase starts at 0.0
      this->phase     = 0.0;
      this->lastPhase = 0.0;
      // the enter time is the current system time
      this->enterTime = theFrameInfo->time_ms;
      this->startTime = this->enterTime;
      this->lastTime  = this->enterTime;
      this->exitTime  = 0;
      currentSpeed    = Pose2D(0,0,0);
      lastMotionType  = MotionRequest::WALK;
    }
    

    // get phase 
    double stepDuration = stepParameters->params[WalkStepParameters::stepDuration];
    uint32_t   diffTime = (theFrameInfo->time_ms - this->startTime);
    this->phase = (double)(diffTime % (uint32_t)stepDuration) / stepDuration;
    
    this->lastPhase = this->phase;
    //this->lastTime  = extract_time->get_time_ms(); 

    /* debug
    if(this->lastPhase > this->phase)
    {
      D_INFO(extract_time->get_time_ms() - this->lastTime)
	this->lastTime = extract_time->get_time_ms();
    }   
    this->lastPhase = this->phase; 
    */
    
    // target speed from walkRequest
    Pose2D targetSpeed = theMotionRequest->walkRequest;
    
    //cout << targetSpeed.translation.x << " " << targetSpeed.translation.y << " " << targetSpeed.rotation << endl;
    // limit the step values
    limitStepSizes(targetSpeed.translation.x, 
		   targetSpeed.translation.y, 
		   targetSpeed.rotation);
      
    // odometry scale to match the target speed to compensate the difference between 
    //targetSpeed.translation.x /= behaviorParameters->odometryScale.translation.x;
    //targetSpeed.translation.y /= behaviorParameters->odometryScale.translation.y;
    //targetSpeed.rotation      /= behaviorParameters->odometryScale.rotation;

    // calculate maximum speed changes
    float time_diff = min(100.0f, (float)theFrameInfo->time_ms - (float)this->lastTime); 
    double xMax = config.getValue("maxSpeedChangeX", 800.0) * 
                  time_diff/1000.0f, //dimensions->motionCycleTime,
           yMax = config.getValue("maxSpeedChangeY", 800.0) * 
                  time_diff/1000.0f, //dimensions->motionCycleTime,
           rMax = config.getValue("maxSpeedChangeRotation", 10.0) *
                  time_diff/1000.0f; //dimensions->motionCycleTime;
    
    this->lastTime = theFrameInfo->time_ms;

    // limit the step value changes
    currentSpeed.translation.x += min(xMax, 
				      max(-xMax, 
					  (targetSpeed.translation.x - 
					   currentSpeed.translation.x)));

    currentSpeed.translation.y += min(yMax, 
				      max(-yMax, 
					  (targetSpeed.translation.y - 
					   currentSpeed.translation.y)));
    currentSpeed.rotation      += min(rMax, 
				      max(-rMax, 
					  (targetSpeed.rotation - 
					   currentSpeed.rotation)));
    
    //cout << " curr x: " << currentSpeed.translation.x 
    //	 << " curr y: " << currentSpeed.translation.y  
    //     << " curr r: " << currentSpeed.rotation << endl;

    // calculate speedRatio to match step length with target speed by behavior
    double speedRatio = stepParameters->params[WalkStepParameters::stepDuration]/4000.0;
    
    // original : set the size values
    double xSize    = currentSpeed.translation.x * speedRatio; 
    double ySize    = currentSpeed.translation.y * speedRatio;

    // for optimization : set the size values
    //double xSize    = currentSpeed.translation.x; 
    //double ySize    = currentSpeed.translation.y;
    // should rotation stay the same ??? rotation in rad/sec ???
    double rotation = currentSpeed.rotation;          

    // bodyShift calculation
    double bodyShiftScale  = stepParameters->bodyShift->getPosition(phase);
           bodyShiftScale *= (((bodyShiftScale > 0) ? fabs(footPositions[0].y) : fabs(footPositions[1].y)) +
			      stepParameters->params[WalkStepParameters::bodyShiftFootDiff]);
    bodyShift  = bodyShiftScale + 
                 stepParameters->params[WalkStepParameters::bodyShiftOrigin];

    //D_INFO( "[vw3d] we bs traj: phase " << phase << "- bS " << stepParameters->bodyShift->getPosition(phase)) 

    // bodyTilt calculation
    bodyTilt = stepParameters->bodyTilt->getPosition(phase);
    // current linear scale value for bodyTilt according to speed in x-direction
    double bodyTiltOrigin = stepParameters->params[WalkStepParameters::bodyTiltOrigin];
      //* ((xSize == 0 || ySize != 0) ? 1 : sgn(xSize)); // testing walk backward with invers bodytilt

    //cout << "x " << xSize << " sign von x " << sgn(xSize) << endl;
    bodyTilt = bodyTilt * (currentSpeed.translation.x) +
      bodyTiltOrigin;
      //(acceleration * -0.02);
    
    // bodyTilt correction with up_direction-values
    float upright_direction_y = -atan2(theUprightVec->vec.x, 
                                       theUprightVec->vec.z);
    //D_INFO("upright_direction_y = " << upright_direction_y);
    //D_INFO("body_tilt = " << bodyTilt*(180/M_PI));
    float error = max(-10.0f, min(10.0f, upright_direction_y)) - bodyTilt*(180/M_PI);
    if(error > 180)
      error -= 360;
    if(error < -180)
      error += 360;
      
    //D_INFO("bodytilterror " << error);

    /*
    static float max_error = 0.0;
    if(xSize > 5.0f){
      if(fabs(error) > max_error){
	max_error = fabs(error);
	//cout << max_error << endl;
      }
    }
    */
    static double p = 0;//-0.4; //-0.95;
    /*
    static uint32_t counter = 0.0;
    static double all_errors = 0.0;
    all_errors += fabs(error);
    
    if( counter == 500 ){
      D_INFO("all_errors = " << all_errors << " with p = " << p);
      counter = 0;
      all_errors = 0;
      p -= 0.25;
    }
    counter++;
    */
    //cout << bodyTilt*(180.0f/M_PI) << " "; 
    float p_value = p * (M_PI/180.0f);
    float correction_value = error * p_value;
    bodyTilt += correction_value;
    
    //D_INFO(bodyTilt*(180.0f/M_PI) << " " << upright_direction_y << " " << error << " " << correction_value*(180/M_PI));
    
    

    // in simple version simply set to zero 
    footRotationAngles[0] = footRotationAngles[1] = 0; 

    // stepRearFront Correction
    xSize = (stepParameters->stepX[0]->getPosition(phase) >= 0) ?
      (2*xSize / (1.0 + 1.0/stepParameters->params[WalkStepParameters::stepRearFrontRatio])) :
      (2*xSize / (1.0 +     stepParameters->params[WalkStepParameters::stepRearFrontRatio]));
    
      
    xSizeRotLeft = xSizeRotRight = xSize;

    // calculate 
    if(rotation != 0.0)
    {
      // two cases : 
      // first  - rotate while walking with speed in x-direction
      // second - rotate while walking without speed in x-direction,
      //          maybe walk sidewards or stand
      // maybe use rotation trajectory to match behavior in double support phases better
      if(xSize != 0)
      {
        footRotationAngles[0] = stepParameters->stepX[0]->getPosition(phase) * rotation;
        footRotationAngles[1] = stepParameters->stepX[1]->getPosition(phase) * rotation;	
        // correct step size in x-direction 
        correctStepSizeForRotation(xSizeRotLeft, xSizeRotRight);
      }
      else if(ySize != 0)
      {
        footRotationAngles[0] = sgn(ySize) * stepParameters->stepX[0]->getPosition(phase) * rotation;
        footRotationAngles[1] = sgn(ySize) * stepParameters->stepX[1]->getPosition(phase) * rotation;
      }
      else 
      {
        footRotationAngles[0] = stepParameters->stepX[0]->getPosition(phase) * -rotation;
        footRotationAngles[1] = stepParameters->stepX[1]->getPosition(phase) * rotation;
      } 
    }

    //D_INFO("rot r:" << footRotationAngles[0] << " rot l: " << footRotationAngles[1])

    // calculation of both foot target positions with current phase and parameters
    for(int side = 0; side < 2; ++side)
    {
      // calculate leg height, width and x-position
      double legX = stepParameters->params[WalkStepParameters::stepOriginX] + 
	            stepParameters->stepX[side]->getPosition(phase) * 
	            ((side == 0) ? xSizeRotLeft : xSizeRotRight);
      double legY = stepParameters->params[WalkStepParameters::stepOriginY] * (side ? -1 : 1) + 
                    stepParameters->stepX[side]->getPosition(phase) * ySize;
      double legZ = stepParameters->params[WalkStepParameters::stepOriginZ] + 
                    stepParameters->stepHeight[side]->getPosition(phase);
      
      // set current target foot positions
      footPositions[side] = Vector3<double>(legX, legY, legZ);
    }

    // if all values are zero, just stand on the position
    if(ySize == 0 && 
       xSize == 0 && 
       rotation == 0 && 
       fabs(bodyShift) < 10)
    {
      footPositions[0].x = stepParameters->params[WalkStepParameters::stepOriginX];
      footPositions[1].x = stepParameters->params[WalkStepParameters::stepOriginX];
      footPositions[0].z = stepParameters->params[WalkStepParameters::stepOriginZ];
      footPositions[1].z = stepParameters->params[WalkStepParameters::stepOriginZ];
      bodyShift          = stepParameters->params[WalkStepParameters::bodyShiftOrigin];
      bodyTilt           = stepParameters->params[WalkStepParameters::bodyTiltOrigin];

      // check current phase
      //(phase < 0.5) ? phase = 0.5 : phase = 0.0;
    }
    
    // target joint calculation for both foot and arm positions
    for(int side = 0; side < 2; ++side)
    {
      // finally calculate the leg angles upon the current target footPosition
      calcLegJoints(footPositions[side], 
                    bodyTilt, 
		    bodyShift, 
                    side ? JID_LEG_RIGHT1 : JID_LEG_LEFT1);

      // calculate arm joint - simple because the robot will move his arms simultanly
      // to the foot position of the leg on the other body side.
      calcArmJoints(side, side ? JID_ARM_RIGHT0 : JID_ARM_LEFT0);
    }
    
      /*     
    D_INFO( "[vw3d] we traj: phase " << phase << 
         " - bS " << bodyShift << 
         " - bT " << bodyTilt << 
         " - fLx " <<  footPositions[0].x << " - fLz " <<  footPositions[0].z << 
         " - fRx " <<  footPositions[1].x << " - fRz " <<  footPositions[1].z) 
      */
	 
    // enter odometry related calculations here  ---------- TODO
    long time = theFrameInfo->time_ms;
    float factor = (float)(time - odometryTimestamp) / 1000.0f;
    if(factor < 0.05)
    {
      Vector2<double> rotated(currentSpeed.translation * 0.00066f);
      rotated.rotate(theOdometry->pose.rotation);
      odometryData.translation += rotated * factor;
      odometryData.rotation = 0; //not set here
    }
    odometryTimestamp = time;
    
    //D_INFO("odometry in walkingEngine = " << odometryData.translation.x << "," << odometryData.translation.y << "  ,   " << odometryData.rotation);

    // set motion infos
    executedWalkRequest = theMotionRequest->walkRequest;
    motionIsStable = true;
    positionInWalkCycle = phase;
  }
  
  lastMotionType = theMotionRequest->motion;
  // TODO remove this, when the speed is measuered. Now the speed is increased
  // also when a special action is running and the robot is not walking yet.
  // So this checks, whether the walking is actually active.
  if(theSpecialActionsOutput->active || theSpecialMotionsOutput->active)
    lastMotionType = MotionRequest::SPECIAL_ACTION;
}

bool Hoap2WalkingEngine::isLeavingPossible()
{
  // Only allow to leave the movement if both feet are on the ground
  return this->theMotionRequest->motion != MotionRequest::WALK ||
         (fabs(bodyShift) < 10);
}

void Hoap2WalkingEngine::calcLegJoints(const Vector3<double>& position, 
					 double bodyTilt, 
					 double bodyShift, 
					 JOINT_ID joint)
{
  // set sign of joint
  double sign = joint == JID_LEG_LEFT1 ? 1 : -1;

  // foot is assumed to be flat on the ground, so we can remove offset between ground and joint 5
  Vector3<double> offset = position - Vector3<double>(0, 
						      bodyShift + 
						      sign * config.getValue("dimensions_lengthBetweenLegs",78.0) / 2.0, 
						      -config.getValue("dimensions_footHeight",33.25));

  // now we can calculate joints 1 and 5
  jointAngles[joint] = -atan2(sign * offset.y, -offset.z);
  jointAngles[joint + 4] = -jointAngles[joint];

  /* now we know the positions of joint 2 and 4 in space, so remove their offsets
  // no offsets for hoap2
  offset -= Vector3<double>(0, sign * 
			       dimensions->lengthHipToLegJoint2 *  
			       -sin(jointAngles[joint]), 
                               dimensions->lengthHipToLegJoint2 *  
			       -cos(jointAngles[joint])); 
  offset -= Vector3<double>(0, sign *  
			       dimensions->lengthFootToLegJoint4 *  
			       sin(jointAngles[joint + 4]), 
                               dimensions->lengthFootToLegJoint4 *  
			       -cos(jointAngles[joint + 4]));  
  */

  jointAngles[joint + 1] = -Vector2<double>(offset.y *  sin(jointAngles[joint]) +
                                                   offset.z * -cos(jointAngles[joint]), 
                                                   offset.x).angle();
  double diagonal = offset.abs();

  // upperLegLength, lowerLegLength, and diagonal form a triangle, use cosine theorem
  double upperLegLength = config.getValue("dimensions_upperLegLength",135.0);
  double lowerLegLength = config.getValue("dimensions_lowerLegLength",77.65);
  double a1 = (  upperLegLength * upperLegLength
               - lowerLegLength * lowerLegLength + 
	             diagonal * diagonal) / 
               (2 * upperLegLength * diagonal);
  a1 = fabs(a1) > 1.0 ? 0 : acos(a1);

  double a2 = (upperLegLength * upperLegLength + 
	       lowerLegLength * lowerLegLength - 
	       diagonal * diagonal) / 
              (2 * upperLegLength * lowerLegLength);
  a2 = fabs(a2) > 1.0 ? pi : acos(a2);

  jointAngles[joint + 1] -= a1;
  jointAngles[joint + 2] = pi - a2;
  jointAngles[joint + 3] = -jointAngles[joint + 1] - jointAngles[joint + 2];
  //jointAngles[joint + 3] -= 0.03;  //hack: offset for foot angle
  jointAngles[joint + 1] -= bodyTilt;

  // rotation angle
  jointAngles[joint - 1] = footRotationAngles[joint == JID_LEG_LEFT1 ? 0 : 1] ;

  // mapping
  jointAngles[joint - 1] =  jointAngles[joint - 1];
  jointAngles[joint]     =  -sign * jointAngles[joint];
  jointAngles[joint + 1] = -jointAngles[joint + 1];
  jointAngles[joint + 2] = -jointAngles[joint + 2];
  jointAngles[joint + 3] = -jointAngles[joint + 3];
  jointAngles[joint + 4] =  -sign * jointAngles[joint + 4];

  /*
  cout << "Joints: side " << sign << endl;
  cout << "j0 " << jointAngles[joint - 1] << " " << endl; 
  cout << "j1 " << jointAngles[joint] << " " << endl; 
  cout << "j2 " << jointAngles[joint + 1] << " " << endl; 
  cout << "j3 " << jointAngles[joint + 2] << " " << endl; 
  cout << "j4 " << jointAngles[joint + 3] << " " << endl; 
  cout << "j5 " << jointAngles[joint + 4] << " " << endl; 
  */
}

void Hoap2WalkingEngine::calcArmJoints(uint32_t side, JOINT_ID joint)
{
  double upperArmLength = config.getValue("dimensions_upperArmLength",64.0);
  double lowerArmLength = config.getValue("dimensions_lowerArmLength",134.4);
  double shoulderLength = config.getValue("dimensions_shoulderLength",28.6);
  // for arm joint 1: always keep the arm synchron to the leg on the other side, 
  // stretched by the factor armTiltScale 
  double armLengthVertical      = lowerArmLength + 
                                  cos(-pi/2 - 
				      stepParameters->params[WalkStepParameters::armRollOrigin]) * 
                                      (upperArmLength + 
				       shoulderLength);

  jointAngles[joint + 1] = ((side) ? -1 : 1) * -stepParameters->params[WalkStepParameters::armRollOrigin];
  jointAngles[joint    ] = -pi/2 +
                                  tan(footPositions[(side+1)%2].x/armLengthVertical) * 
                                  stepParameters->params[WalkStepParameters::armTiltScale];
  jointAngles[joint + 2] = 0; //((side) ? -1 : 1) * pi/2;
  jointAngles[joint + 3] = ((side) ? -1 : 1) * pi/4;//-pi - stepParameters->params[WalkStepParameters::armRollOrigin];
  
  /*
  jointAngles[joint] =
    jointAngles[joint + 1] =
    jointAngles[joint + 2] =
    jointAngles[joint + 3] = 0.0;
  */
}

void Hoap2WalkingEngine::limitStepSizes(double& xSize, double& ySize, double& rotSize)
{
  // apply behaviour max step size 
  double maxStepSizeX = config.getValue("maxStepSizeX", 1500.0);
  double maxStepSizeY = config.getValue("maxStepSizeY", 500.0);
  double maxStepSizeRot = config.getValue("maxStepSizeRot", 1500.0);
  rotSize = min(maxStepSizeRot, max(-maxStepSizeRot, rotSize));
  xSize   = min(maxStepSizeX, max(-maxStepSizeX,   xSize));
  ySize   = min(maxStepSizeY, max(-maxStepSizeY,   ySize));

  double rotTest = fabs(rotSize),
         xTest   = xSize + rotTest * (xSize < 0 ? -1 : 1);

  // check whether (xTest, yStep) is inside ellipse described by maxStepSize
  if((xTest * xTest / (maxStepSizeX * maxStepSizeX) +
      ySize * ySize / (maxStepSizeY * maxStepSizeY)) > 1.0)
  {
    // step size outside of ellipse described by maxStepSize
    // calculate point on ellipse with direction of xTest, ySize
    if(xTest)
    {
      double m = ySize / xTest;
      xSize = sgn(xTest) * maxStepSizeX * maxStepSizeY / 
        sqrt(maxStepSizeY * maxStepSizeY + 
             maxStepSizeX * maxStepSizeX * m * m);
      ySize = m * xSize;
    } 
    else 
    {
      xSize = 0;
      ySize = sgn(ySize) * maxStepSizeY;
    }

    // subtract rotation, but keep direction of xSize
    if(fabs(xSize) > rotTest)
      xSize -= sgn(xSize) * rotTest;
    else
    {
      rotSize = fabs(xSize) * sgn(rotSize);
      xSize   = sgn(xSize)  * 0.001;
    }
  }
}

void Hoap2WalkingEngine::correctStepSizeForRotation(double& xSizeRotLeft, double& xSizeRotRight)
{
  // calculate current walk radius
  double radius = (fabs(currentSpeed.translation.x)/(4.0f * 100.0f)) /
    tan(fabs(currentSpeed.rotation));

  // / 4.0f);
  //cout << "walk radius: " << radius << endl;
  
  // calculate radius for y-walking
  //double y_radius = (fabs(currentSpeed.translation.y)/(4.0f * 100.0f)) /
  //  tan(fabs(currentSpeed.rotation));// / 4.0f);
  //cout << "walk radius: " << y_radius << endl;

  // calculate current step correct ratio
  double ratio = min(10.0, (max(0.4, (radius + 0.4))) / (max(0.001, (radius - 0.4))));
  //cout << "curr ratio: " << ratio << endl;
  
  //cout << "step size: " << xSizeRotLeft << " with rot: " << currentSpeed.rotation << endl;
  // decide which is the inner foot
  if(currentSpeed.rotation > 0.0){
    xSizeRotLeft  = 2 * xSizeRotLeft  / (1 + ratio);
    xSizeRotRight = 2 * xSizeRotRight / ( 1.0 + (1.0/ratio)); 
  }else{
    xSizeRotRight = 2 * xSizeRotRight / (1 + ratio);
    xSizeRotLeft  = 2 * xSizeRotLeft  / ( 1.0 + (1.0/ratio));
  }
  
  // switch while walking backward
  if(currentSpeed.translation.x < 0.0){
    double temp   = xSizeRotRight;
    xSizeRotRight = xSizeRotLeft;
    xSizeRotLeft  = temp;
  }
  
  //cout << "corr left step: " << xSizeRotLeft << endl;
  //cout << "corr right step: " << xSizeRotRight << endl;
  return;
}
