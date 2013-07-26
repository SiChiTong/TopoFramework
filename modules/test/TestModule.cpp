/*
 * TestModule.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#include "TestModule.h"
#include <sstream>

TestModule::TestModule() :
    lastTime(0)
{
}

void TestModule::execute()
{
}

void TestModule::update(SayMessage& theSayMessage)
{
  theSayMessage.active = true;
  std::stringstream ss;
  ss << "msg:" << thePlayerInfo->unum << ":" << theFrameInfo->time_ms;
  theSayMessage.msg = ss.str();
}

void TestModule::update(BeamRequest& theBeamRequest)
{
  static uint32_t startTime = 0;
  if (startTime < 100)
  {
    ++startTime;
    myBeamRequest.translation.x = config.getValue("beamRequest.x", -5.0f);
    myBeamRequest.translation.y = config.getValue("beamRequest.y", 0.0f);
    myBeamRequest.rotation = config.getValue("beamRequest.rotation", 0.0f) * M_PI / 180.0f;
    theBeamRequest.active = true;
    theBeamRequest.pose = myBeamRequest;
  }
  else
  {
    theBeamRequest.active = false;
  }
}

void TestModule::update(MotionRequest& theMotionRequest)
{
  static float sign = -1.0f;
  if (config.getValue("walkRequest", false))
  {
    if (theFallState->orientation == ORIENTATION_LYING_FORWARD)
    {
      theMotionRequest.motion = MotionRequest::SPECIAL_ACTION;
      theMotionRequest.specialActionRequest = MotionRequest::STAND_UP_FRONT;
    }
    else if (theFallState->orientation == ORIENTATION_LYING_BACK)
    {
      theMotionRequest.motion = MotionRequest::SPECIAL_ACTION;
      theMotionRequest.specialActionRequest = MotionRequest::STAND_UP_BACK;
    }
    else
    {
      theMotionRequest.motion = MotionRequest::WALK;
      Pose2D walkRequest(0, 0, 0);
      walkRequest.translation.x = 900.0f * sign;
      walkRequest.translation.y = 0.0f * sign;
      walkRequest.rotation = 0.f;
      if (theFrameInfo->time_ms - lastTime > 10000)
      {
        sign *= -1.0;
        lastTime = theFrameInfo->time_ms;
      }
      theMotionRequest.walkRequest = walkRequest;
    }
  }
  else
  {
    theMotionRequest.motion = MotionRequest::SPECIAL_ACTION;
    theMotionRequest.specialActionRequest = MotionRequest::SPECIAL_STAND;
  }

  if (config.getValue("drawHeadCoordinateSystem", true))
  {

    // Drawing the coordinate frame
    Pose3D robotPose = Pose3D();
    robotPose.translate(theRobotPose->pose.translation.x, theRobotPose->pose.translation.y, 0.0f);
    robotPose.rotateZ(theRobotPose->pose.rotation);
    robotPose.conc(*theTorsoPose);

    Pose3D cameraMatrix = Pose3D();
    cameraMatrix.translate(0, 0.005, 0.09 + 0.065);
    cameraMatrix.rotateZ(theJointData->values[JID_HEAD_PAN].angle);
    cameraMatrix.rotateY(-theJointData->values[JID_HEAD_TILT].angle);
    robotPose.conc(cameraMatrix);

    Vector3<double> v1 = robotPose.translation;
    Vector3<double> v2 = robotPose * Vector3<double>(1, 0, 0);
    drawing.line("CoordianteFramex", v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, 255, 0, 0, 2);
    Vector3<double> v3 = robotPose * Vector3<double>(0, 1, 0);
    drawing.line("CoordianteFramey", v1.x, v1.y, v1.z, v3.x, v3.y, v3.z, 0, 255, 0, 2);
    Vector3<double> v4 = robotPose * Vector3<double>(0, 0, 1);
    drawing.line("CoordianteFramez", v1.x, v1.y, v1.z, v4.x, v4.y, v4.z, 0, 0, 255, 2);
  }
}

void TestModule::update(SpecialMotionsOutput& theSpecialMotionsOutput)
{
  theSpecialMotionsOutput.active = false;
}

void TestModule::update(DeadMotionOutput& theDeadMotionOutput)
{
  theDeadMotionOutput.active = false;
}

void TestModule::update(KickMotionOutput& theKickMotionOutput)
{
  theKickMotionOutput.active = false;
}

void TestModule::update(HeadMotionRequest& theHeadMotionRequest)
{
  theHeadMotionRequest.pan = config.getValue("HeadMotionRequest.pan", 0.0f) * M_PI / 180.0;
  theHeadMotionRequest.tilt = config.getValue("HeadMotionRequest.tilt", 0.0f) * M_PI / 180.0;
}

MAKE_MODULE(TestModule)

