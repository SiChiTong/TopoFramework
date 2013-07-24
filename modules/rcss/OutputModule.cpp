/*
 * OutputModule.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#include "OutputModule.h"

void OutputModule::init()
{
  config.setPersist(false);
}

void OutputModule::execute()
{
  // Generate stringstream
  std::stringstream stream;
  createJointMessage(stream);
  createSayMessage(stream);
  createBeamMessage(stream);
  stream << "(syn)";

  ime::Communication& com = ime::Communication::getInstance();
  com.putMessage(stream.str());
}

void OutputModule::createJointMessage(std::stringstream &stream)
{
  JointValues values = theJointRequestWithSpeeds->values;
  for (int i = 0; i < NUM_JOINT_ID; i++)
    values[i].speed *= 180.0 / pi;  // convert to degrees

  // ----- Head -----
  stream << "(he1 " << values[JID_HEAD_PAN].speed << ")";
  stream << "(he2 " << values[JID_HEAD_TILT].speed << ")";
  // ----- Left Leg -----
  stream << "(lle1 " << values[JID_LEG_LEFT0].speed << ")";
  stream << "(lle2 " << values[JID_LEG_LEFT1].speed << ")";
  stream << "(lle3 " << values[JID_LEG_LEFT2].speed << ")";
  stream << "(lle4 " << values[JID_LEG_LEFT3].speed << ")";
  stream << "(lle5 " << values[JID_LEG_LEFT4].speed << ")";
  stream << "(lle6 " << values[JID_LEG_LEFT5].speed << ")";
  // ----- Right Leg -----
  stream << "(rle1 " << values[JID_LEG_RIGHT0].speed << ")";
  stream << "(rle2 " << values[JID_LEG_RIGHT1].speed << ")";
  stream << "(rle3 " << values[JID_LEG_RIGHT2].speed << ")";
  stream << "(rle4 " << values[JID_LEG_RIGHT3].speed << ")";
  stream << "(rle5 " << values[JID_LEG_RIGHT4].speed << ")";
  stream << "(rle6 " << values[JID_LEG_RIGHT5].speed << ")";
  // ----- Left Arm -----
  stream << "(lae1 " << values[JID_ARM_LEFT0].speed << ")";
  stream << "(lae2 " << values[JID_ARM_LEFT1].speed << ")";
  stream << "(lae3 " << values[JID_ARM_LEFT2].speed << ")";
  stream << "(lae4 " << values[JID_ARM_LEFT3].speed << ")";
  // ----- Right Arm -----
  stream << "(rae1 " << values[JID_ARM_RIGHT0].speed << ")";
  stream << "(rae2 " << values[JID_ARM_RIGHT1].speed << ")";
  stream << "(rae3 " << values[JID_ARM_RIGHT2].speed << ")";
  stream << "(rae4 " << values[JID_ARM_RIGHT3].speed << ")";
}

void OutputModule::createSayMessage(std::stringstream &stream)
{
  if (theSayMessage->active)
    stream << "(say " << theSayMessage->msg << ")";
}

void OutputModule::createBeamMessage(std::stringstream &stream)
{
  if (theBeamRequest->active)
    stream << "(beam " << theBeamRequest->pose.translation.x << " " << theBeamRequest->pose.translation.y << " "
        << theBeamRequest->pose.rotation / pi * 180.0 << ")";
}

MAKE_MODULE(OutputModule)

