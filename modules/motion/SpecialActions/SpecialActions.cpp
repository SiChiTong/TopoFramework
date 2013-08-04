#include "SpecialActions.h"

#include <cstdio>
#include <cassert>
#include <sstream>
#include <fstream>
#include <string>

MAKE_MODULE(SpecialActions)

SpecialActions::SpecialActions() :
    currentAction(MotionRequest::NUM_SPECIAL_ACTION), skillStartTime(0), skillCurrentTime(0)
{
}

void SpecialActions::init()
{

  //jointNameToJointID.insert(std::make_pair("HeadYaw", JID_HEAD_PAN));
  //jointNameToJointID.insert(std::make_pair("HeadPitch", JID_HEAD_TILT));
  jointNameToJointID.insert(std::make_pair("LA0", JID_ARM_LEFT0));
  jointNameToJointID.insert(std::make_pair("LA1", JID_ARM_LEFT1));
  jointNameToJointID.insert(std::make_pair("LA2", JID_ARM_LEFT2));
  jointNameToJointID.insert(std::make_pair("LA3", JID_ARM_LEFT3));
  jointNameToJointID.insert(std::make_pair("RA0", JID_ARM_RIGHT0));
  jointNameToJointID.insert(std::make_pair("RA1", JID_ARM_RIGHT1));
  jointNameToJointID.insert(std::make_pair("RA2", JID_ARM_RIGHT2));
  jointNameToJointID.insert(std::make_pair("RA3", JID_ARM_RIGHT3));
  jointNameToJointID.insert(std::make_pair("LL0", JID_LEG_LEFT0));
  jointNameToJointID.insert(std::make_pair("LL1", JID_LEG_LEFT1));
  jointNameToJointID.insert(std::make_pair("LL2", JID_LEG_LEFT2));
  jointNameToJointID.insert(std::make_pair("LL3", JID_LEG_LEFT3));
  jointNameToJointID.insert(std::make_pair("LL4", JID_LEG_LEFT4));
  jointNameToJointID.insert(std::make_pair("LL5", JID_LEG_LEFT5));
  jointNameToJointID.insert(std::make_pair("LR0", JID_LEG_RIGHT0));
  jointNameToJointID.insert(std::make_pair("LR1", JID_LEG_RIGHT1));
  jointNameToJointID.insert(std::make_pair("LR2", JID_LEG_RIGHT2));
  jointNameToJointID.insert(std::make_pair("LR3", JID_LEG_RIGHT3));
  jointNameToJointID.insert(std::make_pair("LR4", JID_LEG_RIGHT4));
  jointNameToJointID.insert(std::make_pair("LR5", JID_LEG_RIGHT5));

  filenames[MotionRequest::SPECIAL_STAND] = "actions/stand.act"; //0
  filenames[MotionRequest::STAND_UP_FRONT] = "actions/stand_up_from_front.act"; //1
  filenames[MotionRequest::STAND_UP_BACK] = "actions/stand_up_from_back.act"; //2

  std::string formatFileName("actions/format.act");
  std::ifstream formatFile(formatFileName.c_str());
  if (formatFile.is_open())
  {
    std::string str;
    while (getline(formatFile, str))
    {
      if (str.length())
      {
        unsigned int found = str.find_first_of("#");
        if ((found != std::string::npos) && (found > str.size()))
        {
          std::string value;
          std::istringstream iss(str);
          while (iss >> value)
          {
            jointNameOrder.insert(std::make_pair(jointNameOrder.size(), value));
          }
        }
      }
      break; // only one line is enough
    }
    std::cout << "jointNameOrder=" << jointNameOrder.size() << std::endl;
  }
  else
  {
    std::cerr << "ERROR! " << formatFileName << " is not found!" << std::endl;
    exit(EXIT_FAILURE);
  }

  //parse all act files
  for (MotionRequest::SpecialAction a = MotionRequest::SPECIAL_STAND;
      a < MotionRequest::NUM_SPECIAL_ACTION; ++a)
  {
    if (filenames.find(a) != filenames.end())
    {
      std::cout << filenames[a] << std::endl;
      parseActions(a, filenames[a]);
    }
  }
}

void SpecialActions::update(SpecialActionsOutput& theSpecialActionsOutput)
{
  //printf("theMotionRequest->specialActionRequest = %d\n", theMotionRequest->specialActionRequest);

  if (!theBeamRequest.isNull() && theBeamRequest->active)
    reset();

  MotionRequest::Motions requestedMotion = theMotionRequest->motion;
  MotionRequest::SpecialAction requestedSpecialAction = theMotionRequest->specialActionRequest;

  // HACK :(, use special actions for some other motion types (STAND/KICK_RIGHT)
  if (requestedMotion == MotionRequest::STAND)
  {
    requestedMotion = MotionRequest::SPECIAL_ACTION;
    requestedSpecialAction = MotionRequest::SPECIAL_STAND;
  }

  //Don't do anything, if no action running and not action is requested.
  if (currentAction == MotionRequest::NUM_SPECIAL_ACTION
      && (requestedMotion != MotionRequest::SPECIAL_ACTION
          || filenames.find(requestedSpecialAction) == filenames.end()
          || repository[requestedSpecialAction].getStateCount() == 0))
  {
    theSpecialActionsOutput.active = false;
    //printf("theSpecialActionsOutput->active = false;\n");
    return;
  }

  MotionRequest::SpecialAction action;
  if (currentAction == MotionRequest::NUM_SPECIAL_ACTION)
    currentAction = action = requestedSpecialAction;
  else
    action = currentAction;

  // get the action and its next state
  SpecialAction& act = repository[action];
  //state = act.get_next_state();

  //skill just started
  if (act.getCurrentIndex() == 0 && state.getProgress() == 0.0)
  {
    // get current and next state
    skillStartTime = theFrameInfo->time_ms;
    state = act.getNextState();
    nextState = act.getNextState();
  }

  // get current time
  skillCurrentTime = theFrameInfo->time_ms;

  // make progress
  double factor = (double) (skillCurrentTime - skillStartTime) / (double) nextState.getTime();

  nextState.setProgress(factor);

  if (nextState.getProgress() > 1.0)
  {
    state = nextState;
    nextState = act.getNextState();
    skillStartTime = theFrameInfo->time_ms;
  }

  //printf("next_state.get_progress() = %lf\n", next_state.get_progress());

  // interpolate
  interpolateStates(nextState.getProgress());

  JointValues& outputJointAngles = theSpecialActionsOutput.values;

  // use queue motion request here
  outputJointAngles[JID_HEAD_PAN].angle = 0;
  outputJointAngles[JID_HEAD_TILT].angle = 0;

  //head motion
  for (int i = 2; i < NUM_JOINT_ID; i++)
  {
    outputJointAngles[i].angle = (iState.getJointState((JOINT_ID) i) / 180.0f) * M_PI;
  }

  theSpecialActionsOutput.active = (currentAction != MotionRequest::NUM_SPECIAL_ACTION);

  if (act.isFinished())
  {
    act.reset();
    currentAction = MotionRequest::NUM_SPECIAL_ACTION;
  }

  //make sure hardness is 1
  for (int i = 0; i < NUM_JOINT_ID; i++)
  {
    outputJointAngles[i].hardness = 1.0;
  }
}

void SpecialActions::update(SpecialMotionsOutput& theSpecialMotionsOutput)
{
  // fixMe: is this necessary
}

void SpecialActions::reset()
{
  // get the current action
  SpecialAction& act = repository[currentAction];
  act.reset();
  currentAction = MotionRequest::NUM_SPECIAL_ACTION;
}

void SpecialActions::interpolateStates(double factor)
{

  float stateValue = 0.0;
  for (int i = 0; i < NUM_JOINT_ID; i++)
  {
    stateValue = state.getJointState((JOINT_ID) i) * (1 - factor)
        + nextState.getJointState((JOINT_ID) i) * (factor);
    iState.setJointState((JOINT_ID) i, stateValue);
  }
}

void SpecialActions::parseActions(const MotionRequest::SpecialAction& a,
    const std::string& actionFileName)
{
  std::ifstream actionFile(actionFileName.c_str());
  SpecialAction specialAction;
  if (actionFile.is_open())
  {
    std::string str;
    while (getline(actionFile, str))
    {
      if (str.length())
      {
        unsigned int found = str.find_first_of("#");
        if ((found != std::string::npos) && (found > str.size()))
        {
          SpecialActionState specialActionState;
          float value;
          std::istringstream iss(str);
          unsigned int internalId = 0;
          while (iss >> value)
          {
            //std::cout << value << " ";
            if (internalId == jointNameOrder.size() - 1)
            {
              // This is time
              specialActionState.setTime(value);
            }
            else
            {
              std::map<int, std::string>::iterator iter = jointNameOrder.find(internalId);
              if (iter != jointNameOrder.end())
              {
                std::map<std::string, JOINT_ID>::iterator iter2 = jointNameToJointID.find(
                    iter->second);
                if (iter2 != jointNameToJointID.end())
                {
                  specialActionState.setJointState(iter2->second, value);
                }
                else
                {
                  std::cerr << "JOINT_ID does not exits!" << std::endl;
                  exit(EXIT_FAILURE);
                }
              }
              else
              {
                std::cerr << "jointName does not exits!" << std::endl;
                exit(EXIT_FAILURE);
              }
            }
            ++internalId;
          }
          assert(internalId == jointNameOrder.size());
          specialAction.addState(specialActionState);
        }
      }
      //std::cout << std::endl;
    }
  }
  else
  {
    std::cerr << "ERROR! " << actionFileName << " is not found!" << std::endl;
    exit(EXIT_FAILURE);
  }
  repository[a] = specialAction;
}

