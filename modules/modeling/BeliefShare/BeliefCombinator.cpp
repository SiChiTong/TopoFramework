#include "BeliefCombinator.h"

MAKE_MODULE(BeliefCombinator)

void BeliefCombinator::execute()
{
  //log << theLocalRobotPose->confidence << "  " << theTeamRobotPose->confidence << std::endl;
  if (theLocalRobotPose->confidence >= theTeamRobotPose->confidence)
    robotPose = theLocalRobotPose;
  else
    robotPose = theTeamRobotPose;
}

void BeliefCombinator::update(RobotPose& theRobotPose)
{
  *((RobotPoseData*) &theRobotPose) = *robotPose;
}

void BeliefCombinator::update(BallPos& theBallPos)
{
  /** @todo use team ball, when confidences are correct */
  if (theLocalBallPos->confidence > 0.1 || theTeamBallPos->confidence < 0.1)
  {
//printf("ball local  (confidence %f)\n",theLocalBallPos->confidence);
    *((BallPosData*) &theBallPos) = *theLocalBallPos;
    theBallPos.absPos = robotPose->pose * theBallPos.relativePos;
  }
  else
  {
//printf("ball team  (confidence %f)\n",theTeamBallPos->confidence);
    *((BallPosData*) &theBallPos) = *theTeamBallPos;
    theBallPos.relativePos = robotPose->pose.invert() * theBallPos.absPos;
    //if RobotPose has a lower confidence, adjust relative BallPos confidence
    if (robotPose->confidence < theBallPos.confidence)
      theBallPos.confidence = robotPose->confidence;
  }
}

void BeliefCombinator::update(OtherRobots& theOtherRobots)
{
  theOtherRobots.teammates.clear();
  theOtherRobots.opponents.clear();

  //array with state pointers for all robots
  const int MAX_UNUM = OtherRobotsData::MAX_UNUM;
  const OtherRobotsData::State* states[MAX_UNUM * 2 + 1];
  for (int i = 0; i < MAX_UNUM * 2 + 1; i++)
    states[i] = NULL;

  //get all iterators, begin and end
  std::vector<OtherRobotsData::State>::const_iterator iter[] =
  { theLocalOtherRobots->teammates.begin(), /** @todo upright bad in local states */
  theLocalOtherRobots->opponents.begin(), theTeamOtherRobots->teammates.begin(),
      theTeamOtherRobots->opponents.begin() };
  std::vector<OtherRobotsData::State>::const_iterator iterEnd[] =
  { theLocalOtherRobots->teammates.end(), theLocalOtherRobots->opponents.end(),
      theTeamOtherRobots->teammates.end(), theTeamOtherRobots->opponents.end() };
  //output vectors
  std::vector<OtherRobotsData::State>* output[] =
  { &theOtherRobots.teammates, &theOtherRobots.opponents };

  //select states with highest confidences
  for (int i = 0; i < 4; i++)
    while (iter[i] != iterEnd[i])
    {
      const OtherRobotsData::State &robot = *(iter[i]);
      if (robot.unum > MAX_UNUM || robot.unum < 0)
      {
        //log << "bad unum (" << robot.unum << ")!" << std::endl;
      }
      else
      {
        const int index = robot.unum + (MAX_UNUM * (i & 1)); //every second iter for opp
        if (i >= 2 && states[index] == NULL)
        {
          output[i & 1]->push_back(robot);
        }
        else if (states[index] == NULL)
        {
          states[index] = &robot;
        }
        else if (states[index]->confidence >= robot.confidence)
        {
          output[i & 1]->push_back(*(states[index]));      // correct confidence values needed
          states[index] = NULL;
        }
        else
        {
          output[i & 1]->push_back(robot);
          states[index] = NULL;
        }
      }
      iter[i]++;
    }
  //add remaining robot state, happens when robot is not in team belief
  for (int i = 0; i < 2 * MAX_UNUM + 1; i++)
    if (states[i] != NULL)
      output[i & 1]->push_back(*(states[i]));
}

