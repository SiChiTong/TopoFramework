#ifndef BEHAVIORTOOLS_H
#define BEHAVIORTOOLS_H


//forward declarations
class BallPos;
class FallState;
class SkillRequest;
class RobotPose;


class BehaviorTools
{
  public:
  
    bool standUp(const FallState *theFallState, SkillRequest* skillRequest);
  
    bool searchBall(const BallPos *ballPos,
                    const RobotPose *robotPose,
                    SkillRequest* skillRequest);
 
};


#endif

