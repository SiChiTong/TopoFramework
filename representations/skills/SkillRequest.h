#ifndef SKILLREQUEST_H
#define SKILLREQUEST_H

#include "kernel/Template.h"

#include "representations/skills/MoveToRequest.h"
#include "representations/motion/MotionRequest.h"

REPRESENTATION(SkillRequest)

class SkillRequest: public SkillRequestBase
{
  public:

    SkillRequest()
    {
      moveToRequest.active = true;
      kickSpeed = 100.0;
    }

    enum Skills
    {
      NONE, MOVETOPOS, GETBALL, KICK, DRIBBLE, NUM_SKILLS
    };

    /** The selected motion type. */
    Skills skill;

    /** If skill is NONE, this MotionRequest will be used. */
    MotionRequest motionRequest;
    /** Target position for move skill. */
    MoveToRequest moveToRequest;
    /** Target for other skills like dribble or kick. */
    Pose2D target;

    double kickAccuracy; /** @todo kick accuracy */
    double kickSpeed; /** kick speed */

    void updateSkillRequest(const SkillRequest& that)
    {
      this->skill = that.skill;
      this->motionRequest.updateMotionRequest(that.motionRequest);
      this->moveToRequest = that.moveToRequest;
      this->target = that.target;
      this->kickAccuracy = that.kickAccuracy;
      this->kickSpeed = that.kickSpeed;
    }

    /** Function to request a special action, just simplifies code */
    void requestSpecialAction(MotionRequest::SpecialAction action)
    {
      skill = SkillRequest::NONE;
      motionRequest.motion = MotionRequest::SPECIAL_ACTION;
      motionRequest.specialActionRequest = action;
    }

    void draw() const
    {
      if (skill == MOVETOPOS && moveToRequest.active)
        drawing.pose("SkillRequest.moveToRequest.target", moveToRequest.target, 0.15, 2, 150, 150,
            150);
    }

};

#endif

