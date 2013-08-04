#ifndef MOTIONREQUEST_H
#define MOTIONREQUEST_H

#include "kernel/Template.h"
#include "math/Pose2D.h"
#include "WalkRequest.h"

REPRESENTATION(MotionRequest)

class MotionRequest: public MotionRequestBase
{
  public:

    enum Motions
    {
      WALK, SPECIAL_ACTION, KICK_LEFT,     // 2
      KICK_RIGHT,    // 3
      KICK_LEFT_45,  // 4
      KICK_RIGHT_45, // 5 
      KICK_LEFT_90,     //6 SPECIAL_ACTION: SPECIAL_KICK_LEFT_SIDE
      KICK_RIGHT_90,    //7 SPECIAL_ACTION: SPECIAL_KICK_RIGHT_SIDE
      KICK_LEFT_TO_RIGHT,  //  8 SPECIAL_ACTION: SPECIAL_KICK_LEFT_FORWARD_SIDE
      KICK_RIGHT_TO_LEFT,  //  9 SPECIAL_ACTION: SPECIAL_KICK_RIGHT_FORWARD_SIDE
      DEAD,                // 10
      STAND,               // 11
      NUM_MOTIONS
    };

    enum SpecialAction
    {
      SPECIAL_STAND,
      STAND_UP_FRONT,
      STAND_UP_BACK,
      SPECIAL_KICK_RIGHT,
      SPECIAL_KICK_LEFT,
      SPECIAL_KICK_RIGHT_SMALL_FORWARD,
      SPECIAL_KICK_LEFT_SMALL_FORWARD,
      SPECIAL_KICK_RIGHT_SIDE,
      SPECIAL_KICK_LEFT_SIDE,
      SPECIAL_KICK_RIGHT_FORWARD_SIDE,
      SPECIAL_KICK_LEFT_FORWARD_SIDE,
      SPECIAL_KICK_RIGHT_FORWARD_SIDE_90,
      SPECIAL_KICK_LEFT_FORWARD_SIDE_90,
      NUM_SPECIAL_ACTION
    };

    /** The selected motion type. */
    Motions motion;

    /** Additional parameters for some motions */
    Pose2D walkRequest;
    WalkRequest complexWalkRequest;  // TODO WALK set values somewhere
    SpecialAction specialActionRequest;
    double kickSpeed;

    /** Initialization */
    MotionRequest() :
        motion(DEAD), specialActionRequest(SPECIAL_STAND), kickSpeed(0.0f)
    {
    }


    void updateMotionRequest(const MotionRequest& that)
    {
      this->motion = that.motion;
      this->walkRequest = that.walkRequest;
      this->specialActionRequest = that.specialActionRequest;
      this->kickSpeed = that.kickSpeed;
    }

    void draw() const
    {
      if (motion == WALK)
      {
        Vector2<double> tmp = walkRequest.translation / 100.0;
        if (tmp.abs() > 1.0)
          tmp.normalize(1.0);
        drawing.line("MotionRequest.walkRequest", 0, 0, 0, tmp.x, tmp.y, 0, 128, 255, 128, 3);
        tmp = Vector2<double>(0.5, 0);
        tmp.rotate(walkRequest.translation.angle());        // theRobotPose->pose.rotation);
        Vector2<double> tmp2 = tmp;
        tmp2.rotate(walkRequest.rotation);
        drawing.line("MotionRequest.walkRequest", tmp.x, tmp.y, 0, tmp2.x, tmp2.y, 0, 128, 255, 128,
            3);
      }
    }
};

#endif

