#ifndef WALKINGENGINEOUTPUT_H
#define WALKINGENGINEOUTPUT_H

#include "kernel/Template.h"
#include "common/JointValues.h"
#include "math/Pose2D.h"
#include "math/Vector3.h"
#include "WalkRequest.h"

REPRESENTATION(WalkingEngineOutput)

class WalkingEngineOutput: public WalkingEngineOutputBase
{
  public:

    // Used by both walking engines:
    bool active;
    JointValues values;
    Pose2D currentSpeed;

    // Values used by old walking engine:
    Pose2D odometry;
    long odometryTimestamp;

    // The following values are used by the B-Human walking engine:                           

    bool standing; /**< Whether the robot is standing or walking */
    Pose2D odometryOffset; /**< The body motion performed in this step. */
    Pose2D upcomingOdometryOffset; /**< The remaining odometry offset for the currently executed step. */
    bool upcomingOdometryOffsetValid; /**< Whether the \c upcomingOdometryOffset is precise enough to be used */

    bool isLeavingPossible; /**< Is leaving the motion module possible now? */
    float positionInWalkCycle; /**< The current position in the walk cycle in the range [0..1[. */
    float instability; /**< An evaluation of the current walk stability. */
    WalkRequest executedWalk; /**< The walk currently executed. */
    Vector3<> nextPhaseTranslation;

    /**
     * Default constructor.
     */
    WalkingEngineOutput() :
        isLeavingPossible(true), positionInWalkCycle(0), instability(0.)
    {
    }

};

#endif

