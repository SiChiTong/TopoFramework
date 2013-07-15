/**
 * @file JointValues.h
 * Contains the classes JointValue (values for one joint) and
 * JointValues (array of jointvalues).
 * 
 * @author Andreas Seekircher <aseek@cs.miami.edu>
 */

#ifndef JOINTVALUES_H
#define JOINTVALUES_H


/** Enumeration for the humanoid's joints. */
enum JOINT_ID{
    JID_HEAD_PAN,
    JID_HEAD_TILT,
    JID_ARM_LEFT0,
    JID_ARM_LEFT1,
    JID_ARM_LEFT2,
    JID_ARM_LEFT3,
    JID_ARM_RIGHT0,
    JID_ARM_RIGHT1,
    JID_ARM_RIGHT2,
    JID_ARM_RIGHT3,
    JID_LEG_LEFT0,
    JID_LEG_LEFT1,
    JID_LEG_LEFT2,
    JID_LEG_LEFT3,
    JID_LEG_LEFT4,
    JID_LEG_LEFT5,
    JID_LEG_RIGHT0,
    JID_LEG_RIGHT1,
    JID_LEG_RIGHT2,
    JID_LEG_RIGHT3,
    JID_LEG_RIGHT4,
    JID_LEG_RIGHT5,
    NUM_JOINT_ID
};


/**
 * The JointValue class.
 * This class stores the state of a joint.
 */
class JointValue
{
  public:
    JointValue() : angle(0), speed(0), hardness(1.0), 
                   temperature(0), current(0)  {}
  
    float angle; /**< The angle position in radians. */

    float speed; /**< Angular velocity (simspark Nao). */

    float hardness;    /**< The hardness of the joint control (physical Nao). */
    float temperature; /**< Joint temperature (physical Nao). */
    float current;     /**< Electrical current (physical Nao). */
};

/**
 * This class stores a JointValue for each joint. Behaves like an array.
 */
class JointValues
{
  public:
  
    JointValue values[NUM_JOINT_ID]; /**< The JointValue array. */
    
    /** Constant [] operator to access the array. */
    const JointValue& operator[](const int id) const
    { 
      return values[id];
    }
    /** [] operator to access the array. */
    JointValue& operator[](const int id)
    { 
      return values[id];
    }
    /** Assignment operator that copies all values. */
    const JointValues& operator=(const JointValues &other)
    {
      for(int i=0; i<NUM_JOINT_ID; i++)
        values[i] = other[i];
      return (*this);
    }


    //different names for the joints
    enum Joint
    {
        HeadYaw,
        HeadPitch,
        LShoulderPitch,
        LShoulderRoll,
        LElbowYaw,
        LElbowRoll,
        RShoulderPitch,
        RShoulderRoll,
        RElbowYaw,
        RElbowRoll,
        LHipYawPitch,
        LHipRoll,
        LHipPitch,
        LKneePitch,
        LAnklePitch,
        LAnkleRoll,
        RHipYawPitch,
        RHipRoll,
        RHipPitch,
        RKneePitch,
        RAnklePitch,
        RAnkleRoll,
        numOfJoints
    };

    enum
    {
      off    = 1000, /**< Special angle for switching off a joint. */
      ignore = 2000  /**< Special angle for not overwriting the previous setting. */
    };
};


#endif

