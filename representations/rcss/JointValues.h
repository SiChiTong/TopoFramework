/*
 * JointValues.h
 *
 *  Created on: Jul 14, 2013
 *      Author: sam
 */

#ifndef JOINTVALUES_H_
#define JOINTVALUES_H_

#include "kernel/Framework.h"
#include <iostream>
#include <map>

class JointValues
{
  public:
    enum JointID
    {
      JID_HEAD_1,
      JID_HEAD_2,

      JID_LLEG_1,
      JID_RLEG_1,
      JID_LLEG_2,
      JID_RLEG_2,
      JID_LLEG_3,
      JID_RLEG_3,
      JID_LLEG_4,
      JID_RLEG_4,
      JID_LLEG_5,
      JID_RLEG_5,
      JID_LLEG_6,
      JID_RLEG_6,

      JID_LARM_1,
      JID_RARM_1,
      JID_LARM_2,
      JID_RARM_2,
      JID_LARM_3,
      JID_RARM_3,
      JID_LARM_4,
      JID_RARM_4
    };

    class JointValue
    {
      public:
        float angle; // joint angle
        float rate; // joint rate

        JointValue() :
            angle(angle), rate(rate)
        {
        }
    };

    typedef std::map<JointID, JointValue> JointIDToJointValueMap;
    JointIDToJointValueMap jointIDToJointValueMap;
    typedef std::map<std::string, JointID> StringToJointIDMap;
    StringToJointIDMap stringToJointIDMap;

    JointValues()
    {
      stringToJointIDMap["hj1"] = JID_HEAD_1;
      stringToJointIDMap["hj2"] = JID_HEAD_2;

      stringToJointIDMap["llj1"] = JID_LLEG_1;
      stringToJointIDMap["rlj1"] = JID_RLEG_1;
      stringToJointIDMap["llj2"] = JID_LLEG_2;
      stringToJointIDMap["rlj2"] = JID_RLEG_2;
      stringToJointIDMap["llj3"] = JID_LLEG_3;
      stringToJointIDMap["rlj3"] = JID_RLEG_3;
      stringToJointIDMap["llj4"] = JID_LLEG_4;
      stringToJointIDMap["rlj4"] = JID_RLEG_4;
      stringToJointIDMap["llj5"] = JID_LLEG_5;
      stringToJointIDMap["rlj5"] = JID_RLEG_5;
      stringToJointIDMap["llj6"] = JID_LLEG_6;
      stringToJointIDMap["rlj6"] = JID_RLEG_6;

      stringToJointIDMap["laj1"] = JID_LARM_1;
      stringToJointIDMap["raj1"] = JID_RARM_1;
      stringToJointIDMap["laj2"] = JID_LARM_2;
      stringToJointIDMap["raj2"] = JID_RARM_2;
      stringToJointIDMap["laj3"] = JID_LARM_3;
      stringToJointIDMap["raj3"] = JID_RARM_3;
      stringToJointIDMap["laj4"] = JID_LARM_4;
      stringToJointIDMap["raj4"] = JID_RARM_4;
    }

};

REPRESENTATION(InputJointValues)
class InputJointValues: public JointValues, public InputJointValuesBase
{
  public:
    InputJointValues() :
        JointValues()
    {
    }
};

REPRESENTATION(OutputJointValues)
class OutputJointValues: public JointValues, public OutputJointValuesBase
{
  public:
    std::string msg; // TODO: fixMe

    OutputJointValues() :
        JointValues()
    {
    }
};

#endif /* JOINTVALUES_H_ */
