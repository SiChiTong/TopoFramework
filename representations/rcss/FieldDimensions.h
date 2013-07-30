#ifndef FIELDDIMENSIONS_H
#define FIELDDIMENSIONS_H

#include "kernel/Template.h"

REPRESENTATION(FieldDimensions)

class FieldDimensions: public FieldDimensionsBase
{
  public:

    FieldDimensions() :
        initialized(false), length(0), halfLength(0), width(0), halfWidth(0), borderStripWidth(0), borderStripLength(
            0), lengthPlusBorder(0), halfLengthPlusBorder(0), widthPlusBorder(0), halfWidthPlusBorder(
            0), goalWidth(0), halfGoalWidth(0), goalDepth(0), goalHeight(0), goalPostRadius(0), lineWidth(
            0), penaltyAreaWidth(0), halfPenaltyAreaWidth(0), penaltyAreaDepth(0), centerCircleRadius(
            0), ballRadius(0), penaltyMarkSize(0), penaltyMarkDistance(0), freeKickDistance(0), freeKickMoveDist(
            0), goalKickDistance(0)
    {
    }
    bool initialized;

    //field size
    float length;
    float halfLength;
    float width;
    float halfWidth;

    float borderStripWidth;
    float borderStripLength;
    float lengthPlusBorder;
    float halfLengthPlusBorder;
    float widthPlusBorder;
    float halfWidthPlusBorder;

    //goal
    float goalWidth;
    float halfGoalWidth;
    float goalDepth;
    float goalHeight;
    float goalPostRadius;

    //field lines
    float lineWidth;
    float penaltyAreaWidth;
    float halfPenaltyAreaWidth;
    float penaltyAreaDepth;
    float centerCircleRadius;

    //ball
    float ballRadius;

    //spl
    float penaltyMarkSize;
    float penaltyMarkDistance;
    //simspark
    float freeKickDistance;
    float freeKickMoveDist;
    float goalKickDistance;

};

#endif

