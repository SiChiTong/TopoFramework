#ifndef SKILLKICKPARAMETERS_H
#define SKILLKICKPARAMETERS_H

#include "kernel/Template.h"
#include "representations/motion/MotionRequest.h"
#include "math/Vector2.h"
#include <map>

REPRESENTATION(SkillKickParameters)

class SkillKickParameters: public SkillKickParametersBase
{
  public:

    class Params
    {
      public:
        bool kickWithLeft;
        Vector2<double> foot;
        double angle;
        double footDistMax;

        double dist;
        double distAngleStart;
        double distAngleFactor;
        double distMax;
    };

    SkillKickParameters() :
        initialized(false)
    {
    }
    bool initialized;

    std::map<MotionRequest::Motions, Params> params;

    void load(ime::Config& config)
    {
      char n[64];
      for (int m = MotionRequest::KICK_LEFT; m <= MotionRequest::KICK_RIGHT_TO_LEFT; m++)
      {
        Params p;
        sprintf(n, "kick%d_enabled", m);
        if (config.getValue(n, false))
        {
          sprintf(n, "kick%d_kickWithLeft", m);
          p.kickWithLeft = config.getValue(n, true);
          sprintf(n, "kick%d_foot.x", m);
          p.foot.x = config.getValue(n, 0.09);
          sprintf(n, "kick%d_foot.y", m);
          p.foot.y = config.getValue(n, 0.07);
          sprintf(n, "kick%d_angle", m);
          p.angle = config.getValue(n, 0.1);
          sprintf(n, "kick%d_footDistMax", m);
          p.footDistMax = config.getValue(n, 0.08);
          sprintf(n, "kick%d_dist", m);
          p.dist = config.getValue(n, 0.05);
          sprintf(n, "kick%d_distAngleStart", m);
          p.distAngleStart = config.getValue(n, M_PI_4 * 0.5);
          sprintf(n, "kick%d_distAngleFactor", m);
          p.distAngleFactor = config.getValue(n, 0.15);
          sprintf(n, "kick%d_distMax", m);
          p.distMax = config.getValue(n, 0.3);
          params[(MotionRequest::Motions) m] = p;
        }
      }
      initialized = true;
    }

    void persist(ime::Config& config) const
    {
      char n[64];
      for (std::map<MotionRequest::Motions, Params>::const_iterator iter = params.begin();
          iter != params.end(); iter++)
      {
        int m = iter->first;
        const Params& p = iter->second;
        sprintf(n, "kick%d_kickWithLeft", m);
        config.setValue(n, p.kickWithLeft);
        sprintf(n, "kick%d_foot.x", m);
        config.setValue(n, p.foot.x);
        sprintf(n, "kick%d_foot.y", m);
        config.setValue(n, p.foot.y);
        sprintf(n, "kick%d_angle", m);
        config.setValue(n, p.angle);
        sprintf(n, "kick%d_footDistMax", m);
        config.setValue(n, p.footDistMax);
        sprintf(n, "kick%d_dist", m);
        config.setValue(n, p.dist);
        sprintf(n, "kick%d_distAngleStart", m);
        config.setValue(n, p.distAngleStart);
        sprintf(n, "kick%d_distAngleFactor", m);
        config.setValue(n, p.distAngleFactor);
        sprintf(n, "kick%d_distMax", m);
        config.setValue(n, p.distMax);
      }
      config.persist();
    }

};

#endif

