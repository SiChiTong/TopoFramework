/**
 * @file obstacle.h
 * Obstacle class used by RRT.
 * 
 * @author      Andreas Seekircher
 * 
 */

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <math.h>
#include <vector>
#include "math/Vector2.h"
#include "kernel/Drawing.h"

namespace RRT
{

typedef Vector2<double> Vector;

class Obstacle
{
  public:
    virtual bool collision(Vector point) = 0;
    virtual bool collision(Vector p1, Vector p2) = 0;
    virtual Vector center() = 0;
    virtual Vector escape_direction(Vector p) = 0;
    virtual void avoid_collision_with_pose(Vector p) = 0;
    virtual void getBorderPoints(double res, std::vector<Vector> &pts) = 0;
    virtual void draw(ime::Drawing& drawing, int r, int g, int b) const = 0;

    Obstacle()
    {
    }

    virtual ~Obstacle()
    {
    }
};

class Round_Obstacle: public Obstacle
{
  private:
    double pos_x;
    double pos_y;
    double radius;
    double radius2;
  public:
    Round_Obstacle(double x, double y, double r);
    bool collision(Vector point);
    bool collision(Vector p1, Vector p2);
    Vector center();
    Vector escape_direction(Vector p);
    void avoid_collision_with_pose(Vector p);
    void getBorderPoints(double res, std::vector<Vector> &pts);
    void draw(ime::Drawing& drawing, int r, int g, int b) const;
};

class Line_Obstacle: public Obstacle
{
  public:
    //private:
    double x1;
    double y1;
    double x2;
    double y2;
    double radius;
    double radius2;
  public:
    Line_Obstacle(double x1_, double y1_, double x2_, double y2_, double r_);
    bool collision(Vector point);
    bool collision(Vector p1, Vector p2);
    Vector center();
    Vector escape_direction(Vector p);
    void avoid_collision_with_pose(Vector p);
    void getBorderPoints(double res, std::vector<Vector> &pts);
    void draw(ime::Drawing& drawing, int r, int g, int b) const;
};

} //namespace RRT

#endif
