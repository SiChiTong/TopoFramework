#include "Obstacle.h"
#include <iostream>
#include "math/Vector2.h"

namespace RRT
{

Round_Obstacle::Round_Obstacle(double x, double y, double r)
{
  pos_x = x;
  pos_y = y;
  radius = r;
  radius2 = r * r;
}

bool Round_Obstacle::collision(Vector point)
{
  return (pow(point.x - pos_x, 2) + pow(point.y - pos_y, 2) < radius2);
}

bool Round_Obstacle::collision(Vector p1, Vector p2)
{
  Vector v1 = Vector(p2.x - p1.x, p2.y - p1.y);
  Vector v2 = Vector(pos_x - p1.x, pos_y - p1.y);
  double v1len = sqrt(v1.x * v1.x + v1.y * v1.y);
  double v2lenquadrat = v2.x * v2.x + v2.y * v2.y;
  double v2len = sqrt(v2lenquadrat);
  double winkel = acos((v1.x * v2.x + v1.y * v2.y) / (v1len * v2len));
  if (winkel > 3.1416 / 2.0)
    return (v2lenquadrat <= radius2);
  double x = cos(winkel) * v2len;
  if (x > v1len)
    return ((pos_x - p2.x) * (pos_x - p2.x) + (pos_y - p2.y) * (pos_y - p2.y) <= radius2);
  double d = sin(winkel) * v2len;
  return (d * d <= radius2);
}

Vector Round_Obstacle::center()
{
  return Vector(pos_x, pos_y);
}

Vector Round_Obstacle::escape_direction(Vector p)
{
  return p - center();
}

void Round_Obstacle::avoid_collision_with_pose(Vector p)
{
  if (collision(p))
  {
    radius = (p - center()).abs() - 0.001;
    radius2 = radius * radius;
  }
}

void Round_Obstacle::getBorderPoints(double res, std::vector<Vector> &pts)
{
  double step = pi2 / 8.0;
//  step = M_PI*2.0 / ((int)(M_PI*2.0/step)+1);
  for (double angle = 0; angle < pi2; angle += step)
    pts.push_back(center() + Vector(cos(angle), sin(angle)) * radius);
}

void Round_Obstacle::draw(ime::Drawing& drawing, int r, int g, int b) const
{
  drawing.circle("PathPlanning.obstacles", pos_x, pos_y, radius, 2, r, g, b);
}

Line_Obstacle::Line_Obstacle(double x1_, double y1_, double x2_, double y2_, double r_)
{
  x1 = x1_;
  y1 = y1_;
  x2 = x2_;
  y2 = y2_;
  radius = r_;
  radius2 = r_ * r_;
}

bool Line_Obstacle::collision(Vector point)
{
  Vector v1 = Vector(x2 - x1, y2 - y1);
  Vector v2 = Vector(point.x - x1, point.y - y1);
  double v1len = sqrt(v1.x * v1.x + v1.y * v1.y);
  double v2lenquadrat = v2.x * v2.x + v2.y * v2.y;
  double v2len = sqrt(v2lenquadrat);
  double winkel = acos((v1.x * v2.x + v1.y * v2.y) / (v1len * v2len));
  if (winkel > 3.1416 / 2.0)
    return (v2lenquadrat <= radius2);
  double x = cos(winkel) * v2len;
  if (x > v1len)
    return ((point.x - x2) * (point.x - x2) + (point.y - y2) * (point.y - y2) <= radius2);
  double d = sin(winkel) * v2len;
  return (d * d <= radius2);
}

bool Line_Obstacle::collision(Vector p1, Vector p2)
{
  Vector a1(x1, y1);
  Vector b1(x2 - x1, y2 - y1);
  Vector a2(p1.x, p1.y);
  Vector b2(p2.x - p1.x, p2.y - p1.y);
  double s, t;
  if (b2.x != 0)
  {
    double q = b1.y - (b1.x * b2.y) / b2.x;
    if (q == 0)
      s = -1.0;
    else
      s = (a2.y - a1.y + (b2.y / b2.x) * (a1.x - a2.x)) / q;
    t = (b1.x * s + a1.x - a2.x) / b2.x;
  }
  else if (b2.y != 0)
  {
    double q = b1.x - (b1.y * b2.x) / b2.y;
    if (q == 0)
      s = -1.0;
    else
      s = (a2.x - a1.x + (b2.x / b2.y) * (a1.y - a2.y)) / q;
    t = (b1.y * s + a1.y - a2.y) / b2.y;
  }
  else
    s = -1.0;

  if (s >= 0 && s <= 1.0 && t >= 0 && t <= 1.0)
    return true;
  if (collision(p1) || collision(p2))
    return true;
  Line_Obstacle line(p1.x, p1.y, p2.x, p2.y, radius);
  return line.collision(Vector(x1, y1)) || line.collision(Vector(x2, y2));
}

Vector Line_Obstacle::center()
{
  return Vector(x1 + x2, y1 + y2) * 0.5;
}

Vector Line_Obstacle::escape_direction(Vector p)
{
  Vector u(x2 - x1, y2 - y1);
  Vector v(p.x - x1, p.y - y1);
  const double uAbs = u.abs();
  double l = (u.x * v.x + u.y * v.y) / uAbs;
  if (l < 0)
    l = 0;
  if (l > uAbs)
    l = uAbs;
  u.normalize(l);
  Vector nearest_point_on_line(x1 + u.x, y1 + u.y);
  return p - nearest_point_on_line;
}

void Line_Obstacle::avoid_collision_with_pose(Vector p)
{
  if (!collision(p))
    return;

  Vector u(x2 - x1, y2 - y1);
  Vector v(p.x - x1, p.y - y1);
  double l = (u.x * v.x + u.y * v.y) / u.abs();
  if (l <= 0)
    radius2 = (Vector(x1, y1) - p).abs() - 1;
  else if (l >= u.abs())
    radius2 = (Vector(x2, y2) - p).abs() - 1;
  radius = (Vector(x1, y1) + u.normalize(l) - p).abs() - 1;
  radius2 = radius * radius;
}

void Line_Obstacle::getBorderPoints(double res, std::vector<Vector> &pts)
{
  Vector2<double> p1(x1, y1);
  Vector2<double> p2(x2, y2);

  /*  double lineLength = (p1-p2).abs();
   Vector2<double> pA = p1 + (p2-p1).normalize(radius2).rotate( M_PI_2);
   Vector2<double> pB = p1 + (p2-p1).normalize(radius2).rotate(-M_PI_2);
   double lineStep = lineLength / ((int)(lineLength / res)+1);
   for(double s=0; s<lineLength+0.5*lineStep; s+=lineStep)
   {
   pts.push_back(pA + (p2-p1).normalize(s));
   pts.push_back(pB + (p2-p1).normalize(s));
   }*/

  double step = (double) res / radius;
  double a1 = (p2 - p1).angle() + pi_2;
  double a2 = (p1 - p2).angle() + pi_2;
  for (double angle = 0; angle < pi; angle += step)
    pts.push_back(p1 + Vector(cos(a1 + angle), sin(a1 + angle)) * radius);
  for (double angle = 0; angle < pi; angle += step)
    pts.push_back(p2 + Vector(cos(a2 + angle), sin(a2 + angle)) * radius);
}

void Line_Obstacle::draw(ime::Drawing& drawing, int r, int g, int b) const
{
  drawing.circle("PathPlanning.obstacles", x1, y1, radius, 2, r, g, b);
  drawing.circle("PathPlanning.obstacles", x2, y2, radius, 2, r, g, b);

  Vector2<double> u(-(y2 - y1), x2 - x1);
  u.normalize(radius);
  Vector2<double> v = u * -1.0;

  drawing.line("PathPlanning.obstacles", x1 + u.x, y1 + u.y, 0, x2 + u.x, y2 + u.y, 0, r, g, b, 2);
  drawing.line("PathPlanning.obstacles", x1 + v.x, y1 + v.y, 0, x2 + v.x, y2 + v.y, 0, r, g, b, 2);
}

} //namespace RRT
