/**
 * @file rrt.h
 * Path planning using Rapidly Exploring Random Tree (ERRT)
 * 
 * @author      Andreas Seekircher
 * 
 */
#ifndef RRT_H
#define RRT_H

#include <vector>
#include "Obstacle.h"
#include "math/Vector2.h"
#include "kernel/Drawing.h"

namespace RRT
{

struct State
{
    Vector position;
    State* parent;
};

struct KDNode
{
    State *state;
    KDNode *left;
    KDNode *right;
};

class RRT
{
  private:

    std::vector<Obstacle*> *obstacles;
    State* waypoints;
    int waypoint_chosen;
    int waypoints_left;
    int lastpathlength;

    State randomState();
    bool extend(State **extended, State *current, State *target);
    double distance(State *current, State *target);

    State* rrtPlan(State *start, State *goal, double goal_threshold, int &nodes_max);
    State chooseTarget(State *goal);

    KDNode *root;
    void addNode(KDNode** node, int d, State *state);
    State* nearestState(KDNode *node, int d, double maxdistance, State *target);
    void deleteTree(KDNode* node);

    std::vector<State*> states_vec;

    /** Reference to Debug object for drawings. */
    ime::Drawing& drawing;

    float fieldLength, fieldWidth;

  public:
    RRT(float fieldLength, float fieldWidth, ime::Drawing &drawing);
    ~RRT();

    Vector createPath(Vector current_pos, Vector target_pos, std::vector<Obstacle*> *obstacles_vec);
    std::vector<Vector> path;
};

} //namespace RRT

#endif
