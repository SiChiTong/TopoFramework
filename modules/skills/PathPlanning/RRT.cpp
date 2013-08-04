#include "RRT.h"
#include <cstdlib>
#include <iostream>
#include <ctime>

using namespace RRT;

namespace RRT
{
#define NODES_MAX       150

#define USE_KDTREE
#define P_GOAL            0.1
#define P_WAYPOINT        0.7

#define STEP_SIZE       0.2
#define GOAL_THRESHOLD  0.01

RRT::RRT(float fieldLength, float fieldWidth, ime::Drawing &drawing) :
    drawing(drawing), fieldLength(fieldLength), fieldWidth(fieldWidth)
{
  //randomize();
  srand(time(NULL));
  root = NULL;
  waypoints = new State[NODES_MAX];
  waypoints_left = NODES_MAX;
  lastpathlength = NODES_MAX;
  waypoint_chosen = -1;
  for (int i = 0; i < NODES_MAX; i++)
    waypoints[i] = randomState();
}

RRT::~RRT()
{
  delete[] waypoints;
}

Vector RRT::createPath(Vector current_pos, Vector target_pos, std::vector<Obstacle*> *obstacle_vec)
{
  int nodes_max = NODES_MAX;
  obstacles = obstacle_vec;

  deleteTree(root);
  root = NULL;

  State *start, goal;
  start = new State();
  start->position = current_pos;
  goal.position = target_pos;

  //check if start or goal is inside an obstacle
  bool goal_collision = false;
  std::vector<Obstacle*>::iterator iter = obstacles->begin();
  while (iter != obstacles->end())
  {
    if ((*iter)->collision(start->position))
    {
      //if start is inside an obstacle, return position to leave obstacle area
      Vector escape_direction;
      Vector obstacle_center = (*iter)->center();
      //if(   fabs(current_pos.x) > fieldLength*0.5 - 1
      //   || fabs(current_pos.y) > fieldWidth *0.5 - 1)
      //  escape_direction = obstacle_center * -1.0;
      //else
      escape_direction = (*iter)->escape_direction(current_pos);
      return current_pos + escape_direction.normalize(1.0);
    }
    if ((*iter)->collision(goal.position))
      goal_collision = true;
    iter++;
  }

  double threshold = GOAL_THRESHOLD;
  if (goal_collision)
    threshold = GOAL_THRESHOLD + 0.08;

  //search path
  State* nearest = rrtPlan(start, &goal, threshold, nodes_max);

  //copy path to path vector
  bool refresh_waypoints = (nearest == &goal);
  int w = 0;
  path.clear();
  State* current = nearest;
  while (current != start)
  {
    path.push_back(current->position);
    if (refresh_waypoints)
      waypoints[w++] = *current;
    current = current->parent;
  }
  path.push_back(start->position);
  if (refresh_waypoints)
  {
    waypoints_left = w;
    lastpathlength = w;
  }
  else
    waypoints_left = lastpathlength;

  //smooth
  //search point to drive to
  Vector driveToPosition = current_pos;
  Vector first_point = current_pos;
  if (path.size() >= 2)
  {
    unsigned int driveto_index = 0;
    //first_point = path[path.size()-1];
    bool collision = true;
    Obstacle *closestObstacle = NULL;
    while (collision && driveto_index < path.size() - 1)
    {
      collision = false;
      for (unsigned int i = 0; i < obstacles->size(); i++)
        if ((*obstacles)[i]->collision(first_point, path[driveto_index]))
        {
          collision = true;
          closestObstacle = (*obstacles)[i];
        }
      if (collision)
        driveto_index++;
    }
    Vector directionAwayFromObstacle(0, 0);
    if (closestObstacle != NULL)
    {
      directionAwayFromObstacle = closestObstacle->escape_direction(path[driveto_index]);
      drawing.line("PathPlanning.awayFromObstacle", path[driveto_index].x, path[driveto_index].y, 0,
          path[driveto_index].x + directionAwayFromObstacle.x,
          path[driveto_index].y + directionAwayFromObstacle.y, 0, 255, 128, 255, 3);
    }
    double extraDist = ((current_pos - target_pos).abs() < 0.2) ? 0 : 0.15;
    driveToPosition = path[driveto_index] + directionAwayFromObstacle.normalize(extraDist);
  }

  drawing.circle("PathPlanning.walkToPosition", driveToPosition.x, driveToPosition.y, 0.15, 5, 255,
      128, 255);
  drawing.circle("PathPlanning.nearestPosition", nearest->position.x, nearest->position.y, 0.15, 5,
      255, 255, 0);
  drawing.circle("PathPlanning.targetPos", target_pos.x, target_pos.y, 0.15, 5, 0, 0, 255);

  //if there is almost no node towards the target check the direction towards target
  if ((driveToPosition - current_pos).abs() < STEP_SIZE * 0.9
      && (current_pos - target_pos).abs() > 0.2)
  {
    Vector towardsTarget = current_pos + (target_pos - current_pos).normalize(0.2);
    //check if in direction to target_pos is an obstacle
    bool noObstacle = true;
    for (unsigned int i = 0; i < obstacles->size(); i++)
      if ((*obstacles)[i]->collision(first_point, towardsTarget))
        noObstacle = false;
    if (noObstacle)
      return towardsTarget;
  }
  return driveToPosition;
}

State RRT::randomState()
{
  State s;
  s.position.x = rand() * (double) (fieldLength + 3.0) / (double) RAND_MAX
      - (fieldLength + 3.0) / 2.0;
  s.position.y = rand() * (double) (fieldWidth + 3.0) / (double) RAND_MAX
      - (fieldWidth + 3.0) / 2.0;
  return s;
}

bool RRT::extend(State **extended, State *current, State *target)
{
  State *newstate = new State;
  double l = sqrt(distance(current, target));
  if (l == 0)
    return false;
  newstate->position.x = current->position.x
      + (target->position.x - current->position.x) / l * STEP_SIZE;
  newstate->position.y = current->position.y
      + (target->position.y - current->position.y) / l * STEP_SIZE;
  newstate->parent = current;
  *extended = newstate;
  std::vector<Obstacle*>::iterator iter = obstacles->begin();
  while (iter != obstacles->end())
  {
    if ((*iter)->collision(newstate->position))
    {
      delete newstate;
      *extended = NULL;
      return false;
    }
    iter++;
  }
  if (waypoint_chosen != -1)
    if (distance(newstate, target) < GOAL_THRESHOLD)
      waypoints_left = waypoint_chosen;
  return true;
}

double RRT::distance(State *current, State *target)
{
  return pow(current->position.x - target->position.x, 2)
      + pow(current->position.y - target->position.y, 2);
}

State* RRT::rrtPlan(State *start, State *goal, double goal_threshold, int &nodes_max)
{
  State *nearest, target, *extended;
  addNode(&root, 0, start);
  nearest = start;
  while (distance(nearest, goal) > goal_threshold && nodes_max > 0)
  {
    nodes_max--;
    target = chooseTarget(goal);
    nearest = nearestState(root, 0, -1, &target);
    if (extend(&extended, nearest, &target))
    {
      addNode(&root, 0, extended);

      drawing.circle("PathPlanning.tree", extended->position.x, extended->position.y, 0.02, 3, 255,
          255, 255);
    }
  }

  if (distance(nearest, goal) <= goal_threshold)
  {
    goal->parent = nearest;
    return goal;
  }
  nearest = nearestState(root, 0, -1, goal);
  return nearest;
}

State RRT::chooseTarget(State *goal)
{
  waypoint_chosen = -1;
  double r = (double) rand() / (double) RAND_MAX;
  if (r < P_GOAL)
    return *goal;
  else if (waypoints_left > 0 && r < P_GOAL + P_WAYPOINT)
  {
    int w = (int) (((double) rand() / (double) RAND_MAX) * waypoints_left);
    if (w == waypoints_left)
      w--;
    waypoint_chosen = w;
    return waypoints[w];
  }
  return randomState();
}

State* RRT::nearestState(KDNode* node, int d, double maxdistance, State *target)
{
#ifndef USE_KDTREE
  double m = -1;
  State* b = NULL;
  std::vector<State*>::iterator iter = states_vec.begin();
  while(iter!=states_vec.end())
  {
    if(m==-1 || distance(b,target)>distance(*iter, target))
    {
      m = distance(*iter, target);
      b = *iter;
    }
    iter++;
  }
  return b;
#else

  if (node == NULL)
    return NULL;

  State* best = node->state;
  double mindist = distance(best, target);
  if (maxdistance != -1 && maxdistance < mindist)
    mindist = maxdistance;

  bool left = false;
  if (d > 1)
    d = 0;
  if ((d == 0 && target->position.x < node->state->position.x)
      || (d == 1 && target->position.y < node->state->position.y))
    left = true;

  State* state1;
  if (left)
    state1 = nearestState(node->left, d + 1, mindist, target);
  else
    state1 = nearestState(node->right, d + 1, mindist, target);

  if (state1 != NULL)
  {
    double dist1 = distance(state1, target);
    if (dist1 < mindist)
    {
      mindist = dist1;
      best = state1;
    }
  }

  if ((left && node->right != NULL) || (!left && node->left != NULL))
  {
    double planedist = 0;
    if (d == 0)
      planedist = pow(target->position.x - node->state->position.x, 2);
    if (d == 1)
      planedist = pow(target->position.y - node->state->position.y, 2);

    if (planedist < mindist)
    {
      State* state2;
      if (left)
        state2 = nearestState(node->right, d + 1, mindist, target);
      else
        state2 = nearestState(node->left, d + 1, mindist, target);

      if (state2 != NULL)
      {
        double dist2 = distance(state2, target);
        if (dist2 < mindist)
        {
          mindist = dist2;
          best = state2;
        }
      }
    }
  }
  return best;
#endif
}

void RRT::addNode(KDNode** node, int d, State *state)
{
#ifndef USE_KDTREE
  states_vec.push_back(state);
#else
  if (*node == NULL)
  {
    KDNode *newnode = new KDNode;
    newnode->state = state;
    newnode->left = NULL;
    newnode->right = NULL;
    *node = newnode;
    return;
  }

  if (d > 1)
    d = 0;
  if ((d == 0 && state->position.x < (*node)->state->position.x)
      || (d == 1 && state->position.y < (*node)->state->position.y))
    addNode(&((*node)->left), d + 1, state);
  else
    addNode(&((*node)->right), d + 1, state);
#endif
}

void RRT::deleteTree(KDNode* node)
{
#ifndef USE_KDTREE
  states_vec.clear();
  return;
#else
  if (node == NULL)
    return;
  deleteTree(node->left);
  deleteTree(node->right);
  delete node->state;
  delete node;
  return;
#endif
}

} // namespace RRT
