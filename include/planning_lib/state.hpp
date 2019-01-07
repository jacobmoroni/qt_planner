#ifndef STATE_HPP
#define STATE_HPP

#include "obstaclemap.hpp"

struct Position
{
    Position() {}
    double north{0};
    double east{0};
    double down{0};
    double yaw{0};
};

struct GoalPoint
{
    GoalPoint() {}
    double north{0};
    double east{0};
    double down{0};
    double yaw{0};
};

struct State {
  State() {}
  ~State()
  {
    delete position;
    delete obstacles;
    delete goal_point;
  }

  Position *position{new Position};
  ObstacleMap *obstacles{new ObstacleMap};
  GoalPoint *goal_point{new GoalPoint};
};

#endif // STATE_HPP
