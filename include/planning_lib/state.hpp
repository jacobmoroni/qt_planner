#ifndef STATE_HPP
#define STATE_HPP

#include "obstaclemap.hpp"
#include "waypoint.hpp"

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
    delete current_waypoint;
//    delete waypoints;
  }

  Position *position{new Position};
  ObstacleMap *obstacles{new ObstacleMap};
  GoalPoint *goal_point{new GoalPoint};
  Waypoint *current_waypoint{new Waypoint};
  std::vector<std::vector<double>> waypoint_list;
};

#endif // STATE_HPP
