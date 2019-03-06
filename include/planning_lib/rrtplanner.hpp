#ifndef RRTPLANNER_HPP
#define RRTPLANNER_HPP

#include "obstaclemap.hpp"
#include "rrtnode.hpp"
#include "waypoint.hpp"
#include <random>
#include <vector>
#include <math.h>

class Settings;
class State;

class RRTPlanner
{
public:
  RRTPlanner();
  RRTPlanner(Settings *settings, State *state);
  void initializeTree();
  bool expandTreeToGoal();
  bool checkIfGoalIsValid();
  bool checkIfCurrentLocationIsValid();
  std::vector<std::vector<double>> getSmoothedPath();
  std::vector<int> findPathToGoal();
  std::vector<int> smoothPath(std::vector<int> path);
  bool checkPathForCollisions(const std::vector<Waypoint> &waypoints);
  void updateObstaclesNED();

protected:
  bool checkForLineCollision(std::vector<double> point1, std::vector<double> point2);
  bool obstacleInRange(std::vector<double> obstacle, std::vector<double> point1, std::vector<double> point2);
  void updateBoundaries();
  std::vector<double> sampleNextPoint();
  double generateRandomDouble(double min_value, double max_value);
  RRTNode generateNewNode();
  unsigned int getNearestNodeIndex(std::vector<double> point);
  bool checkForPointCollision(std::vector<double> point);
  void expandTreeOneTime();

  std::vector<std::vector<double>> m_obstacles_ned{{}};
  Settings *m_settings;
  State *m_state;
  double m_min_x{0};
  double m_max_x{0};
  double m_min_y{0};
  double m_max_y{0};
  std::vector<RRTNode> m_tree;
  std::random_device random_32bit;
  std::mt19937 generator;
};

#endif // RRTPLANNER_HPP
