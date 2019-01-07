#ifndef WAYPOINTMANAGER_HPP
#define WAYPOINTMANAGER_HPP

#include <vector>
#include <QObject>
#include "waypoint.hpp"

class Settings;
class State;

class WaypointManager : public QObject
{
  Q_OBJECT

public:
  WaypointManager();
  WaypointManager(Settings* settings, State* state);
  ~WaypointManager();

  void generateWaypointsFromPath(std::vector<std::vector<double>> &path);
  void updateRobotYaw(double new_yaw);
  void updateRobotNorth(double new_north);
  void updateRobotEast(double new_east);
  void updateRobotDown(double new_down);
  Waypoint getCurrentWaypoint();
  void clearWaypoints();
  std::vector<Waypoint> getWaypoints();
  void setDefaultWaypoint(Waypoint new_waypoint);

signals:
  void reachedWaypoint();
  void goalReached();

protected:
  double calculateWaypointYaw(std::vector<double> point1, std::vector<double> point2);
  bool checkWaypointThreshold(Waypoint waypoint);
  Settings* m_settings;
  State* m_state;
  std::vector<Waypoint> m_waypoints{};
  Waypoint m_default_waypoint{0, 0, -1.3, 0, .5};
};

#endif // WAYPOINTMANAGER_HPP
