#include "waypointmanager.hpp"
#include "waypoint.hpp"
#include "settings.hpp"
#include "state.hpp"
#include <tgmath.h>

WaypointManager::WaypointManager(){}
WaypointManager::WaypointManager(Settings* settings, State *state):
  m_settings{settings},
  m_state{state}
{
  m_state->current_waypoint = &m_default_waypoint;
}

WaypointManager::~WaypointManager()
{
}

void WaypointManager::generateWaypointsFromPath(std::vector<std::vector<double> > &path)
{
  Waypoint waypoint;
  for (unsigned int i{1}; i<path.size(); i++)
  {
    waypoint.setNorth(path[i][0]);
    waypoint.setEast(path[i][1]);
    waypoint.setDown(m_state->goal_point->down);
    waypoint.setYaw(calculateWaypointYaw(path[i-1], path[i]));
    waypoint.setThreshold(m_settings->waypoint_manager->position_threshold);
    m_waypoints.push_back(waypoint);
  }
  waypoint.setWaypoint(path.back()[0], path.back()[1], m_state->goal_point->down, m_state->goal_point->yaw, m_settings->waypoint_manager->position_threshold);
  m_waypoints.push_back(waypoint);
}

double WaypointManager::calculateWaypointYaw(std::vector<double> point1, std::vector<double> point2)
{
  double x_diff{point2[0]-point1[0]};
  double y_diff{point2[1]-point1[1]};
  return atan2(y_diff, x_diff);
}

Waypoint WaypointManager::getCurrentWaypoint()
{
  if (m_waypoints.size()>0)
  {
    if (checkWaypointThreshold(m_waypoints[0]))
    {
      if (m_waypoints.size()>0)
      {
        m_default_waypoint = m_waypoints[0];
        m_waypoints.erase(m_waypoints.begin());
        emit reachedWaypoint();
      }
    }
    return m_waypoints[0];
  }
  else
  {
    emit goalReached();
    return m_default_waypoint;
  }
}

bool WaypointManager::checkWaypointThreshold(Waypoint waypoint)
{
  double x_diff{waypoint.getNorth()-m_state->position->north};
  double y_diff{waypoint.getEast()-m_state->position->east};
  double z_diff{waypoint.getDown()-m_state->position->down};
  double yaw_diff{waypoint.getYaw()-m_state->position->yaw};
  double norm{100};
  if (m_settings->waypoint_manager->threshold_2d)
  {
    norm = sqrt(x_diff*x_diff + y_diff * y_diff);
  }
  else
  {
    norm = sqrt(x_diff*x_diff + y_diff * y_diff + z_diff * z_diff);
  }
  if (norm < waypoint.getThreshold())
  {
    if (yaw_diff<m_settings->waypoint_manager->yaw_threshold)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

void WaypointManager::clearWaypoints()
{
  m_waypoints.clear();
}

std::vector<Waypoint> WaypointManager::getWaypoints()
{
  if (m_waypoints.size()>0)
  {
    return m_waypoints;
  }
  else return std::vector<Waypoint>{m_default_waypoint};
}

void WaypointManager::setDefaultWaypoint(Waypoint new_waypoint)
{
  m_default_waypoint = new_waypoint;
}
