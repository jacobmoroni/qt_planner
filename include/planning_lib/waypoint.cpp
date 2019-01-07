#include "waypoint.hpp"

Waypoint::Waypoint()
{
}

Waypoint::Waypoint(double north, double east, double down, double yaw, double threshold):
  m_north{north}, m_east{east}, m_down{down}, m_yaw{yaw}, m_threshold{threshold}
{
}

Waypoint::~Waypoint()
{
}

double Waypoint::getNorth() const
{
  return m_north;
}

void Waypoint::setNorth(double north)
{
  m_north = north;
}

double Waypoint::getEast() const
{
  return m_east;
}

void Waypoint::setEast(double east)
{
  m_east = east;
}

double Waypoint::getDown() const
{
  return m_down;
}

void Waypoint::setDown(double down)
{
  m_down = down;
}

double Waypoint::getYaw() const
{
  return m_yaw;
}

void Waypoint::setYaw(double yaw)
{
  m_yaw = yaw;
}

double Waypoint::getThreshold() const
{
  return m_threshold;
}

void Waypoint::setThreshold(double threshold)
{
  m_threshold = threshold;
}

void Waypoint::setWaypoint(double north, double east, double down, double yaw, double threshold)
{
    m_north = north;
    m_east = east;
    m_down = down;
    m_yaw = yaw;
    m_threshold = threshold;
}
