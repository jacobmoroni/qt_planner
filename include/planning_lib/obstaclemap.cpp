#include "obstaclemap.hpp"
#include "transforms2d.hpp"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <math.h>

ObstacleMap::ObstacleMap(){}

void ObstacleMap::setValues(const nav_msgs::OccupancyGrid &obstacles)
{
  m_width = obstacles.info.width;
  m_height = obstacles.info.height;
  m_resolution = obstacles.info.resolution;
  m_position = std::vector<double>{obstacles.info.origin.position.x,
             obstacles.info.origin.position.y,
             obstacles.info.origin.position.z};
  m_position_2d =std::vector<double>{obstacles.info.origin.position.x,
      obstacles.info.origin.position.y};
  m_orientation = std::vector<double>{obstacles.info.origin.orientation.x,
                obstacles.info.origin.orientation.y,
                obstacles.info.origin.orientation.z,
                obstacles.info.origin.orientation.w};
  m_yaw = getYawFromQuaternion();
  m_obstacle_data = obstacles.data;
  setObstacleLocations();
}

void ObstacleMap::setValues(int width, int height, double resolution,
                            std::vector<double> position,
                            std::vector<double> orientation,
                            std::vector<signed char> obstacle_data)
{
  m_width = width;
  m_height = height;
  m_resolution = resolution;
  m_position = position;
  m_orientation = orientation;
  m_obstacle_data = obstacle_data;
  setObstacleLocations();
}

std::vector<std::vector<signed char>> ObstacleMap::get2DGridMap()
{
  std::vector<signed char> grid_map_1d{m_obstacle_data};
  std::vector<std::vector<signed char>> grid_map_2d;
  grid_map_2d.resize(m_height);
  if (m_height > 0)
  {
    for (int i{0}; i < m_height; i++)
    {
        grid_map_2d[i].resize(m_width);
    }
    for (int i{0}; i < grid_map_1d.size(); i++)
    {
        int row = i / m_width;
        int col = i %m_width;
        grid_map_2d[row][col] = grid_map_1d[i];
    }
  }
  return grid_map_2d;
}

std::vector<std::vector<int>> ObstacleMap::getObstacleIndicies()
{
  signed char confidence_thresh{90};
  std::vector<std::vector<signed char>> grid_2d{get2DGridMap()};
  std::vector<std::vector<int>> obstacle_locations{};
  for (int row{0}; row<this->m_height; row++)
  {
    for (int column{0}; column<this->m_width; column++)
    {
      if (m_unknown_as_obstacles == true)
      {
        if (grid_2d[row][column]!=0)
        {
          obstacle_locations.push_back(std::vector<int>{(int) row,(int) column});
        }
      }
      else
      {
        if (grid_2d[row][column]>confidence_thresh)
        {
          obstacle_locations.push_back(std::vector<int>{(int) row,(int) column});
        }
      }
    }
  }
  return obstacle_locations;
}

void ObstacleMap::setObstacleLocations()
{
  std::vector<std::vector<double>> locations{};
  std::vector<std::vector<double>> double_indicies{convertIntVector2DoubleVector(getObstacleIndicies())};
  locations = scaleVector(double_indicies,m_resolution);
  locations = translateVector(locations,convertPointNWU2Grid(m_position_2d));
  locations = rotateVector(locations, m_yaw);
  m_obstacle_locations = locations;
}

std::vector<std::vector<double>> ObstacleMap::getObstacleLocations()
{
  return m_obstacle_locations;
}

double ObstacleMap::getYawFromQuaternion()
{
  double siny_cosp = +2.0 * (m_orientation[3] * m_orientation[2] + m_orientation[0] * m_orientation[1]);
  double cosy_cosp = +1.0 - 2.0 * (m_orientation[1] * m_orientation[1] + m_orientation[2] * m_orientation[2]);
  double yaw = atan2(siny_cosp, cosy_cosp);
  yaw = yaw*180/PI;
  return yaw;
}

void ObstacleMap::setUnknownAsObstacles(bool new_value)
{
  m_unknown_as_obstacles = new_value;
}

std::vector<double> ObstacleMap::get2DPosition()
{
  return this->m_position_2d;
}
