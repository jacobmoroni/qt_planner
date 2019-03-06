#ifndef OBSTACLEMAP_HPP
#define OBSTACLEMAP_HPP

#include <vector>
#include <nav_msgs/OccupancyGrid.h>

class ObstacleMap
{
public:
  ObstacleMap();
  void setValues(const nav_msgs::OccupancyGrid &obstacles);
  void setValues(unsigned int width, unsigned int height, float resolution,
                              std::vector<double> position,
                              std::vector<double> orientation,
                              std::vector<signed char> obstacle_data);
  std::vector<std::vector<signed char>> get2DGridMap();
  std::vector<double> get2DPosition();
  std::vector<std::vector<int>> getObstacleIndicies();
  void setObstacleLocations();
  std::vector<std::vector<double>> getObstacleLocations();
  double getYawFromQuaternion();
  void setUnknownAsObstacles(bool new_value);

private:
  unsigned int m_width{0};
  unsigned int m_height{0};
  float m_resolution{0.5};
  std::vector<double> m_position{0,0,0};
  std::vector<double> m_position_2d{0,0};
  std::vector<double> m_orientation{0,0,0,1};
  double m_yaw{0};
  std::vector<signed char> m_obstacle_data{0};
  std::vector<std::vector<double>> m_obstacle_locations{{0}};
  bool m_unknown_as_obstacles{false};
};

#endif // OBSTACLEMAP_HPP
