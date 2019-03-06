#include "rrtplanner.hpp"
#include "transforms2d.hpp"
#include "obstaclemap.hpp"
#include "rrtnode.hpp"
#include "settings.hpp"
#include "state.hpp"
#include <cmath>

RRTPlanner::RRTPlanner(){}

RRTPlanner::RRTPlanner(Settings *settings, State *state):
  m_settings{settings},
  m_state{state}
{
  generator.seed(random_32bit());
}

void RRTPlanner::updateObstaclesNED()
{
  std::vector<std::vector<double>> obstacles_grid{m_state->obstacles->getObstacleLocations()};
  std::vector<std::vector<double>> obstacles_pixel{convertVectorGrid2Pixel(obstacles_grid)};
  m_obstacles_ned = convertVectorPixel2NED(obstacles_pixel);
}

void RRTPlanner::updateBoundaries()
{
  std::vector<double> x_minmax{getXMinMax(m_obstacles_ned)};
  std::vector<double> y_minmax{getYMinMax(m_obstacles_ned)};
  m_min_x = x_minmax[0];
  m_max_x = x_minmax[1];
  m_min_y = y_minmax[0];
  m_max_y = y_minmax[1];
}

bool RRTPlanner::checkForLineCollision(std::vector<double> point1, std::vector<double> point2)
{
  double y_diff{point2[1]-point1[1]};
  double x_diff{point2[0]-point1[0]};
  double slope_diff{point2[0]*point1[1] - point2[1]*point1[0]};
  double distance{0};
  bool collision{false};

  for (std::vector<double> obstacle : m_obstacles_ned)
  {
    if (obstacleInRange(obstacle,point1,point2))
    {
      distance = fabs(y_diff*obstacle[0]-x_diff*obstacle[1]+slope_diff)/sqrt(y_diff * y_diff + x_diff * x_diff);
      if (distance <= m_settings->obstacle->buffer_size)
      {
        collision = true;
        break;
      }
    }
  }
  return collision;
}

bool RRTPlanner::obstacleInRange(std::vector<double> obstacle, std::vector<double> point1, std::vector<double> point2)
{
  if (((obstacle[0] + m_settings->obstacle->buffer_size) <= point2[0] && (obstacle[0] + m_settings->obstacle->buffer_size) <= point1[0]) ||
      ((obstacle[0] - m_settings->obstacle->buffer_size) >= point2[0] && (obstacle[0] - m_settings->obstacle->buffer_size) >= point1[0]))
  {
    return false;
  }
  else if (((obstacle[1] + m_settings->obstacle->buffer_size) <= point2[1] && (obstacle[1] + m_settings->obstacle->buffer_size) <= point1[1]) ||
           ((obstacle[1] - m_settings->obstacle->buffer_size) >= point2[1] && (obstacle[1] - m_settings->obstacle->buffer_size) >= point1[1]))
  {
    return false;
  }
  else
  {
    return true;
  }
}

double RRTPlanner::generateRandomDouble(double min_value, double max_value)
{
  std::uniform_real_distribution<> distribution{min_value, max_value};
  return distribution(generator);
}

std::vector<double> RRTPlanner::sampleNextPoint()
{
  std::vector<double> next_point{};
  double random_number{generateRandomDouble(0,100)};
  if (random_number < m_settings->rrt->goal_sample_rate)
  {
    next_point = std::vector<double>{m_state->goal_point->north, m_state->goal_point->east};
  }
  else
  {
    next_point = {generateRandomDouble(m_min_x-m_settings->rrt->boundary_buffer, m_max_x+m_settings->rrt->boundary_buffer),
                  generateRandomDouble(m_min_y-m_settings->rrt->boundary_buffer,m_max_y+m_settings->rrt->boundary_buffer )};
  }
  return next_point;
}

void RRTPlanner::initializeTree()
{
  RRTNode start{m_state->position->north, m_state->position->east, -1};
  m_tree = {};
  m_tree.push_back(start);
  updateObstaclesNED();
  updateBoundaries();
}

unsigned int RRTPlanner::getNearestNodeIndex(std::vector<double> point)
{
  double distance_min{10000};
  double distance{0};
  unsigned int nearest_index{0};
  for (unsigned int i{0}; i<static_cast<unsigned int>(m_tree.size()); i++)
  {
   distance = pow((m_tree[i].getX()-point[0]),2) + pow((m_tree[i].getY()-point[1]),2);
   if (distance < distance_min)
   {
     distance_min = distance;
     nearest_index = i;
   }
  }
  return nearest_index;
}

RRTNode RRTPlanner::generateNewNode()
{
  std::vector<double> new_point{sampleNextPoint()};
  unsigned int nearest_node_index{getNearestNodeIndex(new_point)};
  double atan_y{new_point[1]-m_tree[nearest_node_index].getY()};
  double atan_x{new_point[0]-m_tree[nearest_node_index].getX()};
  double angle_to_point{atan2(atan_y, atan_x)};
  RRTNode new_node{m_tree[nearest_node_index]};

  new_node.setX(new_node.getX()+m_settings->rrt->expand_distance*cos(angle_to_point));
  new_node.setY(new_node.getY()+m_settings->rrt->expand_distance*sin(angle_to_point));
  new_node.setParent(static_cast<int>(nearest_node_index));

  return new_node;
}

bool RRTPlanner::checkForPointCollision(std::vector<double> point)
{
  double distance{0};
  bool collision{false};
  for (std::vector<double> obstacle : m_obstacles_ned)
  {
    distance = sqrt(pow(obstacle[0]-point[0],2)+pow(obstacle[1]-point[1],2));
    if (distance < m_settings->obstacle->buffer_size)
    {
      collision = true;
      break;
    }
  }
  return collision;
}

void RRTPlanner::expandTreeOneTime()
{
  RRTNode new_node{generateNewNode()};
  std::vector<double> new_point{new_node.getX(), new_node.getY()};
  std::vector<double> parent_point{m_tree[static_cast<unsigned int>(new_node.getParent())].getX(), m_tree[static_cast<unsigned int>(new_node.getParent())].getY()};
  if (!checkForLineCollision(new_point, parent_point))
  {
    m_tree.push_back(new_node);
  }
}

bool RRTPlanner::checkIfGoalIsValid()
{
  if (checkForPointCollision(std::vector<double>{m_state->goal_point->north, m_state->goal_point->east}))
    return false;
  else
    return true;
}

bool RRTPlanner::checkIfCurrentLocationIsValid()
{
  if (checkForPointCollision(std::vector<double>{m_state->position->north, m_state->position->east}))
    return false;
  else
    return true;
}

bool RRTPlanner::expandTreeToGoal()
{
  RRTNode node{m_tree[0]};
  int counter{0};

  while (checkForLineCollision(std::vector<double>{node.getX(),
                               node.getY()},
                               std::vector<double>{m_state->goal_point->north, m_state->goal_point->east}) &&
         counter < m_settings->rrt->timeout)
  {
    expandTreeOneTime();
    node = m_tree.back();
    counter++;
  }
  m_tree.push_back(RRTNode{m_state->goal_point->north, m_state->goal_point->east, static_cast<int>(m_tree.size()-1)});
  if (counter == m_settings->rrt->timeout)
    return false;
  else
    return true;
}

std::vector<int> RRTPlanner::findPathToGoal()
{
  int current_node_idx{static_cast<int>(m_tree.size()-1)};
  std::vector<int> path_to_goal{current_node_idx};
  while (current_node_idx!=0)
  {
    current_node_idx = m_tree[static_cast<unsigned int>(current_node_idx)].getParent();
    path_to_goal.insert(path_to_goal.begin(),current_node_idx);
  }
  return path_to_goal;
}

std::vector<int> RRTPlanner::smoothPath(std::vector<int> path)
{
  unsigned int current_index{0};
  std::vector<unsigned int> path2(path.begin(), path.end());
  std::vector<int> smoothed_path{static_cast<int>(current_index)};
  while (smoothed_path.back() != path.back())
  {
    for (unsigned int i{static_cast<unsigned int>(path.size()-1)}; i>current_index; i--)
    {
      RRTNode current_node{m_tree[path2[current_index]]};
      RRTNode target_node{m_tree[path2[i]]};
      if (checkForLineCollision(std::vector<double>{current_node.getX(),current_node.getY()},
                                std::vector<double>{target_node.getX(),target_node.getY()}) == false)
      {
        current_index = i;
        smoothed_path.push_back(path[static_cast<unsigned int>(current_index)]);
        break;
      }
    }
  }
  return smoothed_path;
}

std::vector<std::vector<double>> RRTPlanner::getSmoothedPath()
{
  std::vector<int> path_indicies{findPathToGoal()};
  path_indicies = smoothPath(path_indicies);
  std::vector<std::vector<double>> smoothed_path_points{};
  for (int idx: path_indicies)
  {
    smoothed_path_points.push_back(std::vector<double>{m_tree[static_cast<unsigned int>(idx)].getX(),m_tree[static_cast<unsigned int>(idx)].getY()});
  }
  return smoothed_path_points;
}

bool RRTPlanner::checkPathForCollisions(const std::vector<Waypoint> &waypoints)
{
  for (unsigned int i{0}; i<static_cast<unsigned int>(waypoints.size()-1); i++)
  {
    std::vector<double> point1{waypoints[i].getNorth(), waypoints[i].getEast()};
    std::vector<double> point2{waypoints[i+1].getNorth(), waypoints[i+1].getEast()};
    if (checkForLineCollision(point1, point2))
    {
      return true;
    }
  }
  return false;
}
