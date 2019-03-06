#include "test_functions.h"
#include <vector>
#include <cstdlib>
#include "waypoint.hpp"

bool expectNearDoubleVector(std::vector<std::vector<double>> vec1, std::vector<std::vector<double>> vec2, double threshold)
{
  for (unsigned int i{0}; i<static_cast<unsigned int>(vec1.size()); i++)
  {
    if (std::abs(vec1[i][0]-vec2[i][0]) > threshold || std::abs(vec1[i][1]-vec2[i][1]) > threshold)
      return false;
  }
  return true;
}

bool expectNearSingleVector(std::vector<double> vec1, std::vector<double> vec2, double threshold)
{
  for (unsigned int i{0}; i<static_cast<unsigned int>(vec1.size()); i++)
  {
    if (std::abs(vec1[0]-vec2[0]) > threshold || std::abs(vec1[1]-vec2[1]) > threshold)
      return false;
  }
  return true;
}

bool expectNearWaypoint(Waypoint waypoint1, Waypoint waypoint2, double threshold)
{
  std::vector<double> waypoint1vec{waypoint1.getNorth(),waypoint1.getEast(), waypoint1.getDown(), waypoint1.getYaw(), waypoint1.getThreshold()};
  std::vector<double> waypoint2vec{waypoint2.getNorth(),waypoint2.getEast(), waypoint2.getDown(), waypoint2.getYaw(), waypoint2.getThreshold()};
  return expectNearSingleVector(waypoint1vec,waypoint2vec, threshold);
}
