#ifndef TEST_FUNCTIONS_H
#define TEST_FUNCTIONS_H

#include <vector>
#include "waypoint.hpp"

bool expectNearDoubleVector(std::vector<std::vector<double>> vec1, std::vector<std::vector<double>> vec2, double threshold);
bool expectNearSingleVector(std::vector<double> vec1, std::vector<double> vec2, double threshold);
bool expectNearWaypoint(Waypoint waypoint1, Waypoint waypoint2, double threshold);

#endif // TEST_FUNCTIONS_H
