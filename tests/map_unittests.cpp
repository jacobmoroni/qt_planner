#include <gtest/gtest.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include "obstaclemap.hpp"
#include "test_functions.h"
#include <vector>
#include <tuple>

class TestObstacleMap : public ::testing::Test
{
public:
  nav_msgs::OccupancyGrid test_grid_msg;
  ObstacleMap *test_map = new ObstacleMap;
  TestObstacleMap()
  {
    test_grid_msg.header.frame_id = "map";
    test_grid_msg.header.seq = 53;
    test_grid_msg.info.resolution = 0.05;
    test_grid_msg.info.width = 10;
    test_grid_msg.info.height = 20;
    test_grid_msg.info.origin.position.x = 1;
    test_grid_msg.info.origin.position.y = 2;
    test_grid_msg.info.origin.position.z = 0;
    test_grid_msg.info.origin.orientation.x = 0;
    test_grid_msg.info.origin.orientation.y = 0;
    test_grid_msg.info.origin.orientation.z = 0;
    test_grid_msg.info.origin.orientation.w = 1;
    test_grid_msg.data = std::vector<signed char>{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, 100, 100, 100, 100, -1, -1, -1, -1, -1,
        -1, 0, 0, 0, 0, 100, 0, 0, 0, -1,
        -1, 0, 0, 0, 0, 100, 0, 0, 0, -1,
        -1, 0, 0, 0, 0, 100, 0, 0, 0, -1,
        -1, 0, 0, 0, 0, 100, 0, 0, 0, -1,
        -1, 0, 0, 0, 0, 100, 0, 0, 0, -1,
        -1, 100, 100, 100, 100, 100, -1, -1, -1, -1,
        -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
        100, 0, 0, 0, 0, 0, -1, -1, -1, 100,
        100, 0, 0, 0, 0, 0, -1, -1, -1, 100,
        100, 0, 0, 0, 0, 0, -1, -1, -1, 100,
        100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

  }
  ~TestObstacleMap()
  {
    delete test_map;
    test_map = nullptr;
  }
};

TEST_F(TestObstacleMap, whenReshapingDataArray_sizeIsCorrect)
{
  test_map->setValues(test_grid_msg);
  std::vector<std::vector<signed char>> grid_map_2d{test_map->get2DGridMap()};
  int golden_width_value{10};
  int golden_height_value{20};
  EXPECT_EQ(golden_width_value,grid_map_2d[0].size());
  EXPECT_EQ(golden_height_value,grid_map_2d.size());
}

TEST_F(TestObstacleMap, whenAskedForObstaclesIndicies_OutputIsCorrect)
{
  test_map->setValues(test_grid_msg);
  std::vector<std::vector<int>> obstacles{test_map->getObstacleIndicies()};
  std::vector<int> golden_obstacle_1{1,1};
  std::vector<int> golden_obstacle_2{2,1};
  std::vector<int> golden_obstacle_last{19,9};

  int golen_number_of_obstacles{38};

  EXPECT_EQ(golden_obstacle_1, obstacles[0]);
  EXPECT_EQ(golden_obstacle_2, obstacles[1]);
  EXPECT_EQ(golden_obstacle_last, obstacles.back());
  EXPECT_EQ(golen_number_of_obstacles,obstacles.size());
}

TEST_F(TestObstacleMap, whenConvertingObstacleIndiciesToLocations_LocationsAreCorrect)
{
  test_map->setValues(test_grid_msg);
  std::vector<std::vector<double>> obstacle_locations{test_map->getObstacleLocations()};
  std::vector<std::vector<double>> golden_obstacle_locations{ { 2.05, 1.05 },
                                                              { 2.05, 1.10 },
                                                              { 2.05, 1.15 },
                                                              { 2.05, 1.20 },
                                                              { 2.05, 1.25 },
                                                              { 2.05, 1.30 },
                                                              { 2.05, 1.35 },
                                                              { 2.10, 1.35 },
                                                              { 2.15, 1.35 },
                                                              { 2.20, 1.35 },
                                                              { 2.25, 1.40 },
                                                              { 2.25, 1.45 },
                                                              { 2.25, 1.50 },
                                                              { 2.25, 1.55 },
                                                              { 2.25, 1.60 },
                                                              { 2.05, 1.65 },
                                                              { 2.10, 1.65 },
                                                              { 2.15, 1.65 },
                                                              { 2.20, 1.65 },
                                                              { 2.25, 1.65 },
                                                              { 2.05, 1.70 },
                                                              { 2.05, 1.75 },
                                                              { 2.00, 1.80 },
                                                              { 2.45, 1.80 },
                                                              { 2.00, 1.85 },
                                                              { 2.45, 1.85 },
                                                              { 2.00, 1.90 },
                                                              { 2.45, 1.90 },
                                                              { 2.00, 1.95 },
                                                              { 2.05, 1.95 },
                                                              { 2.10, 1.95 },
                                                              { 2.15, 1.95 },
                                                              { 2.20, 1.95 },
                                                              { 2.25, 1.95 },
                                                              { 2.30, 1.95 },
                                                              { 2.35, 1.95 },
                                                              { 2.40, 1.95 },
                                                              { 2.45, 1.95 }};
  EXPECT_TRUE(expectNearDoubleVector(golden_obstacle_locations, obstacle_locations,0.0001));
}

TEST_F(TestObstacleMap, whenExtractingYawFromQuaternion_YawIsCorrect)
{
  double yaw1 = test_map->getYawFromQuaternion();
  double golden_yaw1{0};

  test_grid_msg.info.origin.orientation.x = 0;
  test_grid_msg.info.origin.orientation.y = 0;
  test_grid_msg.info.origin.orientation.z = 1;
  test_grid_msg.info.origin.orientation.w = 0;
  test_map->setValues(test_grid_msg);
  double yaw2 = test_map->getYawFromQuaternion();
  double golden_yaw2{180};

  test_grid_msg.info.origin.orientation.x = 0;
  test_grid_msg.info.origin.orientation.y = 0;
  test_grid_msg.info.origin.orientation.z = 0.13052619;
  test_grid_msg.info.origin.orientation.w = 0.99144486;
  test_map->setValues(test_grid_msg);
  double yaw3 = test_map->getYawFromQuaternion();
  double golden_yaw3{15};

  EXPECT_NEAR(golden_yaw1,yaw1,0.00001);
  EXPECT_NEAR(golden_yaw2,yaw2,0.00001);
  EXPECT_NEAR(golden_yaw3,yaw3,0.00001);
}

