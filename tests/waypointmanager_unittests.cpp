#include <gtest/gtest.h>
#include "test_functions.h"
#include "waypointmanager.hpp"
#include "settings.hpp"
#include "state.hpp"
#include "waypoint.hpp"

class WaypointManagerTest : public WaypointManager, public ::testing::Test
{
public:
  std::vector<std::vector<double>> path{{0,0},{1,5},{7,5},{7,-3},{-5,-5}};
  WaypointManagerTest()
  {
    m_settings = new Settings;
    m_state = new State;
    m_state->goal_point->down = -1.4;
    m_state->goal_point->yaw = 0;
  }
  ~WaypointManagerTest()
  {
    delete m_settings;
    delete m_state;
  }
};

TEST_F(WaypointManagerTest, whenComputingYawFromPath_YawIsCorrect)
{
  double yaw1{calculateWaypointYaw(path[0], path[1])};
  double golden_yaw1{1.3734};
  double yaw2{calculateWaypointYaw(path[1], path[2])};
  double golden_yaw2{0};
  double yaw3{calculateWaypointYaw(path[2], path[3])};
  double golden_yaw3{-1.5708};
  double yaw4{calculateWaypointYaw(path[3], path[4])};
  double golden_yaw4{-2.97644};


  EXPECT_NEAR(golden_yaw1, yaw1,0.0001);
  EXPECT_NEAR(golden_yaw2, yaw2,0.0001);
  EXPECT_NEAR(golden_yaw3, yaw3,0.0001);
  EXPECT_NEAR(golden_yaw4, yaw4,0.0001);
}

TEST_F(WaypointManagerTest, whenAddingWaypointsFromPath_AddedWaypointsAreCorrect)
{
  generateWaypointsFromPath(path);
  Waypoint golden_waypoint1{1,5,m_state->goal_point->down,1.3734,m_settings->waypoint_manager->position_threshold};
  Waypoint golden_waypoint2{7,5,m_state->goal_point->down,0,m_settings->waypoint_manager->position_threshold};
  Waypoint golden_waypoint3{7,-3,m_state->goal_point->down,-1.5708,m_settings->waypoint_manager->position_threshold};
  Waypoint golden_waypoint4{-5,-5,m_state->goal_point->down,-2.97644,m_settings->waypoint_manager->position_threshold};
  Waypoint golden_waypoint5{-5,-5,m_state->goal_point->down,0,m_settings->waypoint_manager->position_threshold};

  std::vector<Waypoint> golden_waypoints{golden_waypoint1, golden_waypoint2, golden_waypoint3, golden_waypoint4, golden_waypoint5};
  for (int i{0}; i<golden_waypoints.size(); i++)
  {
    EXPECT_TRUE(expectNearWaypoint(golden_waypoints[i], m_waypoints[i], 0.0001));
  }
}

TEST_F(WaypointManagerTest, whenCheckingWaypointThresholdBeforeThresholdIsReached_ReturnsFalse)
{
  generateWaypointsFromPath(path);

  m_state->position->north = 0;
  m_state->position->east = 0;
  m_state->position->down = 0;
  m_state->position->yaw = 0;
  bool threshold_reached1{checkWaypointThreshold(m_waypoints[0])};

  m_state->position->north = 7;
  m_state->position->east = 5;
  m_state->position->down = 0;
  m_state->position->yaw = 1.3734;
  bool threshold_reached2{checkWaypointThreshold(m_waypoints[0])};

  m_state->position->north = 1;
  m_state->position->east = 5;
  m_state->position->down = -1.4;
  m_state->position->yaw = 0;
  bool threshold_reached3{checkWaypointThreshold(m_waypoints[0])};

  EXPECT_FALSE(threshold_reached1);
  EXPECT_FALSE(threshold_reached2);
  EXPECT_FALSE(threshold_reached3);
}

TEST_F(WaypointManagerTest, whenCheckingWaypointThresholdWhenThresholdIsReached_ReturnsTrue)
{
  generateWaypointsFromPath(path);

  m_state->position->north = 1;
  m_state->position->east = 5;
  m_state->position->down = m_state->goal_point->down;
  m_state->position->yaw = 1.3734;
  bool threshold_reached1{checkWaypointThreshold(m_waypoints[0])};

  m_state->position->north = 7;
  m_state->position->east = 4.81;
  m_state->position->down = m_state->goal_point->down;
  m_state->position->yaw = 0;
  bool threshold_reached2{checkWaypointThreshold(m_waypoints[1])};

  m_state->position->north = 7;
  m_state->position->east = -3;
  m_state->position->down = m_state->goal_point->down-.2;
  m_state->position->yaw = -1.5;
  bool threshold_reached3{checkWaypointThreshold(m_waypoints[2])};

  EXPECT_TRUE(threshold_reached1);
  EXPECT_TRUE(threshold_reached2);
  EXPECT_TRUE(threshold_reached3);
}

TEST_F(WaypointManagerTest, whenWaypointThresholdIsReached_WaypointIsRemoved)
{
  generateWaypointsFromPath(path);
  m_state->position->north = 0;
  m_state->position->east = 0;
  m_state->position->down = 0;
  m_state->position->yaw = 0;
  Waypoint test_waypoint1{getCurrentWaypoint()};
  Waypoint golden_waypoint1{1, 5, m_state->goal_point->down, 1.3734, m_settings->waypoint_manager->position_threshold};
  int waypoint_num1{(int) m_waypoints.size()};
  int golden_waypoint_num1{5};

  m_state->position->north = 1;
  m_state->position->east = 5;
  m_state->position->down = m_state->goal_point->down;
  m_state->position->yaw = 1.3734;
  Waypoint test_waypoint2{getCurrentWaypoint()};
  Waypoint golden_waypoint2{7, 5, m_state->goal_point->down, 0, m_settings->waypoint_manager->position_threshold};
  int waypoint_num2{(int) m_waypoints.size()};
  int golden_waypoint_num2{4};

  EXPECT_TRUE(expectNearWaypoint(test_waypoint1, golden_waypoint1, 0.0001));
  EXPECT_EQ(golden_waypoint_num1, waypoint_num1);
  EXPECT_TRUE(expectNearWaypoint(test_waypoint2, golden_waypoint2, 0.0001));
  EXPECT_EQ(golden_waypoint_num2, waypoint_num2);
}

TEST_F(WaypointManagerTest, whenWaypointVectorIsEmpty_CurrentWaypointDoesntBreak)
{
  m_default_waypoint.setWaypoint(1,5,m_state->goal_point->down,-1.3,m_settings->waypoint_manager->position_threshold);
  Waypoint default_waypoint{getCurrentWaypoint()};
  Waypoint golden_waypoint{1,5,m_state->goal_point->down,-1.3, m_settings->waypoint_manager->position_threshold};

  EXPECT_TRUE(expectNearWaypoint(golden_waypoint,default_waypoint,0.0001));
}
