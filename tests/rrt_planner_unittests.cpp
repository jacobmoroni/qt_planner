#include <gtest/gtest.h>
#include "rrtplanner.hpp"
#include "obstaclemap.hpp"
#include "test_functions.h"
#include "rrtnode.hpp"
#include "waypoint.hpp"
#include "settings.hpp"
#include "state.hpp"
#include <nav_msgs/OccupancyGrid.h>

class TestRRTPlanner : public RRTPlanner, public ::testing::Test
{
public:
  nav_msgs::OccupancyGrid test_grid_msg;
  void setup_grid_msg(nav_msgs::OccupancyGrid &test_grid_msg)
  {
    test_grid_msg.header.frame_id = "map";
    test_grid_msg.header.seq = 53;
    test_grid_msg.info.resolution = 0.5;
    test_grid_msg.info.width = 10;
    test_grid_msg.info.height = 20;
    test_grid_msg.info.origin.position.x = -5;
    test_grid_msg.info.origin.position.y = 0.5;
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
  TestRRTPlanner()
  {
    m_settings = new Settings;
    m_state = new State;
    m_settings->obstacle->buffer_size = 0.8;
    setup_grid_msg(test_grid_msg);
    m_state->obstacles->setValues(test_grid_msg);
    m_state->goal_point->north = 2;
    m_state->goal_point->east = 2;
    m_settings->rrt->boundary_buffer = 0;
    updateObstaclesNED();
    updateBoundaries();
    generator.seed(0);
  }

  ~TestRRTPlanner()
  {
    delete m_settings;
    delete m_state;
  }
};

TEST_F(TestRRTPlanner, whenCheckingForCollisionBetween2Points_collisionDetectionIsCorrect)
{
  std::vector<double> collision_point1{-6,-11};
  std::vector<double> collision_point2{-6,-0.5};
  std::vector<double> no_collision_point1{-1,-7};
  std::vector<double> no_collision_point2{-3,-2};
  bool collision1_detected{checkForLineCollision(std::vector<double>{m_state->position->north, m_state->position->east},
                                                 collision_point1)};
  bool collision2_detected{checkForLineCollision(std::vector<double>{m_state->position->north, m_state->position->east},
                                                 collision_point2)};
  bool collision3_detected{checkForLineCollision(std::vector<double>{m_state->position->north, m_state->position->east},
                                                 no_collision_point1)};
  bool collision4_detected{checkForLineCollision(std::vector<double>{m_state->position->north, m_state->position->east},
                                                 no_collision_point2)};

  EXPECT_TRUE(collision1_detected);
  EXPECT_TRUE(collision2_detected);
  EXPECT_FALSE(collision3_detected);
  EXPECT_FALSE(collision4_detected);
}

TEST_F(TestRRTPlanner, whenCheckingIfInRangeObstacleIsInRange_ReturnsTrue)
{
  std::vector<double> obstacle1{1,1};
  std::vector<double> obstacle2{-.7,0};
  std::vector<double> obstacle3{0,-.7};
  std::vector<double> obstacle4{2.7,2};
  std::vector<double> obstacle5{2,2.7};

  bool in_range1{obstacleInRange(obstacle1, std::vector<double>{m_state->position->north, m_state->position->east},
                                 std::vector<double>{m_state->goal_point->north,m_state->goal_point->east})};
  bool in_range2{obstacleInRange(obstacle2, std::vector<double>{m_state->position->north, m_state->position->east},
                                 std::vector<double>{m_state->goal_point->north,m_state->goal_point->east})};
  bool in_range3{obstacleInRange(obstacle3, std::vector<double>{m_state->position->north, m_state->position->east},
                                 std::vector<double>{m_state->goal_point->north,m_state->goal_point->east})};
  bool in_range4{obstacleInRange(obstacle4, std::vector<double>{m_state->position->north, m_state->position->east},
                                 std::vector<double>{m_state->goal_point->north,m_state->goal_point->east})};
  bool in_range5{obstacleInRange(obstacle5, std::vector<double>{m_state->position->north, m_state->position->east},
                                 std::vector<double>{m_state->goal_point->north,m_state->goal_point->east})};

  EXPECT_TRUE(in_range1);
  EXPECT_TRUE(in_range2);
  EXPECT_TRUE(in_range3);
  EXPECT_TRUE(in_range4);
  EXPECT_TRUE(in_range5);
}

TEST_F(TestRRTPlanner, whenCheckingIfOutOfRangeObstacleIsInRange_ReturnsTrue)
{
  std::vector<double> obstacle1{-1,-1};
  std::vector<double> obstacle2{-.9,0};
  std::vector<double> obstacle3{0,-.9};
  std::vector<double> obstacle4{2.9,2};
  std::vector<double> obstacle5{2,2.9};

  bool out_of_range1{obstacleInRange(obstacle1, std::vector<double>{m_state->position->north, m_state->position->east},
                                     std::vector<double>{m_state->goal_point->north,m_state->goal_point->east})};
  bool out_of_range2{obstacleInRange(obstacle2, std::vector<double>{m_state->position->north, m_state->position->east},
                                     std::vector<double>{m_state->goal_point->north,m_state->goal_point->east})};
  bool out_of_range3{obstacleInRange(obstacle3, std::vector<double>{m_state->position->north, m_state->position->east},
                                     std::vector<double>{m_state->goal_point->north,m_state->goal_point->east})};
  bool out_of_range4{obstacleInRange(obstacle4, std::vector<double>{m_state->position->north, m_state->position->east},
                                     std::vector<double>{m_state->goal_point->north,m_state->goal_point->east})};
  bool out_of_range5{obstacleInRange(obstacle5, std::vector<double>{m_state->position->north, m_state->position->east},
                                     std::vector<double>{m_state->goal_point->north,m_state->goal_point->east})};

  EXPECT_FALSE(out_of_range1);
  EXPECT_FALSE(out_of_range2);
  EXPECT_FALSE(out_of_range3);
  EXPECT_FALSE(out_of_range4);
  EXPECT_FALSE(out_of_range5);
}

TEST_F(TestRRTPlanner, whenGeneratingRandomNumberWithSeed_NumberIsAlwaysTheSame)
{
  double rand1{generateRandomDouble(0,10)};
  double golden_random{5.928446165166826};
  EXPECT_EQ(golden_random, rand1);
}

TEST_F(TestRRTPlanner, whenSamplingNewPointsWithGoalSampleRate_SampleRateWorks)
{
  m_settings->rrt->goal_sample_rate = 50;
  std::vector<double> random_point1{sampleNextPoint()};
  std::vector<double> random_point2{sampleNextPoint()};
  std::vector<double> random_point3{sampleNextPoint()};
  std::vector<double> random_point4{sampleNextPoint()};

  std::vector<double> golden_point1{-1.200804150845308, -2.278489420091531};
  std::vector<double> golden_point2{-2.193963365767512, -6.540564624636191};
  std::vector<double> golden_point3{2,2};
  std::vector<double> golden_point4{2,2};

  EXPECT_TRUE(expectNearSingleVector(golden_point1, random_point1, 0.0000001));
  EXPECT_TRUE(expectNearSingleVector(golden_point2, random_point2, 0.0000001));
  EXPECT_EQ(golden_point3, random_point3);
  EXPECT_EQ(golden_point4, random_point4);
}

TEST_F(TestRRTPlanner, whenCheckingNearestNode_NodeIsCorrect)
{
  m_state->goal_point->north = -6;
  m_state->goal_point->east = -11;

  initializeTree();
  std::vector<double> point1{sampleNextPoint()};
  sampleNextPoint();
  std::vector<double> point2{sampleNextPoint()};

  int point1_nearest_node{getNearestNodeIndex(point1)};
  int golden_nearest_node1{0};

  int point2_nearest_node{getNearestNodeIndex(point2)};
  int golden_nearest_node2{0};

  EXPECT_EQ(golden_nearest_node1, point1_nearest_node);
  EXPECT_EQ(golden_nearest_node2, point2_nearest_node);
}

TEST_F(TestRRTPlanner, whenGeneratingNewRRTNode_LocationIsCorrect)
{
  m_state->goal_point->north = -6;
  m_state->goal_point->east = -11;

  initializeTree();
  RRTNode new_node{generateNewNode()};
  RRTNode golden_node{-0.23311629392619657, -0.4423310903679676, 0};

  EXPECT_EQ(golden_node.getX(), new_node.getX());
  EXPECT_EQ(golden_node.getY(), new_node.getY());
  EXPECT_EQ(golden_node.getParent(), new_node.getParent());
}

TEST_F(TestRRTPlanner, whenCheckingPointForCollisions_CollissionsAreCorrect)
{
  std::vector<double> collision_point1{-4,-4};
  std::vector<double> collision_point2{-4,-3.5};
  std::vector<double> collision_point3{-4.5,-3.5};
  std::vector<double> non_collision_point1{-5,-5};

  bool collision1{checkForPointCollision(collision_point1)};
  bool collision2{checkForPointCollision(collision_point2)};
  bool collision3{checkForPointCollision(collision_point3)};
  bool non_collision1{checkForPointCollision(non_collision_point1)};

  EXPECT_TRUE(collision1);
  EXPECT_TRUE(collision2);
  EXPECT_TRUE(collision3);
  EXPECT_FALSE(non_collision1);
}

TEST_F(TestRRTPlanner, whenExpandingTree_TreeSizeIsCorrect)
{
  m_state->goal_point->north = -6;
  m_state->goal_point->east = -11;
  initializeTree();
  for (int i{0}; i<50; i++)
  {
    expandTreeOneTime();
  }
  int golden_tree_size{23};
  EXPECT_EQ(golden_tree_size, m_tree.size());
}

TEST_F(TestRRTPlanner, whenExpandingTree_TreeReachesGoalAndStops)
{
  m_state->goal_point->north = -3;
  m_state->goal_point->east = -8.5;
  initializeTree();
  bool reached_goal{expandTreeToGoal()};

  EXPECT_TRUE(reached_goal);
  EXPECT_EQ(m_state->goal_point->north, m_tree.back().getX());
  EXPECT_EQ(m_state->goal_point->east, m_tree.back().getY());
  EXPECT_EQ(m_tree.back().getParent(),m_tree.size()-2);
}

TEST_F(TestRRTPlanner, whenExpandingTreeWithOutOfBoundsGoal_FunctionExitsCleanly)
{
  m_state->goal_point->north = -4;
  m_state->goal_point->east = -5.5;
  initializeTree();
  bool reached_goal{expandTreeToGoal()};

  EXPECT_FALSE(reached_goal);
}

TEST_F(TestRRTPlanner, whenCheckingBadGoalPoint_ValidGoalPointIsFalse)
{
  m_state->goal_point->north = -2.5;
  m_state->goal_point->east = -4.5;
  initializeTree();
  bool valid_goal{checkIfGoalIsValid()};
  EXPECT_FALSE(valid_goal);
}

TEST_F(TestRRTPlanner, whenCheckingBadStartPoint_ValidStartPointIsFalse)
{
  m_state->position->north = -2.5;
  m_state->position->east = -4.5;
  initializeTree();
  bool valid_start{checkIfCurrentLocationIsValid()};
  EXPECT_FALSE(valid_start);
}

TEST_F(TestRRTPlanner, whenFindingPathToGoal_PathIsCompleteAndCorrect)
{
  m_state->goal_point->north = -3;
  m_state->goal_point->east = -8.5;
  initializeTree();
  bool valid_goal{checkIfGoalIsValid()};
  bool valid_start{checkIfCurrentLocationIsValid()};
  bool reached_goal{expandTreeToGoal()};
  std::vector<int> path_to_goal{findPathToGoal()};
  std::vector<int> golden_path_to_goal{0, 1, 2, 3, 4, 5, 6, 7, 11, 12, 13, 14, 15, 17, 21, 23, 27, 28};

  EXPECT_TRUE(valid_start);
  EXPECT_TRUE(valid_goal);
  EXPECT_TRUE(reached_goal);
  EXPECT_EQ(golden_path_to_goal, path_to_goal);
}

TEST_F(TestRRTPlanner, whenSmoothingPath_PathIsShorterAndComplete)
{
  m_state->goal_point->north = -3;
  m_state->goal_point->east = -8.5;
  initializeTree();
  bool reached_goal{expandTreeToGoal()};
  std::vector<int> path_to_goal{findPathToGoal()};
  std::vector<int> smoothed_path{smoothPath(path_to_goal)};
  std::vector<int> golden_smoothed_path{0,27,28};

  EXPECT_TRUE(reached_goal);
  EXPECT_EQ(golden_smoothed_path,smoothed_path);
}

TEST_F(TestRRTPlanner, whenGettingSmoothedPath_PathReturnedIsCorrect)
{
  m_state->goal_point->north = -3;
  m_state->goal_point->east = -8.5;
  initializeTree();
  bool reached_goal{expandTreeToGoal()};
  std::vector<std::vector<double>> smoothed_path_points{getSmoothedPath()};
  std::vector<std::vector<double>> golden_smoothed_path_points{{0,0},{-1.554787332400035,-7.366500669628218},{-3,-8.5}};

  EXPECT_TRUE(reached_goal);
  EXPECT_TRUE(expectNearDoubleVector(golden_smoothed_path_points, smoothed_path_points, 0.00001));
}

TEST_F(TestRRTPlanner, whenCheckingPathForCollision_returnsTrueForCollision)
{
  Waypoint waypoint1{0,0,0,0,0};
  Waypoint waypoint2{-2,-2.5,0,0,0};
  Waypoint waypoint3{-1,-5.5,0,0,0};
  Waypoint waypoint4{-5,-8.5,0,0,0};
  std::vector<Waypoint> path{waypoint1, waypoint2, waypoint3, waypoint4};
  bool collision = checkPathForCollisions(path);
  EXPECT_TRUE(collision);
}

TEST_F(TestRRTPlanner, whenCheckingPathForCollision_returnsFalseWithNoCollision)
{
  Waypoint waypoint1{0,0,0,0,0};
  Waypoint waypoint2{-2,-2.5,0,0,0};
  Waypoint waypoint3{-1,-5.5,0,0,0};
  Waypoint waypoint4{-2,-8.5,0,0,0};
  std::vector<Waypoint> path{waypoint1, waypoint2, waypoint3, waypoint4};
  bool collision = checkPathForCollisions(path);
  EXPECT_FALSE(collision);
}
