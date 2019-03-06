#include <gtest/gtest.h>
#include <math.h>
#include <vector>
#include "transforms2d.hpp"
#include "test_functions.h"
#pragma clang diagnostic ignored "-Wweak-vtables"

class TestTransformations2d : public ::testing::Test
{
public:
  TestTransformations2d(){}
  std::vector<double> point1{1,1};
  double angle1{15.0};
  double angle2{90.0};
  std::vector<double> translation{3,5};
  double scale1{15.0};
  double scale2{0.05};

  std::vector<double> point2{0,0};
  std::vector<double> point3{-1,-1};
  std::vector<double> point4{3,-4};
  std::vector<double> point5{-2,3};

  std::vector<std::vector<double>> points{point1, point2, point3, point4, point5};
};

TEST_F(TestTransformations2d, whenRotatingPointByAngle_rotationIsCorrect)
{
  std::vector<double> rotated_point1(rotatePoint(point1,angle1));
  std::vector<double> golden_rotated_point1{0.7071,1.2247};
  std::vector<double> rotated_point2(rotatePoint(point1,angle2));
  std::vector<double> golden_rotated_point2{-1.0,1.0};

  EXPECT_NEAR(rotated_point1[0],golden_rotated_point1[0],0.0001);
  EXPECT_NEAR(rotated_point1[1],golden_rotated_point1[1],0.0001);
  EXPECT_NEAR(rotated_point2[0],golden_rotated_point2[0],0.0001);
  EXPECT_NEAR(rotated_point2[1],golden_rotated_point2[1],0.0001);
}

TEST_F(TestTransformations2d, whenTranslatingPoint_TranslationIsCorrect)
{
  std::vector<double> translated_point{translatePoint(point1,translation)};
  std::vector<double> golden_translated_point{4,6};

  EXPECT_EQ(translated_point, golden_translated_point);
}

TEST_F(TestTransformations2d, whenScalingPoint_ScaledValueIsCorrect)
{
  std::vector<double> scaled_point1(scalePoint(point1,scale1));
  std::vector<double> golden_scaled_point1{15.0,15.0};
  std::vector<double> scaled_point2(scalePoint(point1,scale2));
  std::vector<double> golden_scaled_point2{0.05,0.05};

  EXPECT_NEAR(scaled_point1[0],golden_scaled_point1[0],0.0001);
  EXPECT_NEAR(scaled_point1[1],golden_scaled_point1[1],0.0001);
  EXPECT_NEAR(scaled_point2[0],golden_scaled_point2[0],0.0001);
  EXPECT_NEAR(scaled_point2[1],golden_scaled_point2[1],0.0001);
}

TEST_F(TestTransformations2d, whenRotatingVectorOfPointsByAngle_rotationIsCorrect)
{
  std::vector<std::vector<double>> rotated_vector{rotateVector(points,angle1)};
  std::vector<std::vector<double>> golden_rotated_vector{{0.7071067812, 1.224744871},
                                                         {0, 0},
                                                         {-0.7071067812, -1.224744871},
                                                         {3.933053659,	-3.08724617},
                                                         {-2.708308788,	2.380139389}};
  EXPECT_TRUE(expectNearDoubleVector(golden_rotated_vector, rotated_vector,0.00001));
}

TEST_F(TestTransformations2d, whenTranslatingVectorOfPoints_TranslationIsCorrect)
{
  std::vector<std::vector<double>> translated_vector{translateVector(points,translation)};
  std::vector<std::vector<double>> golden_translated_vector{{4, 6},
                                                            {3, 5},
                                                            {2, 4},
                                                            {6,	1},
                                                            {1,	8}};
  EXPECT_EQ(golden_translated_vector, translated_vector);
}

TEST_F(TestTransformations2d, whenScalingVectorOfPoints_scaleIsCorrect)
{
  std::vector<std::vector<double>> rotated_vector{scaleVector(points,scale2)};
  std::vector<std::vector<double>> golden_rotated_vector{{0.05,0.05},
                                                         {0, 0},
                                                         {-0.05, -0.05},
                                                         {0.15,	-0.25},
                                                         {-0.1,	0.15}};
  EXPECT_TRUE(expectNearDoubleVector(golden_rotated_vector, rotated_vector,0.00001));
}

TEST_F(TestTransformations2d, whenConvertingPointFromNEDFrameToNWUFrame_ConversionIsCorrect)
{
  std::vector<double> nwu_point{convertPointNED2NWU(point1)};
  std::vector<double> golden_nwu_point{1,-1};
  EXPECT_EQ(golden_nwu_point,nwu_point);
}

TEST_F(TestTransformations2d, whenConvertingPointFromNEDFrameToPixelFrame_ConversionIsCorrect)
{
  std::vector<double> pixel_point{convertPointNED2Pixel(point4)};
  std::vector<double> golden_pixel_point{-4,-3};
  EXPECT_EQ(golden_pixel_point,pixel_point);
}

TEST_F(TestTransformations2d, whenConvertingPointFromNWUFrameToPixelFrame_ConversionIsCorrect)
{
  std::vector<double> pixel_point{convertPointNWU2Pixel(point4)};
  std::vector<double> golden_pixel_point{4,-3};
  EXPECT_EQ(golden_pixel_point,pixel_point);
}

TEST_F(TestTransformations2d, whenConvertingPointFromNWUFrameToNEDFrame_ConversionIsCorrect)
{
  std::vector<double> ned_point{convertPointNWU2NED(point4)};
  std::vector<double> golden_ned_point{3,4};
  EXPECT_EQ(golden_ned_point,ned_point);
}

TEST_F(TestTransformations2d, whenConvertingPointFromPixelFrameToNEDFrame_ConversionIsCorrect)
{
  std::vector<double> ned_point{convertPointPixel2NED(point4)};
  std::vector<double> golden_ned_point{4,3};
  EXPECT_EQ(golden_ned_point,ned_point);
}

TEST_F(TestTransformations2d, whenConvertingPointFromPixelFrameToNWUFrame_ConversionIsCorrect)
{
  std::vector<double> ned_point{convertPointPixel2NWU(point4)};
  std::vector<double> golden_ned_point{4,-3};
  EXPECT_EQ(golden_ned_point,ned_point);
}

TEST_F(TestTransformations2d, whenConvertingPointFromGridFrameToPixelFrame_ConversionIsCorrect)
{
  std::vector<double> pixel_point{convertPointGrid2Pixel(point4)};
  std::vector<double> golden_pixel_point{-3,4};
  EXPECT_EQ(golden_pixel_point,pixel_point);
}

TEST_F(TestTransformations2d, whenConvertingPointFromNWUFrameToGridFrame_ConversionIsCorrect)
{
  std::vector<double> nwu_point{convertPointNWU2Grid(point4)};
  std::vector<double> golden_grid_point{-4,3};
  EXPECT_EQ(golden_grid_point,nwu_point);
}

TEST_F(TestTransformations2d, whenConvertingVectorFromNEDFrameToNWUFrame_ConversionIsCorrect)
{
  std::vector<std::vector<double>> nwu_points{convertVectorNED2NWU(points)};
  std::vector<std::vector<double>> golden_nwu_points{{1,-1},
                                                     {0,0},
                                                     {-1,1},
                                                     {3,4},
                                                     {-2,-3}};
  EXPECT_EQ(golden_nwu_points,nwu_points);
}

TEST_F(TestTransformations2d, whenConvertingVectorFromNEDFrameToPixelFrame_ConversionIsCorrect)
{
  std::vector<std::vector<double>>  pixel_points{convertVectorNED2Pixel(points)};
  std::vector<std::vector<double>>  golden_pixel_points{{1,-1},
                                                        {0,0},
                                                        {-1,1},
                                                        {-4,-3},
                                                        {3,2}};
  EXPECT_EQ(golden_pixel_points,pixel_points);
}

TEST_F(TestTransformations2d, whenConvertingVectorFromNWUFrameToPixelFrame_ConversionIsCorrect)
{
  std::vector<std::vector<double>>  pixel_points{convertVectorNWU2Pixel(points)};
  std::vector<std::vector<double>>  golden_pixel_points{{-1,-1},
                                                        {0,0},
                                                        {1,1},
                                                        {4,-3},
                                                        {-3,2}};
  EXPECT_EQ(golden_pixel_points,pixel_points);
}

TEST_F(TestTransformations2d, whenConvertingVectorFromNWUFrameToNEDFrame_ConversionIsCorrect)
{
  std::vector<std::vector<double>>  ned_points{convertVectorNWU2NED(points)};
  std::vector<std::vector<double>>  golden_ned_points{{1,-1},
                                                      {0,0},
                                                      {-1,1},
                                                      {3,4},
                                                      {-2,-3}};
  EXPECT_EQ(golden_ned_points,ned_points);
}

TEST_F(TestTransformations2d, whenConvertingVectorFromPixelFrameToNEDFrame_ConversionIsCorrect)
{
  std::vector<std::vector<double>>  ned_points{convertVectorPixel2NED(points)};
  std::vector<std::vector<double>>  golden_ned_points{{-1,1},
                                                      {0,0},
                                                      {1,-1},
                                                      {4,3},
                                                      {-3,-2}};
  EXPECT_EQ(golden_ned_points,ned_points);
}

TEST_F(TestTransformations2d, whenConvertingVectorFromPixelFrameToNWUFrame_ConversionIsCorrect)
{
  std::vector<std::vector<double>>  ned_points{convertVectorPixel2NWU(points)};
  std::vector<std::vector<double>>  golden_ned_points{{-1,-1},
                                                      {0,0},
                                                      {1,1},
                                                      {4,-3},
                                                      {-3,2}};
  EXPECT_EQ(golden_ned_points,ned_points);
}

TEST_F(TestTransformations2d, whenConvertingVectorFromGridFrameToPixelFrame_ConversionIsCorrect)
{
  std::vector<std::vector<double>>  pixel_points{convertVectorGrid2Pixel(points)};
  std::vector<std::vector<double>>  golden_pixel_points{{-1,-1},
                                                        {0,0},
                                                        {1,1},
                                                        {-3,4},
                                                        {2,-3}};
  EXPECT_EQ(golden_pixel_points,pixel_points);
}

TEST_F(TestTransformations2d, whenConvertingPointFromIntToDouble_ReturnsVectorOfDoubles)
{
  std::vector<int> int_point{1,1};
  std::vector<double> golden_double_point{1.0,1.0};
  EXPECT_EQ(golden_double_point,convertIntPoint2DoublePoint(int_point));
}

TEST_F(TestTransformations2d, whenConvertingVectorFromIntToDouble_ReturnsVectorOfVectorOfDoubles)
{
  std::vector<std::vector<int>> int_vec{{1,1},{2,2},{3,3},{4,4}};
  std::vector<std::vector<double>> golden_double_point{{1.0,1.0},{2.0,2.0},{3.0,3.0},{4.0,4.0}};
  EXPECT_EQ(golden_double_point,convertIntVector2DoubleVector(int_vec));
}

TEST_F(TestTransformations2d, whenAskingForXValues_ValuesAreCorrect)
{
  std::vector<double> x_values{getXValues(points)};
  std::vector<double> golden_x_values{1.0, 0.0, -1.0, 3.0, -2.0};
  EXPECT_EQ(golden_x_values, x_values);
}

TEST_F(TestTransformations2d, whenAskingForYValues_ValuesAreCorrect)
{
  std::vector<double> y_values{getYValues(points)};
  std::vector<double> golden_y_values{1.0, 0.0, -1.0, -4.0, 3.0};
  EXPECT_EQ(golden_y_values, y_values);
}

TEST_F(TestTransformations2d, whenCheckingObstacleMinAndMax_MinAndMaxAreCorrect)
{
  std::vector<double> x_min_max{getXMinMax(points)};
  double golden_x_min{-2.0};
  double golden_x_max{3.0};
  std::vector<double> y_min_max{getYMinMax(points)};
  double golden_y_min{-4.0};
  double golden_y_max{3};

  EXPECT_NEAR(golden_x_min,x_min_max[0],0.00001);
  EXPECT_NEAR(golden_y_min,y_min_max[0],0.00001);
  EXPECT_NEAR(golden_x_max,x_min_max[1],0.00001);
  EXPECT_NEAR(golden_y_max,y_min_max[1],0.00001);
}
