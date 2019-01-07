#include "transforms2d.hpp"
#include <tgmath.h>
#include <algorithm>
#include <vector>

std::vector<double> rotatePoint(const std::vector<double> &point, double angle)
{
    std::vector<double> rotated_point{0,0};
    double angle_radian{angle*PI/180};
    rotated_point[0] = point[0]*cos(angle_radian) + point[1]*-sin(angle_radian);
    rotated_point[1] = point[0]*sin(angle_radian) + point[1]*cos(angle_radian);

    return rotated_point;
}

std::vector<double> translatePoint(const std::vector<double> &point, const std::vector<double> &translation)
{
    std::vector<double> translated_point{0,0};

    translated_point[0] = point[0]+translation[0];
    translated_point[1] = point[1]+translation[1];

    return translated_point;
}

std::vector<double> scalePoint(const std::vector<double> &point, double scale)
{
    std::vector<double> scaled_point{0,0};
    scaled_point[0] = point[0]*scale;
    scaled_point[1] = point[1]*scale;

    return scaled_point;
}

std::vector<std::vector<double>> rotateVector(const std::vector<std::vector<double>> &points, double angle)
{
    std::vector<std::vector<double>> rotated_points{};
    for (std::vector<double> point: points)
    {
        point = rotatePoint(point,angle);
        rotated_points.push_back(point);
    }
    return rotated_points;
}

std::vector<std::vector<double>> translateVector(const std::vector<std::vector<double>> &points, const std::vector<double> &translation)
{
    std::vector<std::vector<double>> translated_points{};
    for (std::vector<double> point: points)
    {
        point = translatePoint(point,translation);
        translated_points.push_back(point);
    }
    return translated_points;
}

std::vector<std::vector<double>> scaleVector(const std::vector<std::vector<double>> &points, double scale)
{
    std::vector<std::vector<double>> scaled_points{};
    for (std::vector<double> point: points)
    {
        point = scalePoint(point,scale);
        scaled_points.push_back(point);
    }
    return scaled_points;
}

std::vector<double> convertPointNED2NWU(const std::vector<double> &ned_point)
{
  std::vector<double> nwu_point{0,0};
  nwu_point[0]=ned_point[0];
  nwu_point[1]=-ned_point[1];
  return nwu_point;
}

std::vector<double> convertPointNED2Pixel(const std::vector<double> &ned_point)
{
  std::vector<double> pixel_point{0,0};
  pixel_point[0]=ned_point[1];
  pixel_point[1]=-ned_point[0];
  return pixel_point;
}

std::vector<double> convertPointNWU2Pixel(const std::vector<double> &nwu_point)
{
  std::vector<double> pixel_point{0,0};
  pixel_point[0]=-nwu_point[1];
  pixel_point[1]=-nwu_point[0];
  return pixel_point;
}

std::vector<double> convertPointNWU2NED(const std::vector<double> &nwu_point)
{
  std::vector<double> ned_point{0,0};
  ned_point[0]=nwu_point[0];
  ned_point[1]=-nwu_point[1];
  return ned_point;
}

std::vector<double> convertPointPixel2NED(const std::vector<double> &pixel_point)
{
  std::vector<double> ned_point{0,0};
  ned_point[0]=-pixel_point[1];
  ned_point[1]=pixel_point[0];
  return ned_point;
}

std::vector<double> convertPointPixel2NWU(const std::vector<double> &pixel_point)
{
  std::vector<double> nwu_point{0,0};
  nwu_point[0]=-pixel_point[1];
  nwu_point[1]=-pixel_point[0];
  return nwu_point;
}

std::vector<double> convertPointGrid2Pixel(const std::vector<double> &grid_point)
{
  std::vector<double> pixel_point{0,0};
  pixel_point[0]=-grid_point[0];
  pixel_point[1]=-grid_point[1];
  return pixel_point;
}

std::vector<double> convertPointNWU2Grid(const std::vector<double> &nwu_point)
{
  std::vector<double> pixel_point{0,0};
  pixel_point[0]=nwu_point[1];
  pixel_point[1]=nwu_point[0];
  return pixel_point;
}

std::vector<std::vector<double>> convertVectorNED2NWU(const std::vector<std::vector<double>> &ned_points)
{
    std::vector<std::vector<double>> nwu_points{};
    for (std::vector<double> point: ned_points)
    {
        point = convertPointNED2NWU(point);
        nwu_points.push_back(point);
    }
    return nwu_points;
}

std::vector<std::vector<double>> convertVectorNED2Pixel(const std::vector<std::vector<double>> &ned_points)
{
    std::vector<std::vector<double>> pixel_points{};
    for (std::vector<double> point: ned_points)
    {
        point = convertPointNED2Pixel(point);
        pixel_points.push_back(point);
    }
    return pixel_points;
}

std::vector<std::vector<double>> convertVectorNWU2Pixel(const std::vector<std::vector<double>> &nwu_points)
{
    std::vector<std::vector<double>> pixel_points{};
    for (std::vector<double> point: nwu_points)
    {
        point = convertPointNWU2Pixel(point);
        pixel_points.push_back(point);
    }
    return pixel_points;
}

std::vector<std::vector<double>> convertVectorNWU2NED(const std::vector<std::vector<double>> &nwu_points)
{
    std::vector<std::vector<double>> ned_points{};
    for (std::vector<double> point: nwu_points)
    {
        point = convertPointNWU2NED(point);
        ned_points.push_back(point);
    }
    return ned_points;
}

std::vector<std::vector<double>> convertVectorPixel2NED(const std::vector<std::vector<double>> &pixel_points)
{
    std::vector<std::vector<double>> ned_points{};
    for (std::vector<double> point: pixel_points)
    {
        point = convertPointPixel2NED(point);
        ned_points.push_back(point);
    }
    return ned_points;
}

std::vector<std::vector<double>> convertVectorPixel2NWU(const std::vector<std::vector<double>> &pixel_points)
{
    std::vector<std::vector<double>> nwu_points{};
    for (std::vector<double> point: pixel_points)
    {
        point = convertPointPixel2NWU(point);
        nwu_points.push_back(point);
    }
    return nwu_points;
}

std::vector<std::vector<double>> convertVectorGrid2Pixel(const std::vector<std::vector<double>> &grid_points)
{
    std::vector<std::vector<double>> pixel_points{};
    for (std::vector<double> point: grid_points)
    {
        point = convertPointGrid2Pixel(point);
        pixel_points.push_back(point);
    }
    return pixel_points;
}

std::vector<double> convertIntPoint2DoublePoint(const std::vector<int> &int_point)
{
  std::vector<double> double_point(int_point.begin(),int_point.end());
  return double_point;
}

std::vector<std::vector<double>> convertIntVector2DoubleVector(const std::vector<std::vector<int>> &int_vec)
{
  std::vector<std::vector<double>> double_vec;
  for (std::vector<int> int_point: int_vec)
  {
    std::vector<double> double_point{convertIntPoint2DoublePoint(int_point)};
    double_vec.push_back(double_point);
  }
  return double_vec;
}

std::vector<double> getXValues(const std::vector<std::vector<double>> &points)
{
  std::vector<double> x_values{};
  for (std::vector<double> point: points)
  {
    x_values.push_back(point[0]);
  }
  return x_values;
}

std::vector<double> getYValues(const std::vector<std::vector<double>> &points)
{
  std::vector<double> y_values{};
  for (std::vector<double> point: points)
  {
    y_values.push_back(point[1]);
  }
  return y_values;
}

std::vector<double> getXMinMax(const std::vector<std::vector<double>> &points)
{
  std::vector<double> x_values{getXValues(points)};
  auto x_minmax = std::minmax_element(x_values.begin(), x_values.end());
  std::vector<double> x_min_max{*x_minmax.first, *x_minmax.second};
  return x_min_max;
}

std::vector<double> getYMinMax(const std::vector<std::vector<double>> &points)
{
  std::vector<double> y_values{getYValues(points)};
  auto y_minmax = std::minmax_element(y_values.begin(), y_values.end());
  std::vector<double> y_min_max{*y_minmax.first, *y_minmax.second};
  return y_min_max;
}
