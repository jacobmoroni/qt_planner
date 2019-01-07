#ifndef TRANSFORMS2D_HPP
#define TRANSFORMS2D_HPP

#include <vector>

std::vector<double> rotatePoint(const std::vector<double> &point, double angle);
std::vector<double> translatePoint(const std::vector<double> &point, const std::vector<double> &translation);
std::vector<double> scalePoint(const std::vector<double> &point, double scale);
std::vector<std::vector<double>> rotateVector(const std::vector<std::vector<double>> &points, double angle);
std::vector<std::vector<double>> translateVector(const std::vector<std::vector<double>> &points, const std::vector<double> &translation);
std::vector<std::vector<double>> scaleVector(const std::vector<std::vector<double>> &points, double scale);
std::vector<double> convertPointNED2NWU(const std::vector<double> &ned_point);
std::vector<double> convertPointNED2Pixel(const std::vector<double> &ned_point);
std::vector<double> convertPointNWU2Pixel(const std::vector<double> &nwu_point);
std::vector<double> convertPointNWU2NED(const std::vector<double> &nwu_point);
std::vector<double> convertPointPixel2NED(const std::vector<double> &pixel_point);
std::vector<double> convertPointPixel2NWU(const std::vector<double> &pixel_point);
std::vector<double> convertPointGrid2Pixel(const std::vector<double> &grid_point);
std::vector<double> convertPointNWU2Grid(const std::vector<double> &nwu_point);
std::vector<std::vector<double>>  convertVectorNED2NWU(const std::vector<std::vector<double>>  &ned_points);
std::vector<std::vector<double>>  convertVectorNED2Pixel(const std::vector<std::vector<double>>  &ned_points);
std::vector<std::vector<double>>  convertVectorNWU2Pixel(const std::vector<std::vector<double>>  &nwu_points);
std::vector<std::vector<double>>  convertVectorNWU2NED(const std::vector<std::vector<double>>  &nwu_points);
std::vector<std::vector<double>>  convertVectorPixel2NED(const std::vector<std::vector<double>>  &pixel_points);
std::vector<std::vector<double>>  convertVectorPixel2NWU(const std::vector<std::vector<double>>  &pixel_points);
std::vector<std::vector<double>>  convertVectorGrid2Pixel(const std::vector<std::vector<double>>  &grid_points);
std::vector<double> convertIntPoint2DoublePoint(const std::vector<int> &int_point);
std::vector<std::vector<double>> convertIntVector2DoubleVector(const std::vector<std::vector<int>> &int_vec);
std::vector<double> getXValues(const std::vector<std::vector<double>> &points);
std::vector<double> getYValues(const std::vector<std::vector<double>> &points);
std::vector<double> getXMinMax(const std::vector<std::vector<double>> &points);
std::vector<double> getYMinMax(const std::vector<std::vector<double>> &points);
const double PI{3.14159265359};

#endif // TRANSFORMS2D_HPP
