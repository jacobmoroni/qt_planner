#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

class Waypoint
{
public:
  Waypoint();
  Waypoint(double north, double east, double down, double yaw, double threshold);
  ~Waypoint();

  double getNorth() const;
  void setNorth(double getNorth);
  double getEast() const;
  void setEast(double getEast);
  double getDown() const;
  void setDown(double getDown);
  double getYaw() const;
  void setYaw(double getYaw);
  double getThreshold() const;
  void setThreshold(double getThreshold);
  void setWaypoint(double north, double east, double down, double yaw, double threshold);

private:
  double m_north{0};
  double m_east{0};
  double m_down{0};
  double m_yaw{0};
  double m_threshold{.2};
};

#endif // WAYPOINT_HPP
