#ifndef QNODE_HPP
#define QNODE_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <tf/transform_listener.h>
#include "rosflight_msgs/Command.h"
#include "waypoint.hpp"

class Settings;
class State;

namespace qt_planner {
class Widget;

class QNode : public QThread {
  Q_OBJECT

public:
  enum LogLevel {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
  };

  QNode(int argc, char **argv, Settings *settings, State *state);
  virtual ~QNode();
  bool initROSCommunication();
  void run();
  void gridMapCallback(const nav_msgs::OccupancyGrid &msg);
  void waypointCallback(const std_msgs::Float32MultiArray &msg);
  QStringListModel* loggingModel() {return &logging_model;}
  void log(const LogLevel &level, const std::string &msg);
  std::string getOccupancyGridTopics();
  std::string getMultiArrayTopics();
  void updateRobotTransform();
  void updateWaypoint(Waypoint current_waypoint);
  void setParametersFromFile(std::string param_file);
  void saveParametersToFile(std::string param_file);
  void loadParametersOnStart();

signals:
  void loggingUpdated();
  void rosShutdown();
  void requestCurrentWaypoint();
  void checkPath();

private:
  void broadcastCurrentWaypoint();
  int init_argc;
  char** init_argv;
  ros::Publisher waypoint_publisher;
  ros::Subscriber map_subscriber;
  ros::Subscriber waypoint_subscriber;
  QStringListModel logging_model;
  tf::TransformListener *robot_listener{nullptr};
  Settings *m_settings;
  State *m_state;
  rosflight_msgs::Command m_current_waypoint;
  Waypoint m_waypoint;
};

}  // namespace qt_planner

#endif // QNODE_HPP
