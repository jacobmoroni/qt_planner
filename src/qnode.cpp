#include <ros/ros.h>
#include <ros/network.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sstream>
#include <vector>
#include "qnode.hpp"
#include "obstaclemap.hpp"
#include "rosflight_msgs/Command.h"
#include "waypoint.hpp"
#include "waypointmanager.hpp"
#include "settings.hpp"
#include "state.hpp"

namespace qt_planner
{
QNode::QNode(int argc, char** argv, Settings *settings, State *state) :
  init_argc(argc),
  init_argv(argv),
  m_settings{settings},
  m_state{state}
{
  m_current_waypoint.mode = rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE;
}

QNode::~QNode()
{
  if(ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

void QNode::saveParametersToFile(std::string param_file)
{
  ros::param::set("/qt_planner/waypoint_manager/position_threshold",m_settings->waypoint_manager->position_threshold);
  ros::param::set("/qt_planner/waypoint_manager/yaw_threshold",m_settings->waypoint_manager->yaw_threshold);
  ros::param::set("/qt_planner/waypoint_manager/threshold_2d",m_settings->waypoint_manager->threshold_2d);
  ros::param::set("/qt_planner/waypoint_manager/check_path_frequency",m_settings->waypoint_manager->check_path_frequency);
  ros::param::set("/qt_planner/waypoint_manager/preset",m_settings->waypoint_manager->preset);

  ros::param::set("/qt_planner/obstacles/buffer_size",m_settings->obstacle->buffer_size);
  ros::param::set("/qt_planner/obstacles/obstacle_size",m_settings->obstacle->obstacle_size);
  ros::param::set("/qt_planner/obstacles/unknown_as_obstacles",m_settings->obstacle->unknown_as_obstacles);

  ros::param::set("/qt_planner/rrt/timeout",m_settings->rrt->timeout);
  ros::param::set("/qt_planner/rrt/goal_sample_rate",m_settings->rrt->goal_sample_rate);
  ros::param::set("/qt_planner/rrt/expand_distance",m_settings->rrt->expand_distance);
  ros::param::set("/qt_planner/rrt/boundary_buffer",m_settings->rrt->boundary_buffer);

  ros::param::set("/qt_planner/ros/grid_map_topic",m_settings->ros->grid_map_topic);
  ros::param::set("/qt_planner/ros/waypoint_topic",m_settings->ros->waypoint_topic);
  ros::param::set("/qt_planner/ros/tf_from",m_settings->ros->tf_from);
  ros::param::set("/qt_planner/ros/tf_to",m_settings->ros->tf_to);
  ros::param::set("/qt_planner/ros/mav_name",m_settings->ros->mav_name);
  ros::param::set("/qt_planner/ros/tf_reference_frame",m_settings->ros->tf_reference_frame);
  std::stringstream command;
  command<<"rosparam dump "<<param_file<<" qt_planner"<<std::endl;
  const std::string tmp{command.str()};
  const char* cstr{tmp.c_str()};
  int success{system(cstr)};
  if (success == 1)
    log(Info, "Falied to load file");

  std::stringstream log_msg;
  log_msg<<"Saved parameter file to: "<<param_file;
  log(Info, log_msg.str());
}

void QNode::setParametersFromFile(std::string param_file)
{
  ros::NodeHandle parameters{"~"};
  std::stringstream command;
  command<<"rosparam load "<<param_file<<std::endl;
  const std::string tmp{command.str()};
  const char* cstr{tmp.c_str()};
  int success{system(cstr)};
  if (success == 1)
    log(Info, "Falied to load file");
  if (!ros::param::get("/waypoint_manager/position_threshold",m_settings->waypoint_manager->position_threshold))
  {
    log(Warn,"Failed to load paramaters from file");
  }
  else
  {
    parameters.param<double>("/waypoint_manager/yaw_threshold",m_settings->waypoint_manager->yaw_threshold,0.523599);
    parameters.param<bool>("/waypoint_manager/threshold_2d",m_settings->waypoint_manager->threshold_2d,true);
    parameters.param<double>("/waypoint_manager/check_path_frequency",m_settings->waypoint_manager->check_path_frequency,5);
    parameters.param<bool>("/waypoint_manager/preset", m_settings->waypoint_manager->preset,false);

    parameters.param<double>("/obstacles/buffer_size",m_settings->obstacle->buffer_size,0.8);
    parameters.param<double>("/obstacles/obstacle_size",m_settings->obstacle->obstacle_size,0.1);
    parameters.param<bool>("/obstacles/unknown_as_obstacles",m_settings->obstacle->unknown_as_obstacles,false);

    parameters.param<int>("/rrt/timeout",m_settings->rrt->timeout, 5000);
    parameters.param<int>("/rrt/goal_sample_rate",m_settings->rrt->goal_sample_rate, 50);
    parameters.param<double>("/rrt/expand_distance",m_settings->rrt->expand_distance, 0.5);
    parameters.param<double>("/rrt/boundary_buffer",m_settings->rrt->boundary_buffer, 2);

    parameters.param<std::string>("/ros/grid_map_topic",m_settings->ros->grid_map_topic,"/rtabmap/grid_map");
    parameters.param<std::string>("/ros/waypoint_topic",m_settings->ros->waypoint_topic,"/raw_waypoints");
    parameters.param<std::string>("/ros/tf_from",m_settings->ros->tf_from,"/world");
    parameters.param<std::string>("/ros/tf_to",m_settings->ros->tf_to,"/base_link");
    parameters.param<std::string>("/ros/mav_name",m_settings->ros->mav_name,"agent");
    parameters.param<int>("/ros/tf_reference_frame",m_settings->ros->tf_reference_frame, m_settings->ros->NED);

    std::stringstream log_msg;
    log_msg<<"Loaded parameter file from: "<<param_file;
    log(Info, log_msg.str());
  }
}

void QNode::loadParametersOnStart()
{
  ros::NodeHandle parameters{"~"};

  if (!ros::param::get("/waypoint_manager/position_threshold",m_settings->waypoint_manager->position_threshold))
  {
    log(Warn,"Failed to load paramaters from file");
  }
  else
  {
    parameters.param<double>("/waypoint_manager/yaw_threshold",m_settings->waypoint_manager->yaw_threshold,0.523599);
    parameters.param<bool>("/waypoint_manager/threshold_2d",m_settings->waypoint_manager->threshold_2d,true);
    parameters.param<double>("/waypoint_manager/check_path_frequency",m_settings->waypoint_manager->check_path_frequency,5);
    parameters.param<bool>("/waypoint_manager/preset", m_settings->waypoint_manager->preset,false);

    parameters.param<double>("/obstacles/buffer_size",m_settings->obstacle->buffer_size,0.8);
    parameters.param<double>("/obstacles/obstacle_size",m_settings->obstacle->obstacle_size,0.1);
    parameters.param<bool>("/obstacles/unknown_as_obstacles",m_settings->obstacle->unknown_as_obstacles,false);

    parameters.param<int>("/rrt/timeout",m_settings->rrt->timeout, 5000);
    parameters.param<int>("/rrt/goal_sample_rate",m_settings->rrt->goal_sample_rate, 50);
    parameters.param<double>("/rrt/expand_distance",m_settings->rrt->expand_distance, 0.5);
    parameters.param<double>("/rrt/boundary_buffer",m_settings->rrt->boundary_buffer, 2);

    parameters.param<std::string>("/ros/grid_map_topic",m_settings->ros->grid_map_topic,"/rtabmap/grid_map");
    parameters.param<std::string>("/ros/waypoint_topic",m_settings->ros->waypoint_topic,"/raw_waypoints");
    parameters.param<std::string>("/ros/tf_from",m_settings->ros->tf_from,"/world");
    parameters.param<std::string>("/ros/tf_to",m_settings->ros->tf_to,"/base_link");
    parameters.param<std::string>("/ros/mav_name",m_settings->ros->mav_name,"agent");
    parameters.param<int>("/ros/tf_reference_frame",m_settings->ros->tf_reference_frame, m_settings->ros->NED);

    std::stringstream log_msg;
    log_msg<<"Loaded parameters from ros";
    log(Info, log_msg.str());
  }
}

bool QNode::initROSCommunication()
{
  ros::init(init_argc,init_argv,"qt_planner");
  //if (robot_listener !=nullptr)
  //{
    //delete robot_listener;
    //robot_listener = nullptr;
  //}

  if ( ! ros::master::check() )
  {
    return false;
  }
  ros::start();
  ros::NodeHandle n;
  loadParametersOnStart();
  robot_listener = new tf::TransformListener();
  map_subscriber = n.subscribe(m_settings->ros->grid_map_topic, 1, &QNode::gridMapCallback, this);
  waypoint_subscriber = n.subscribe(m_settings->ros->waypoint_topic, 1, &QNode::waypointCallback, this);
  waypoint_publisher = n.advertise<rosflight_msgs::Command>("waypoint_command", 50);
  start();
  return true;
}

void QNode::gridMapCallback(const nav_msgs::OccupancyGrid &msg)
{
  m_state->obstacles->setValues(msg);
}

void QNode::waypointCallback(const std_msgs::Float32MultiArray &msg)
{
  std::vector<std::vector<double>> waypoint_list;
  size_t rows{static_cast<size_t>(msg.layout.dim[0].size)};
  size_t cols{static_cast<size_t>(msg.layout.dim[1].size)};
  for (size_t i{0};i<rows;i++)
  {
    waypoint_list.emplace_back(rows);
    for (size_t j{0};j<cols;j++)
    {
      waypoint_list[i].push_back(static_cast<double>(msg.data[i*cols+rows]));
    }
  }
  m_state->waypoint_list = waypoint_list;
}

void QNode::updateRobotTransform()
{
  tf::StampedTransform robot_transform;
  try
  {
    robot_listener->lookupTransform(m_settings->ros->tf_from, m_settings->ros->tf_to, ros::Time(0), robot_transform);
  }
  catch (tf::TransformException ex)
  {
    log(Warn, (static_cast<std::string>(ex.what())));
    ros::Duration(1.0).sleep();
  }
  double roll{0};
  double pitch{0};
  robot_transform.getBasis().getRPY(roll, pitch, m_state->position->yaw);
  if (m_settings->ros->tf_reference_frame == m_settings->ros->NED)
  {
    m_state->position->north = robot_transform.getOrigin().x();
    m_state->position->east = robot_transform.getOrigin().y();
    m_state->position->down = robot_transform.getOrigin().z();
  }
  else if (m_settings->ros->tf_reference_frame == m_settings->ros->NWU)
  {
    m_state->position->north = robot_transform.getOrigin().x();
    m_state->position->east = -robot_transform.getOrigin().y();
    m_state->position->down = -robot_transform.getOrigin().z();
  }
}

void QNode::run()
{
  ros::Rate loop_rate(16);
  int counter{0};
  while ( ros::ok() )
  {
    updateRobotTransform();

    emit requestCurrentWaypoint();
    if (counter%(static_cast<int>(m_settings->waypoint_manager->check_path_frequency*16)) == 0)
      emit checkPath();
    broadcastCurrentWaypoint();
    waypoint_publisher.publish(m_current_waypoint);
    ros::spinOnce();
    counter ++;
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  emit rosShutdown();
}


void QNode::log( const LogLevel &level, const std::string &msg)
{
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  switch ( level )
  {
  case(Debug) :
  {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Info) :
  {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Warn) :
  {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Error) :
  {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Fatal) :
  {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  emit loggingUpdated();
}

std::string QNode::getMultiArrayTopics()
{
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);
  std::stringstream topics;

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
  {
    const ros::master::TopicInfo& info = *it;
    if (info.datatype == "std_msgs/Float32MultiArray")
    {
        topics << info.name << ",";
    }
  }
  return topics.str();
}

std::string QNode::getOccupancyGridTopics()
{
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);
  std::stringstream topics;

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
  {
    const ros::master::TopicInfo& info = *it;
    if (info.datatype == "nav_msgs/OccupancyGrid")
    {
        topics << info.name << ",";
    }
  }
  return topics.str();
}

void QNode::updateWaypoint(Waypoint current_waypoint)
{
  m_current_waypoint.x = static_cast<float>(current_waypoint.getNorth());
  m_current_waypoint.y = static_cast<float>(current_waypoint.getEast());
  m_current_waypoint.F = static_cast<float>(current_waypoint.getDown());
  m_current_waypoint.z = static_cast<float>(current_waypoint.getYaw());
  m_waypoint = current_waypoint;
}

void QNode::broadcastCurrentWaypoint()
{
  static tf::TransformBroadcaster broadaster;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3{m_waypoint.getNorth(), m_waypoint.getEast(), m_waypoint.getDown()});
  transform.setRotation(tf::createQuaternionFromYaw(m_waypoint.getYaw()));
  std::stringstream waypoint_frame;
  waypoint_frame<<m_settings->ros->mav_name<<"/current_waypoint";
  broadaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", waypoint_frame.str()));
}
}  // namespace qt_planner
