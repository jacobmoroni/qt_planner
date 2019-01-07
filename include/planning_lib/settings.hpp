#ifndef SETTINGS_HPP
#define SETTINGS_HPP

#include <string>

struct WaypointManagerParams
{
    WaypointManagerParams() {}
    double position_threshold{0.5};
    double yaw_threshold{0.523599};
    bool threshold_2d{true};
    double check_path_frequency{5};
};

struct ObstacleParams
{
    ObstacleParams() {}
    double buffer_size{0.8};
    double obstacle_size{0.1};
    bool unknown_as_obstacles{false};
};

struct RRTParams
{
    RRTParams() {}
    int timeout{5000};
    int goal_sample_rate{50};
    double expand_distance{0.5};
    double boundary_buffer{2};
};

struct ROSParams
{
    ROSParams() {}
    enum ReferenceFrame{NED, NWU};

    std::string grid_map_topic{"/rtabmap/grid_map"};
    std::string tf_from{"/world"};
    std::string tf_to{"/base_link"};
    int tf_reference_frame{NED};
};

struct Settings {
  Settings() {}
  ~Settings()
  {
    delete waypoint_manager;
    delete obstacle;
    delete rrt;
    delete ros;
  }
  WaypointManagerParams* waypoint_manager{new WaypointManagerParams};
  ObstacleParams* obstacle{new ObstacleParams};
  RRTParams* rrt{new RRTParams};
  ROSParams* ros{new ROSParams};
};

#endif // SETTINGS_HPP
