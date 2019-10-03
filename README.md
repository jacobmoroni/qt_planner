# qt_planner
2D RRT planner and waypoint manager using ROS

This is a GUI built with QT wrapped around the 2D RRT planner.
The planner is able to subscribe to a 2D occupancy grid map and autonomously plan paths to a goal with live obstacles from the grid map. It uses tf to determine current robot location and publishes waypoints to follow to tf as well. A path planner (not part of this package) is used to control the error between the waypoint and current position to zero.

![qt_planner](/figures/qt_planner_options.png "qt_planner")
This is a view of the GUI with button options labeled.
The following are the functions of the buttons with their (hotkeys).
1. Fit map to screen (F)
1. Zoom in (+)
1. Zoom out (-)

  Zooming can also be done by scrolling

1. Pan view up (up)
1. Pan view down (down)
1. Pan view left (left)
1. Pan view right (right)

  Navigation can also be done with a right-click and drag (or middle click and drag)

1. Show/hide logger (L):

  ![logger](/figures/logger.png "logger")

  an example of the log messages published by ROS. If the qt_planner is launched from a terminal, the same messages will be logged there.

1. ROS options (R):

  ![ROS options](/figures/ROS_options.png "ROS Options")

  A. Gridmap Topic. Dropdown menu automatically populated with all available gridmap messages.

  B. tf Frames. The `From` frame is the base frame (defaults to {namespace}/map_ned). The `To` frame the the robot frame (defaults to {namespace}/base_link)

  C. tf Reference frame. determine whether you are operating ing NED (north, east, down) or NWU (north, east, up).

  D. waypoint topic (not fully implemented). Inteded to subscribe to high_level waypoint topics and planner would plan between the waypoints.  

1. Obstacle settings (O):

  ![obstacle settings](/figures/obstacle_properties.png "obstacle settings")

  A. Obstacle size. Determines size of obstacles used for planning.

  B. Buffer size. Determines size of obstacle buffer used for collision detection in the RRT planner.

  C. Unknown as obstacles. Toggle whether to treat unknown area as obstacles or free space. Warning: if the map is large, it is VERY expensive to treat unknown as obstacles.

1. RRT planner settings (P):

  ![RRT settings](/figures/RRT_settings.png "RRT settings")

  A. Expand distance. Determines distance of expansion for each RRT search iteration.

  B. Goal sample rate. Determines percent chance of "randomly" sampling the goal location when expanding RRT tree.

  C. Planning timeout. Number of RRT expansions to attempt before giving up on finding the goal.

  D. Boundary buffer. Determines how far out the search area goes past the border obstacles when randomly sampling to find a path.

1. Waypoint manager settings:

  ![wpm settings](/figures/wpm_settings.png "wpm settings")

  A. Use preset waypoints. Couples with subscribing to waypoints. Not fully functional. Built to allow subscribing to high level waypoints.

  B. Position threshold. Determines how close the robot needs to get to the waypoint before the waypoint manager commands the next waypoint.

  C. 2D threshold. Determines whether or not the error calculation for the threshold is 2D or 3D.

  D. Yaw threshold. Determines how close the robot heading needs to be to the commanded heading for the waypoint manager to command the next waypoint.

  E. Path check frequency. Determines how often (in seconds) the path is checked for collisions. The path also is checked for collisions each time a waypoint is reached.

![qt_planner2](/figures/qt_planner_options2.png "qt_planner2")
This is a view of the GUI with goal setting options labeled.
The following are the functions of the buttons.

13. Goal setting values. Can be manually entered in these boxes or the North and East posisitons can be set with a right-click.

1. Send Goal. This sends the goal to the RRT planner. When the planning is finished, the next waypoint is automatically published and the robot begins moving. Click again to cancel path wile navigation is in progress. 

1. Readout of the current waypont [north, east, down, yaw]
1. Readout of the current location [north, east, down, yaw]

Enjoy!
