# PD Controller
PD controller implemented in C++ for the Turtlebot

### Prerequisites

Uses ROS Kinetic. Requires a turtlebot2 with a Kobuki base.

### Deployment

```
roslaunch turtlebot_bringup mimimal.launch
roslaunch turtlebot_bringup 3dsensor.launch
rosrun pd_controller pd_controller
```

### Authors
Phong Nguyen
Christopher O'Rourke
Joy Ray

### License
Licensed under the BSD license.
