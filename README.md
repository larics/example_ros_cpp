# example_ros_cpp
An example of a ROS C++ package conforming to the [LARICS C++ coding standard](http://larics.rasip.fer.hr/farm/laricswiki/doku.php?id=software:coding_standard#c_coding_standards) and [ROS developers guide](http://wiki.ros.org/DevelopersGuide).

In this example a PID control algorithms is implemented.

## Dependencies

The code has been developed and tested on an Ubuntu 14.04 system. The following libraries are required to build the code and the developer docs:

 * C++ compiler and linker
 * Robot Operating System (developed and tested on Indigo)
 * doxygen

## ROS package description

### ROS dependencies

[std_msgs](http://wiki.ros.org/std_msgs)

### Nodes
 * pid_controller_node
 The node a pid controller algorithm. It receives referent and process value through ROS topics and publishes referent actuator value.

### Subscribed topics
 * reference ([std_msgs/Float32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Float32.html))
 * measurement ([std_msgs/Float32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Float32.html))

### Published topics
 * actuator ([std_msgs/Float32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Float32.html))

## Quickstart

### Building the code

The code is built in a standard ROS fashion. In your ROS workspace run:

 ```
  catkin_make --pkg example_ros_cpp
 ```

### Building and viewing the docs

From the package root (where `Doxyfile` is located), run:

```
doxygen
firefox doc/html/index.html &
```
