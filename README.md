# Projects

A collection of robotics and embedded systems projects from my coursework and extracurricular
work, built while studying Mechatronics Engineering & Computer Science.

## [turtlebot4-navigation](./turtlebot4-navigation)

ROS 2 / Nav2 practicals on a TurtleBot4 — an AMCL sensor model for particle-filter
localization, the core algorithm behind a custom A* global planner, LIDAR-based reactive
control (wall/person following), and a ROS 2 action server. Developed and tested in both
Gazebo simulation and on physical hardware.

## [arm-teleop-ros2](./arm-teleop-ros2)

A ROS 2 package for teleoperating a WidowX 250s robotic arm via joystick — pub/sub-based
driver and control nodes handling joint-space and Cartesian-space motion, gripper control, and
homing.

## [cubesat-coil-winder](./cubesat-coil-winder)

Automated coil-winding hardware and embedded control system for a CubeSat's attitude
determination and control subsystem, built with the Perth Aerospace Student Team — mechanical
design, dual-stepper actuation, and ESP32 firmware.

---

Each project folder has its own README with more detail on what was built and, where the
project was based on a provided framework (coursework or team codebase), a clear note on what
was mine versus what was already there.
