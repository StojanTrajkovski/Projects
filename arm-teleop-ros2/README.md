# ROS 2 Arm Teleoperation (WidowX 250s)

A ROS 2 package for teleoperating a WidowX 250s robotic arm via joystick, developed as part of
university coursework.

## What I implemented

**Robot driver node** (`robot_driver_node.py`) — subscribes to joint and mode commands,
publishes live joint state, and drives the arm via the vendor SDK. Includes proportional speed
scaling for smooth Cartesian-mode motion (tuned after finding a single fixed speed made
movement inconsistent across different travel distances).

**Joystick processor node** (`joy_processor_node.py`) — reads joystick input and converts it
into joint-space or Cartesian-space motion goals, with deadzone handling, smooth stop-on-release
behaviour, and mode switching between joint and Cartesian control.

## What was provided

The forward/inverse kinematics module (`wx250s_kinematics.py`) — DH table and Newton-Raphson IK
solver for the WidowX 250s — [was provided as part of the course / I derived myself — need to
confirm]. The two nodes above call into it but were built by me.

## Status

Developed and tested on physical hardware.
