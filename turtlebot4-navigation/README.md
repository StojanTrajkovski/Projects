# TurtleBot4 Navigation & Localization

A series of ROS 2 practicals building up a Nav2-based navigation stack for a TurtleBot4,
completed as university coursework. Each practical provided a working ROS 2/Nav2 framework
(node lifecycle, plugin interfaces, message handling) with specific algorithmic components
left for me to implement, marked as milestones in the original assignment.

## What I implemented

**AMCL sensor model** (`amcl-sensor-model/`) — the probabilistic sensor model used to weight
particles during AMCL (Adaptive Monte Carlo Localization). Computes the lidar sensor's pose
relative to the robot, filters invalid and max-range beam readings, and calculates the
likelihood-field probability used to update each particle's weight against the map.

**A\* global planner** (`astar-planner/`) — the core path-planning algorithm
behind a custom Nav2 global planner plugin. Propagates potential values outward from the start
cell across 4- and 8-connected neighbours, applies a heuristic-guided cost threshold to bias
the search toward the goal, and extracts the final path by descending the resulting potential
field.

**Reactive control** (`reactive-control/`) — parameter handling and the control laws for two
LIDAR-based reactive behaviours: computing the bearing and range to the nearest detected
object, then generating velocity commands to follow a wall at a set distance, or follow a
person.

**Action server** (`action-server/`) — goal handling (accept, cancel, execute) and the
odometry callback for a ROS 2 action server that drives the robot through a specified arc.

I also added a `PoseArray` publisher to visualize the SLAM particle cloud in RViz, on top of an
existing gmapping-based SLAM pipeline used elsewhere in the same coursework.

## What was provided

The surrounding framework — the AMCL lifecycle node, k-d tree particle clustering, the Nav2
planner plugin interface, and the SLAM (gmapping) backend itself, and some of the code structure — 
was supplied as part of the course. The files above are the pieces I implemented within that
framework, each corresponding to a set of milestone tasks in the original assignment.

## Status

Developed and tested in both Gazebo simulation and on a physical TurtleBot4.
