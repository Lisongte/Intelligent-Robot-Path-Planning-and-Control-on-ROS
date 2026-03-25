# Intelligent-Robot-Path-Planning-and-Control-on-ROS

A ROS-based intelligent mobile robot navigation project focusing on **map generation, path planning, and motion control**.
The project aims to construct a complete navigation framework for mobile robots under ROS, including environment modeling, global path planning, and future trajectory tracking control.

---

# Project Structure

```bash
Intelligent-Robot-Path-Planning-and-Control-on-ROS/
│── src/                  # ROS source code
│── build/                # Build files
│── devel/                # ROS workspace environment
│── README.md             # Project documentation
```

---

# Environment Requirements

* Ubuntu Linux (Ubuntu 20.04.x is often better)
* ROS Noetic / ROS Melodic
* RViz
* CMake
* catkin workspace

---

# Build the Project

Compile the ROS workspace before running:

```bash
cd ~/robots
catkin_make
```

Then source the environment:

```bash
source ~/robots/devel/setup.bash
```

---

# Map Generation and Path Planning

## Terminal 1: Start ROS Master

```bash
roscore
```

This starts the ROS master for node communication.

---

## Terminal 2: Launch Map Publisher

```bash
source ~/robots/devel/setup.bash
rosrun my_map_display simple_map_publisher
```

This node publishes a predefined occupancy grid map.

---

## Terminal 3: Launch Path Planner

```bash
source ~/robots/devel/setup.bash
rosrun my_map_display path_planner_node
```

This node performs path planning based on the generated map.

---

## Terminal 4: Open RViz Visualization

```bash
source ~/robots/devel/setup.bash
rviz
```

In RViz, add the following displays:

* Map
* Path
* Marker

to visualize the map and planned path.

---

# Current Functions

## Implemented Modules

* Static map generation
* Obstacle publishing
* Basic path planning node
* RViz visualization

---

# Planned Extensions

## Path Planning Algorithms

The following algorithms will be integrated:

* A* Algorithm
* RRT Algorithm
* RRT* Algorithm

---

## Motion Control

Future work includes:

* Differential-drive robot control
* Trajectory tracking
* Virtual control point control method

---

## Trajectory Optimization

* Path interpolation
* Smooth curve generation
* Obstacle avoidance refinement

---

# Future Research Direction

This project will gradually evolve into a complete autonomous mobile robot navigation framework, including:

* environment perception
* path planning
* motion control
* simulation and real-world deployment

---

# Visualization Result

You may place RViz screenshots here:

```markdown
![rviz_result](images/result.png)
```

---

# Notes

Before running the project, make sure all ROS packages are compiled successfully and the workspace path is correct.
