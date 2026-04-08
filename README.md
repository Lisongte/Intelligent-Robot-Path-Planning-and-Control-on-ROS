# Intelligent Robot Path Planning and Control on ROS

A ROS-based intelligent mobile robot navigation project focused on **map generation, global path planning, and robot visualization**.

This project aims to build a complete navigation framework for mobile robots under ROS, including environment modeling, path planning, visualization in RViz, and future motion control extensions.

---

# Project Structure

```bash
Intelligent-Robot-Path-Planning-and-Control-on-ROS/
│── src/
│   └── my_map_display/
│       ├── launch/
│       ├── src/
│       ├── config/
│       ├── CMakeLists.txt
│       └── package.xml
│── .gitignore
│── README.md
```

> `build/` and `devel/` are generated automatically after compilation and are not included in the repository.

---

# Environment Requirements

* Ubuntu 20.04
* ROS Noetic (recommended)
* CMake
* catkin workspace
* RViz

---

# Build the Project

Compile the workspace:

```bash
cd ~/robots
catkin_make
```

Source the environment:

```bash
source ~/robots/devel/setup.bash
```

---

# Run the Project

## Start ROS master

```bash
cd ~/robots
source devel/setup.bash
roslaunch my_map_display demo.launch
```

This launch file starts:

* map publisher
* path planner
* robot visualization node
* RViz

---

# System Modules

## Map Generation

Publishes a predefined occupancy grid map.

## Path Planning

Computes a collision-free global path from start point to goal point.

## Robot Visualization

Displays the robot as a marker in RViz.

## RViz Visualization

Visualizes:

* occupancy grid
* path
* robot marker

⚠️ Set **Fixed Frame** to:

```bash
world
```

---

# Current Implemented Functions

* Static map generation
* Obstacle publishing
* Path planning node
* Robot marker display
* Launch file integration

---

# Algorithms

Current / planned planning algorithms:

* A* algorithm

Future extensions:

* path smoothing
* trajectory optimization
* local replanning

---

# Future Work

Future development will include:

* robot motion control
* virtual point tracking
* trajectory following
* dynamic obstacle avoidance

---

# Visualization Result

Add RViz screenshots here:

```markdown
![rviz_result](images/result.png)
```

---

# Notes

Before running the project:

* make sure all ROS packages are compiled successfully
* source the workspace environment
* ensure RViz frame is set correctly

---
