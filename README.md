# Intelligent Robot Path Planning and Control on ROS

A ROS-based intelligent mobile robot navigation project for **map generation, occupancy grid construction, global path planning, path tracking, and visualization in RViz**.

This project builds a complete navigation pipeline for a differential-drive mobile robot under ROS, including:

* environment map publishing
* occupancy grid generation
* A* global path planning & optimization
* robot path tracking controller
* RViz visualization

---

# Project Structure

```bash
Intelligent-Robot-Path-Planning-and-Control-on-ROS/
│── src/
│   └── my_map_display/
│       ├── launch/
│       ├── src/
│       ├── include/
│       ├── rviz/
│       ├── CMakeLists.txt
│       └── package.xml
│── .gitignore
│── README.md
```

> `build/` and `devel/` are generated automatically after compilation and are not included in the repository.

---

# Environment Requirements

* Ubuntu 20.04
* ROS Noetic
* CMake
* catkin workspace
* RViz

---

# Build and Run the Project

## Step1: Compile the workspace:

```bash
cd ~/robots
catkin_make
```

## Step2: Source the environment:

```bash
source ~/robots/devel/setup.bash
```

## Step3: Run the Project

Launch the full system:

```bash
roslaunch my_map_display demo.launch
```

This launch file starts:

* map publisher
* occupancy grid generator
* path planner
* robot controller
* robot visualization node
* RViz

---

# System Modules

## 1. Map Generation

Publishes a predefined obstacle map as point cloud data.

## 2. Occupancy Grid Construction

Converts point cloud into occupancy grid for planning.

## 3. Global Path Planning

Uses A* algorithm to generate a collision-free path from start point to goal point.

## 4. Robot Path Tracking

Robot sequentially tracks planned path points using velocity control.

## 5. Robot Visualization

Displays robot pose and trajectory in RViz.

## 6. RViz Visualization

Visualizes:

* obstacle point cloud
* occupancy grid
* planned path
* robot marker
* robot trajectory

⚠️ RViz Fixed Frame must be set to:

```bash
world
```

---

# Current Implemented Functions

* static map publishing
* obstacle visualization
* occupancy grid generation
* A* path planning
* single-shot path publishing
* robot path tracking controller
* launch synchronization

---

# Planning Algorithm

Currently implemented:

* A* algorithm

Planned extensions:

* path smoothing
* trajectory optimization
* local replanning

---

# Motion Control

Current controller features:

* waypoint tracking
* heading correction
* minimum velocity protection
* angular velocity saturation

---

# Future Work

Future development will include:

* virtual target point tracking
* improved path smoothness
* dynamic obstacle avoidance
* local planner integration
* full navigation stack compatibility

---



# Notes

Before running:

* compile all ROS packages successfully
* source workspace environment
* ensure RViz frame is set to `world`

---


ROS Navigation Project for Intelligent Mobile Robot Research
