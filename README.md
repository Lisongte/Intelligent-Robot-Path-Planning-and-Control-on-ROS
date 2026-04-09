# Intelligent Robot Path Planning and Control on ROS

This is a ROS-based intelligent mobile robot navigation project for **occupancy grid construction, global path planning, path tracking, and visualization in RViz**, and will be applied to real-world robots. This project builds a complete navigation pipeline for a differential-drive mobile robot under ROS.

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

* ROS Noetic based on Ubuntu 20.04.x
* CMake
* catkin workspace
* RViz (for visualization)

---

# Build and Run the Project
**Note: Before running, make sure the project is placed inside your ROS workspace, e.g., ~/robots/src/.**
## Step1: Compile the workspace:

```bash
cd ~/robots
catkin_make
```

## Step2: Source the environment:

```bash
source ~/robots/devel/setup.bash
```
You must source the workspace in every new terminal before running ROS nodes.
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


# Planning Algorithm

Currently implemented:

* A* algorithm

Planned extensions:

* path smoothing
* trajectory optimization
---

# Motion Control

Current controller features:

* waypoint tracking
* heading correction
* minimum velocity protection
* angular velocity saturation

---



# Notes

Before running the project, pre-settings below must be checked:

* You must compile all ROS packages successfully


