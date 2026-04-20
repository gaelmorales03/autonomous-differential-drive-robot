# Autonomous Differential Drive Robot

An advanced autonomous robotics platform developed with ROS2, NVIDIA Jetson Nano, EKF localization, ArUco computer vision, and hybrid reactive navigation.

Robotics • Autonomous Navigation • Sensor Fusion • Embedded AI

---

## Overview

This project involves an autonomous differential-drive mobile robot capable of navigating unknown environments, avoiding obstacles, localizing itself in real-time, and autonomously reaching multiple targets.

The system was developed using a modular ROS2 architecture and validated in Gazebo and RViz environments.

---

## Core Technologies

* ROS2
* NVIDIA Jetson Nano
* Python
* OpenCV
* LiDAR
* Gazebo
* RViz2
* Extended Kalman Filter (EKF)
* Sensor Fusion
* ArUco Detection

---

## Key Features

- Autonomous waypoint navigation
- Differential drive motion control
- Hybrid Bug0 + Bug2 obstacle avoidance
- EKF pose estimation
- Encoder + camera sensor fusion
- ArUco-based visual corrections
- Real-time ROS2 communication
- Simulation validated in Gazebo

---

##  Final Software Architecture

The final implementation was optimized into two high-level ROS2 nodes for improved synchronization.

### `odom.py`

Responsible for localization and state estimation:

* Wheel encoder odometry
* Extended Kalman Filter (EKF)
* ArUco visual corrections
* Robot pose publishing

### `bug3.py`

Responsible for autonomous navigation:

* Target waypoint handling
* Trajectory generation
* Obstacle avoidance
* Bug navigation logic
* Velocity command publishing

This architecture simplified deployment while maintaining modular functionality.

##  Experimental Results

Four localization experiments were performed with different numbers of ArUco markers:

| ArUcos | Avg Error    |
| ------ | ------------ |
| 5      | 0.149 m      |
| 4      | 0.645 m      |
| 3      | 0.648 m      |
| 0      | Severe drift |

### Conclusion

More visual landmarks significantly improved localization accuracy and reduced odometry drift.

---

## Demo Video

https://youtu.be/akQVxWerSgs

---

##  Technical Documentation

Full engineering report available in: `docs/documentation.pdf`

---

##  Applications

* Warehouse robots
* Autonomous delivery
* Smart factories
* Mobile robotics research
* Industrial automation

---

##  Authors

Gael Alejandro Morales Rodríguez.

Daniel Arturo Mendoza.
