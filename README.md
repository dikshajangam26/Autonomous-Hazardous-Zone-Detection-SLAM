# Autonomous Hazardous Zone Detection Using SLAM

#### **Author/Developer:** Diksha Jangam (Student ID: 2929582)
#### **Course:** Intelligent Robotics
#### **Status:** Completed 

## 1. Project Overview
This project implements an autonomous navigation and safety system for construction site environments using the **Pioneer 3-DX** robot in **Webots**. The system utilizes a custom **SLAM (Simultaneous Localization and Mapping)** engine written in Python to map hazardous zones, differentiate between static obstacles and human workers, and enforce dynamic safety logic.

## 2. System Architecture
The architecture was designed and implemented end-to-end by Diksha Jangam, encompassing the following core components:

### A. Simulation Environment
* **World Design:** Custom "construction-site" environment featuring uneven terrain, static barriers, and dynamic worker models to simulate realistic hazards.
* **Robot Configuration:** Pioneer 3-DX configured with differential-drive kinematics.
* **Sensor Suite:**
    * **LiDAR:** Sick LMS 291 (mounted at 0.13m x, 0.24m z) for mapping.
    * **Vision:** RGB Camera (mounted on front nose) for worker detection.
    * **Inertial:** IMU (Gyroscope + Accelerometer) for heading correction.
* **Noise Modelling:** Gaussian noise (0.01m) injected into LiDAR and sensor jitter ($0.005 rad/s$) applied to IMU to test algorithm robustness.

### B. Custom SLAM Implementation
* **Mapping Engine:** Uses **Bresenham's Line Algorithm** for ray casting and **Log-Odds probabilistic updates** to generate a real-time occupancy grid.
* **Sensor Fusion:** Combines Wheel Odometry with IMU Gyroscope data to correct heading drift caused by terrain slips.
* **State Classification:** Real-time distinction between Occupied (Walls), Free (Safe Space), and Unknown areas.

### C. Hazard Detection & Logic
* **Signatures:**
    * *Static Obstacle:* LiDAR proximity < 0.6m.
    * *Human Worker:* Visual detection + LiDAR proximity < 3.0m.
* **State Machine:** Implemented priority-based logic with hysteresis:
    * *Worker Wait:* 5-second latch to prevent decision flickering.
    * *U-Turn:* 2-second commitment timer for navigation stability.

---

## 3. Contribution Log & Work Breakdown

The following log details the execution of project tasks as verified by the project commit history and source code authorship.

| Module | Task Description | Assigned To | Implemented By |
| :--- | :--- | :--- | :--- |
| **Env Setup** | Webots installation, World Creation, Robot Model | Diksha | **Diksha** |
| **Sensors** | Sensor Selection, Mounting, Noise Calibration | Diksha | **Diksha** |
| **SLAM Core** | Odometry, Mapping Algorithm, Occupancy Grid | Diksha | **Diksha** |
| **Fusion** | IMU + Odometry Data Fusion (Drift Handling) | Shared | **Diksha** |
| **Safety Logic** | Hazard Signatures (Worker vs Wall) | Smrithi* | **Diksha** |
| **Detection** | Visual Detection Integration & State Machine | Smrithi* | **Diksha** |
| **Docs** | Final Report, Documentation, & Visualization | Shared | **Diksha** |

> ***Note:** Tasks originally assigned to partner were completed by Diksha Jangam to ensure project functionality and submission eligibility.*
---

## 4. Technical Specifications
* **Language:** Python
* **Simulator:** Webots
* **Grid Resolution:** 0.1m/pixel 
* **Map Dimensions:** 50m x 50m (500x500 Grid) 
* **Visualization:** Custom `visualize_map` overlay using Webots Display node (Vectorized Numpy Operations).

## 5. Visuals

### Video: Real-Time SLAM & Occupancy Grid Generation**
https://github.com/user-attachments/assets/c350d9db-5f06-47eb-bbdd-42564ea6c62c

This screen recording demonstrates the custom mapping engine in action.
* **Grey Area:** Unexplored/Unknown space (Probability = 0.5).
* **White Area:** Confirmed Free Space (Safety Zone) cleared by the robot's movement.
* **Black Dots:** Detected Static Obstacles (Walls/Barriers) identified by LiDAR.

### Video 2: Dashboard of  Real time Robot Telemetry
https://github.com/user-attachments/assets/4f4c90a0-f447-454b-bc85-830d550adaa4

#### Dashboard Components:
The visualization is divided into four critical quadrants as seen above:
1.  **Live Map (Top-Left):** Tracks the robot's global position (X, Y) and historical path (Blue Line) relative to the start point.
2.  **Obstacle Distances (Top-Right):** Distinct lines monitoring the `Front`, `Left`, and `Right` LiDAR vectors to detect incoming walls or debris.
3.  **Proximity Hazard Level (Bottom-Left):** A dynamic safety graph. The yellow dashed line represents the "Turn Threshold"; if the calculated hazard score (Red Line) crosses this, the robot initiates avoidance maneuvers.
4.  **Current Logic State (Bottom-Right):** Displays the active state of the robot's decision tree, cycling between **Forward**, **Turning**, **Worker Detected**, and **Emergency!** to provide instant feedback on safety triggers.

*The visualization is generated using the custom `visualize_map` function with vectorised Numpy operations to ensure real-time performance.*
