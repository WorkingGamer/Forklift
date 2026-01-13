# Autonomous Forklift: Perception & Control Stack
### **Siemens-Mentored Industrial R&D Project**

## 📌 Project Overview
An industrial-grade autonomous forklift designed for warehouse material handling. This project implements a full autonomous stack—from raw sensor fusion to high-level path following and automated parking—mentored by Siemens engineers to ensure industrial relevance.

> **Key Achievement:** Successfully architected a multi-board distributed computing system to handle intensive CV and Navigation tasks in real-time.

---

## 🛠 System Architecture & Hardware
To ensure real-time performance, the system utilized a distributed ROS architecture:

* **Primary Compute (NVIDIA Jetson TX2):** Managed the main Autonomous Navigation Stack, Path Planning, and Control.
* **Vision Compute (NVIDIA Jetson Nano):** Dedicated to the CV module (Box detection & pose estimation) to offload computational intensity from the main board.
* **Low-Level Control (3x STM32 Blue Pill):**
    * **ECU 1:** Motorized Fork Control (Interrupt-driven PID).
    * **ECU 2:** Steering Control.
    * **ECU 3:** Drive Speed Control.
* **Sensor Suite:** 2D LiDAR (YDLidar-X4), Stereo Depth Camera (Kinect v1), Drive/Steer Encoders, MPU-9250 Accelerometer (via Arduino buffer), and IR lift sensors.

---

## 🚀 Key Modules (Developed from Scratch)

### 1. Advanced Control & Motion Logic (Lead Developer)
Unlike standard packages, we developed a custom Control Module that implements:
* **Pure Pursuit Algorithm:** High-accuracy path tracking with dynamic look-ahead distance.
* **Velocity Scaling:** Automatic speed reduction based on steering angle to mimic real vehicle dynamics.
* **Re-orientation Maneuvers:** Intelligent decision-making for forward vs. backward drive re-positioning.
* **Automated Parking:** High-precision settlement algorithm for final goal docking.

### 2. Perception & Computer Vision
* **Optimized Detection:** Implemented a lightweight CNN-based detector optimized for Jetson hardware.
* **Pose Estimation:** Combined classical CV with CNN classifiers to determine pallet orientation in 3D space using stereo depth data.
* **Obstacle Handling:** Dynamic cost-map updates using fused LiDAR and Camera data for real-time path re-planning.

### 3. Localization & Sensor Fusion
* Implemented **Extended Kalman Filter (EKF)** to fuse Wheel Odometry and IMU data, significantly reducing drift in warehouse environments.
* Utilized **G-mapping and AMCL** for robust mapping and localization in unknown environments.

---

## 🎮 Simulation & Optimization
Because physical testing is time-consuming, we invested heavily in the Simulation Module:

* **Gazebo Physics Optimization:** We optimized the Gazebo physics engine (Real-Time Update Rate) to ensure the simulation ran smoothly on our development hardware while maintaining accurate friction and inertial properties for the forklift.
* **URDF Model Stability:** We spent significant time fine-tuning the URDF inertial parameters to prevent model collapse or jitter during high-torque movements.
* **Sim-to-Real Validation:** We used the simulation to validate our Emergency Braking Protocols and obstacle avoidance logic before deploying them to the physical Jetson boards.

---

## 📂 Repository Structure
* `/control`: Custom Pure Pursuit and vehicle state management nodes.
* `/perception`: CV module for box detection and orientation.
* `/firmware`: STM32 (C++/HAL) and Arduino source code for low-level PID control.
* `/simulation`: URDF models and Gazebo environments.

---

## 🎓 Thesis Context
This project was part of a Mechatronics Engineering Graduation Thesis at **Ain Shams University**. The full technical details regarding the physics engine optimization and system characterization can be found in the accompanying documentation.
