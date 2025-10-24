# **ROS 2 Navigation Assignment – Path Planning, Smoothing & Control**

This repository contains all ROS 2 packages developed for the **Autonomous Navigation Assignment**.  
The project implements a full navigation pipeline using ROS 2 nodes for path planning, path smoothing, trajectory generation, and control — enabling smooth and autonomous robot navigation.

---
## ⚙️ Important Note

> 🧪 **Tested Environment**
>
> This project has been **tested and verified** on:
> - **ROS 2 Humble Hawksbill**
> - **Gazebo Classic**
> - **Ubuntu 22.04 (Jammy Jellyfish)**

> ⚠️ **Prerequisites**
>
> Before running this repository, make sure you have the following:
> - The `turtlebot3` and `turtlebot3_gazebo` packages installed.
> - The directory structure for the **map** and **launch files** matches the paths defined in the launch file .
>    If your directory paths differ, update them accordingly in your bringup.launch.py launch file.

---

## **1. Repository Structure**

```
src/
├── my_turtlebot/          # Robot bring-up node
├── path_planner/          # A* global planner node
├── path_smoother/         # Savitzky–Golay path smoother
├── trajectory_generator/  # Time parameterizer node
└── robot_controller/      # PD-based motion controller
```

---

## **2. Setup and Execution Instructions**

### **2.1. Setting Up the Workspace**

1️⃣ **Create a ROS 2 workspace**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2️⃣ **Copy this repository's contents**
Copy all folders from this repository (`my_turtlebot`, `path_planner`, `path_smoother`, `trajectory_generator`, `robot_controller`) into your workspace `src/` folder.

3️⃣ **Go back to the workspace root**
```bash
cd ~/ros2_ws
```

4️⃣ **Build the workspace**
```bash
colcon build
```

5️⃣ **Source the setup file**
```bash
source install/setup.bash
```

---

### **2.2. Running the Project**

#### **2.2.1. Launch the Full Navigation Stack**
Run the following command to start Gazebo, map server, planner, smoother, trajectory generator, controller, and RViz:
```bash
ros2 launch my_turtlebot bringup.launch.py
```

#### **2.2.2. Set Pose Estimate in RViz2**
Once RViz2 opens, click **“2D Pose Estimate”** and place the robot’s approximate position on the map.  
This allows AMCL to localize the robot properly.

#### **2.2.3. Send a Test Goal Pose**
After setting the pose estimate, you can give a goal pose via terminal using the command below (example):
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position: {x: 1.5, y: 0.5, z: 0.0}
  orientation: {z: 0.0, w: 1.0}" -1
```

---

## **3. Demonstration Videos **

This section showcases the working of the navigation pipeline and its results in both simulation and visualization environments.

### **3.1. Presenting the System in Action**
[Google Drive Link](https://drive.google.com/file/d/17J8mfkN-oli87akVDsjsR3337qvPXLH7/view?usp=sharing)

---

### **3.2. Smoothning on custom zig zag Path  **
<p align="center">
  <img src="path smoothning.png" alt="Path Smoothing Result" width="600"/>
</p>
---

✅ These demonstrations provide clear evidence of the system’s successful end-to-end operation, from path generation to robot motion execution.

---

## **4. Package Descriptions**

### **4.1. my_turtlebot/**
Launches all components together:
- Gazebo world 
- Map server (`nav2_map_server`)
- AMCL localization
- Lifecycle manager
- RViz visualization
- A* planner, smoother, trajectory generator, and controller nodes

### **4.2. path_planner/**
Implements an **A\*** based global planner (`a_star_node`) that plans a safe path between the robot’s position and the goal using the map.

### **4.3. path_smoother/**
Implements the **Savitzky–Golay Smoother** (`savitzky_golay_smoother`) that smooths the A* path to make robot motion continuous and natural.

### **4.4. trajectory_generator/**
Implements the **Time Parameterization Node** (`time_parameterizer_node`) which assigns timestamps to each pose using a constant velocity model.

### **4.5. robot_controller/**
Implements the **PD-based Motion Controller** (`pd_motion_planner_node`) which reads `/time_trajectory` and publishes `/cmd_vel` for robot motion.

---

## **5. Data Flow Overview**

```
[ /goal_pose ] ─▶ [ A* Planner (/a_star/path) ]
                     │
                     ▼
          [ Savitzky–Golay Smoother (/smoothed_path) ]
                     │
                     ▼
       [ Time Parameterizer (/time_trajectory) ]
                     │
                     ▼
           [ PD Controller (/cmd_vel) ]
                     │
                     ▼
              [ Robot Motion + Goal Reached ]
```

---

## **6. Key Concepts Demonstrated**

- ROS 2 Node communication and topic remapping  
- Global path planning using A* algorithm  
- Path smoothing using Savitzky–Golay filter  
- Time-parameterized trajectory generation  
- PD-based velocity control for differential-drive robots  
- Integration via ROS 2 launch files  

---

## **7. Results**

- A* generates collision-free global paths  
- Path smoother refines the trajectory for smooth curvature  
- Controller executes motion precisely along generated trajectory  
- Visualization of every step possible in **RViz2**  

---

## **8. Design Choices, Algorithms, and Architectural Decisions**

### **8.1. Design Overview**
The project follows a **modular ROS 2 architecture** with each core functionality implemented as an independent node.  
This promotes scalability, debugging ease, and reusability.

### **8.2. Algorithms Used**
- **A\*** for **global path planning** to generate a raw global path to the desired goal.  
- **Savitzky–Golay filter** for **path smoothing**, to ensure continuous and physically feasible trajectories.  
- **Constant-velocity time parameterization** to convert spatial paths into time-aware trajectories.  
- **PD Controller** for motion control, ensuring responsive and stable robot movement.

### **8.3. Architectural Decisions**
- Each processing stage (planner → smoother → trajectory → controller) is modular and communicates through ROS 2 topics.  
- Parameters such as velocity, window size, and PD gains are configurable via launch files.  
- Real-time visualization in RViz2 enables clear understanding of robot state and trajectory.

---

## **9. Extension to a Real Robot**

To adapt this project to a **physical TurtleBot or custom robot**, the following steps can be added:

1️⃣ Replace simulated sensors with **real sensor topics** (e.g., `/scan` from LiDAR, `/odom` from encoders).  
2️⃣ Update TF tree for the physical robot frames (`base_link`, `odom`, `map`).  
3️⃣ Integrate with actual **motor drivers** using the `/cmd_vel` topic.  
4️⃣ Calibrate PD gains (`Kp`, `Kd`) for the physical system’s inertia and wheelbase.  
5️⃣ Validate using **SLAM Toolbox** or **Cartographer** for real-time map creation and localization.

---

## **10. AI Tools Used**

- **ChatGPT** – Assisted in documentation writing, explanation of algorithms, and code generation.  

---

## **11. Extending to Avoid Obstacles and Improve Navigation**

The current system assumes a static environment and uses a simplified pipeline (**A\*** → Smoother → Trajectory → PD Controller).  
To extend this project for **dynamic obstacle avoidance** and **advanced navigation**, the following improvements can be implemented:

### **11.1. Integrate the Full Nav2 Stack**
Use the complete **Navigation2 (Nav2)** architecture instead of standalone nodes.  
Nav2 provides an ecosystem of planners, controllers, costmaps, and recovery behaviors for handling dynamic environments.

### **11.2. Use Advanced Global Planners**
Replace the simple A* node with more sophisticated planners such as:  
- **Smac Planner (2D Hybrid-A\*)** – produces smoother, kinodynamically feasible paths.  
- **Smac Lattice Planner** – uses motion primitives matching the robot’s kinematics.

### **11.3. Adopt Advanced Controllers**
Replace the PD controller with modern, model-based controllers like:  
- **MPPI (Model Predictive Path Integral)** Controller – performs sampling-based optimization for dynamic, smooth control.  
- **TEB (Timed Elastic Band)** Controller – optimizes timing and trajectory spacing in real-time.

### **11.4. Enable Dynamic Costmaps**
Activate **Nav2 costmap layers** for real-time obstacle representation using sensor data (LiDAR or depth camera).  
- **Obstacle Layer** – dynamic obstacle mapping  
- **Inflation Layer** – maintains safe margins  
- **Voxel Layer** – enables 3D environment awareness

### **11.5. Integrate Behavior Trees**
Implement **Nav2 Behavior Trees (BTs)** for high-level decision-making, recovery behaviors, and autonomous task management.

---

✅ By combining **Smac Lattice planning**, **MPPI control**, and **Nav2 costmap + BT integration**, this system can evolve into a **fully autonomous, real-time navigation framework** capable of safe and adaptive operation in **dynamic real-world environments**.

---

## **12. References**

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)  
- [TurtleBot3 Tutorials](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)  
- [Savitzky–Golay Smoother in Nav2](https://github.com/ros-navigation/navigation2/blob/main/nav2_smoother/src/savitzky_golay_smoother.cpp)

---

## **13. Author**

**Aman Sharma**  
B.Tech – Mechanical Engineering  
MNNIT Allahabad
