# 🦾 ROS 2 Navigation Assignment – Path Planning, Smoothing & Control

This repository contains all ROS 2 packages developed for the **Autonomous Navigation Assignment**.  
The project implements a full navigation pipeline using ROS 2 nodes for path planning, path smoothing, trajectory generation, and control — enabling smooth and autonomous robot navigation.

---

## 📦 Repository Structure

```
src/
├── my_turtlebot/          # Robot bring-up node
├── path_planner/          # A* global planner node
├── path_smoother/         # Savitzky–Golay path smoother
├── trajectory_generator/  # Time parameterizer node
└── robot_controller/      # PD-based motion controller
```

---

## 🧰 How to Use This Repository

Follow these steps to use this repository in your system:

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

## 🚀 How to Run the Project

### 1️⃣ Launch the Full Navigation Stack
Run the following command to start Gazebo, map server, planner, smoother, trajectory generator, controller, and RViz:
```bash
ros2 launch my_turtlebot bringup.launch.py
```

### 2️⃣ Set Pose Estimate in RViz2
Once RViz2 opens, click **“2D Pose Estimate”** and place the robot’s approximate position on the map.  
This allows AMCL to localize the robot properly.

### 3️⃣ Send a Test Goal Pose
After setting the pose estimate,you can give goal pose via terminal using below code(example):
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position: {x: 1.5, y: 0.5, z: 0.0}
  orientation: {z: 0.0, w: 1.0}" -1
```

---

## 🧩 Package Descriptions

### 🟢 **1. my_turtlebot/**
Launches all components together:
- Gazebo world (optional)
- Map server (`nav2_map_server`)
- AMCL localization
- Lifecycle manager
- RViz visualization
- A* planner, smoother, trajectory generator, and controller nodes

### 🔵 **2. path_planner/**
Implements an **A\*** based global planner (`a_star_node`) that plans a safe path between the robot’s position and the goal using the map.

### 🟠 **3. path_smoother/**
Implements the **Savitzky–Golay Smoother** (`savitzky_golay_smoother`) that smooths the A* path to make robot motion continuous and natural.

### 🟣 **4. trajectory_generator/**
Implements the **Time Parameterization Node** (`time_parameterizer_node`) which assigns timestamps to each pose using a constant velocity model.

### 🔴 **5. robot_controller/**
Implements the **PD-based Motion Controller** (`pd_motion_planner_node`) which reads `/time_trajectory` and publishes `/cmd_vel` for robot motion.

---

## 🔄 **Data Flow Overview**

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

## 🧠 Key Concepts Demonstrated

- ROS 2 Node communication and topic remapping  
- Global path planning using A* algorithm  
- Path smoothing using Savitzky–Golay filter  
- Time-parameterized trajectory generation  
- PD-based velocity control for differential-drive robots  
- Integration via ROS 2 launch files  

---

## 🧪 Results

- A* generates collision-free global paths  
- Path smoother refines the trajectory for smooth curvature  
- Controller executes motion precisely along generated trajectory  
- Visualization of every step possible in **RViz2**  

---
---

### Design Choices, Algorithms, and Architectural Decisions**

#### 🧩 **Design Overview**
The project follows a **modular ROS 2 architecture** with each core functionality implemented as an independent node. This promotes scalability, debugging ease, and reusability.

#### 🧠 **Algorithms Used**
- **A\*** for **global path planning** to generate a raw global path to desired goal.
- **Savitzky–Golay filter** for **path smoothing**, to ensure continuous and physically feasible trajectories.
- **Constant-velocity time parameterization** to convert spatial paths into time-aware trajectories.
- **PD Controller** for motion control, ensuring responsive and stable robot movement.

#### 🧱 **Architectural Decisions**
- **ROS 2 Nodes and Topics:** Each processing stage (planner → smoother → trajectory → controller) is a separate node communicating through defined topics.
- **Parameterization:** All major parameters (e.g., speed, window size, Kp/Kd gains) are configurable via ROS 2 parameters.
- **Visualization:** RViz2 integration enables real-time visualization of path, smoothed trajectory, and robot motion.

---

### **2.3. Extension to a Real Robot**

To adapt this project to a **physical TurtleBot or custom robot**, the following steps can be added:

1️⃣ Replace simulated sensors with **real sensor topics** (e.g., `/scan` from LiDAR, `/odom` from encoders).  
2️⃣ Update TF tree for the physical robot frames (`base_link`, `odom`, `map`).  
3️⃣ Integrate with actual **motor drivers** using the `/cmd_vel` topic.  
4️⃣ Calibrate PD gains (`Kp`, `Kd`) for the physical system’s inertia and wheelbase.  
5️⃣ Validate using real-world mapping tools like **SLAM Toolbox** or **Cartographer** to generate maps dynamically.

---

### **2.4. AI Tools Used**

- **ChatGPT :** Used for documentation, explanation,and code generation.
---

### **2.5. Extending to Avoid Obstacles and Improve Navigation**

The current system assumes a static environment and uses a simplified pipeline (A* planner → Smoother → Trajectory → PD Controller).  
To extend this project for **dynamic obstacle avoidance** and **advanced navigation**, the following improvements can be implemented:

1️⃣ **Integrate the Full Nav2 Stack:**  
   Utilize the complete **Navigation2 (Nav2)** architecture instead of standalone nodes.  
   Nav2 provides an ecosystem of planners, controllers, costmaps, and recovery behaviors that handle real-world dynamic environments seamlessly.

2️⃣ **Use Advanced Global Planners:**  
   Replace the simple A* node with sophisticated planners such as:  
   - **Smac Planner (2D Hybrid-A\*)** – produces smoother, kinodynamically feasible paths.  
   - **Smac Lattice Planner** – uses motion primitives that align with the robot’s kinematic constraints, ideal for differential and car-like robots.

3️⃣ **Adopt Advanced Controllers:**  
   Replace the PD controller with model-based controllers like:  
   - **MPPI (Model Predictive Path Integral)** Controller – performs sampling-based optimization to compute smooth, dynamically-feasible velocity commands.  
   - **TEB (Timed Elastic Band)** Controller – optimizes trajectory timing and spacing in real time.

4️⃣ **Enable Dynamic Costmaps:**  
   Activate **Nav2 costmap layers** to represent moving and newly detected obstacles using sensor data (LiDAR / depth camera).  
   - **Obstacle Layer** for dynamic obstacle mapping  
   - **Inflation Layer** to maintain safety margins  
   - **Voxel Layer** for 3D awareness in complex spaces

5️⃣ **Integrate Behavior Trees:**  
   Employ **Nav2 behavior trees (BTs)** for decision-making, recovery, and task orchestration.  
   BTs provide a modular, reactive control structure that can handle obstacle encounters, replanning, and recovery autonomously.

6️⃣ **Incorporate AI-Based Prediction (Optional):**  
   Extend the perception stack with **machine-learning models** to predict obstacle motion and assist local planners in proactively adjusting paths.

---

✅ By combining **Smac Lattice planning**, **MPPI control**, and **Nav2 costmap + BT integration**, the system can evolve into a **fully autonomous, real-time navigation framework** capable of safe and adaptive operation in **dynamic real-world environments**.

---

## 📚 References

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)  
- [TurtleBot3 Tutorials]([https://emanual.robotis.com/docs/en/platform/turtlebot3/](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/))  
- [Savitzky–Golay Filter](https://github.com/ros-navigation/navigation2/blob/main/nav2_smoother/src/savitzky_golay_smoother.cpp)

---

## 👤 Author

**Aman Sharma**  
B.Tech – Mechanical Engineering  
MNNIT Allahabad
