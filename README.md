# 🦾 ROS 2 Navigation Assignment – Path Planning, Smoothing & Control

This repository contains all ROS 2 packages developed for the **Autonomous Navigation Assignment**.  
The project implements a full navigation pipeline using ROS 2 nodes for path planning, path smoothing, trajectory generation, and control — enabling smooth and autonomous robot navigation.

---

## 📦 Repository Structure

```
src/
├── my_turtlebot/          # Launch files, URDF, and robot bring-up
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
After setting the pose estimate, click **“2D Goal Pose”** in RViz2 to set a goal visually.  
Alternatively, you can test via terminal with:
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

## 📚 References

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)  
- [TurtleBot3 Tutorials](https://emanual.robotis.com/docs/en/platform/turtlebot3/)  
- [Savitzky–Golay Filter](https://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filter)

---

## 👤 Author

**Aman Sharma**  
B.Tech – Mechanical Engineering  
MNNIT Allahabad
