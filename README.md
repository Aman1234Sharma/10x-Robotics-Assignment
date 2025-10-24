# 🦾 ROS 2 Navigation Assignment – Path Planning, Smoothing & Control

This repository contains all ROS 2 packages developed for the **Autonomous Navigation Assignment**.  
The packages together implement a complete navigation pipeline — from path planning to smooth trajectory execution — for a differential-drive robot such as **TurtleBot3**.

---

## 📦 Repository Structure

```
src/
├── my_turtlebot/          # Launch files, URDF, and robot bring-up
├── path_planner/          # Generates global paths between start & goal
├── path_smoother/         # Applies Savitzky–Golay filter to smooth the path
├── trajectory_generator/  # Converts smoothed path into timed velocity commands
└── robot_controller/      # Sends velocity commands to move the robot
```

---

## ⚙️ Build Instructions

> Make sure you are inside your workspace (e.g., `~/10x_ws`).

```bash
cd ~/10x_ws
colcon build
source install/setup.bash
```

To verify:
```bash
ros2 pkg list | grep planner
ros2 pkg list | grep smoother
```

---

## 🚀 How to Launch

### 1️⃣ Bring up the robot (URDF + base nodes)
```bash
ros2 launch my_turtlebot bringup.launch.py
```

### 2️⃣ Start full navigation pipeline
```bash
ros2 launch trajectory_generator pipeline.launch.py
```

### 3️⃣ Send a goal pose
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position: {x: 2.0, y: 1.0, z: 0.0}
  orientation: {z: 0.0, w: 1.0}" -1
```

---

## 🧩 Package Summary

| Package | Node | Function |
|----------|------|-----------|
| **path_planner** | `planner_node` | Generates raw global path |
| **path_smoother** | `smoother_node` | Smooths the path using Savitzky–Golay filter |
| **trajectory_generator** | `traj_node` | Generates smooth time-based velocity profiles |
| **robot_controller** | `controller_node` | Controls the differential-drive motion |
| **my_turtlebot** | Launch files | Integrates all nodes with robot description |

---

## 🧠 Key Concepts Demonstrated

- ROS 2 node communication (topics, parameters)
- Path planning & smoothing
- Differential-drive motion control
- Launch file orchestration
- Modular package design

---

## 🧪 Results

- Raw vs. smoothed path visualized in **RViz2**
- Robot follows smoothed trajectories with minimal oscillation  
- End-to-end autonomous motion successfully achieved  

_Example visualization:_  
![Smoothed Path Example](docs/navigation_result.png)

---

## 📚 References

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [TurtleBot3 Tutorials](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [Savitzky–Golay Filter](https://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filter)

---

## 👤 Author

**Sudhanshu Maurya**  
M.Tech – Manufacturing Technology & Automation  
IIT Delhi | FSM – Robotics Intern
