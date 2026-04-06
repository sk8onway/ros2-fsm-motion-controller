# ROS2 FSM-Based Motion Controller (Square Trajectory)

## Reference

This implementation was inspired by the official ROS (Noetic) `draw_square` example, and was adapted to ROS2 with modifications in node structure, state handling, and control logic.

## Overview

This project implements a **Finite State Machine (FSM)-based motion controller** in ROS2 to generate a square trajectory using pose feedback from `turtlesim`.

The goal was not just trajectory generation, but to **understand and debug control logic**, state transitions, and numerical issues (especially angle handling).

---

## System Design

The controller is structured as a **Finite State Machine (FSM)** with the following states:

| State | Description |
|---|---|
| `FORWARD` | Move toward target position |
| `STOP_FORWARD` | Ensure motion stabilization |
| `TURN` | Rotate toward target orientation |
| `STOP_TURN` | Ensure rotational stabilization |

State transitions are explicitly controlled using **goal conditions and stability checks**.

---

## Control Logic

### Position Control

Target computed using:
```
x_goal = x + d * cos(theta)
y_goal = y + d * sin(theta)
```

Transition condition:
```
distance_error < threshold
```

### Orientation Control

Target:
```
theta_goal = theta + π/2
```

Error computed using **angle normalization**:
```
error = atan2(sin(θ_goal - θ), cos(θ_goal - θ))
```

---

## Key Engineering Challenges & Fixes

### 1. Angle Wrapping Issue

**Problem:** Robot performed full rotations instead of 90° turns due to discontinuity at ±π.

**Fix:** Implemented angle normalization:
```cpp
atan2(sin(error), cos(error))
```

### 2. False Goal Detection

**Problem:** New goals were instantly considered reached due to identical initial values.

**Fix:** Introduced guard logic (`g_new_goal`) to delay evaluation.

### 3. Incorrect State Transitions

**Problem:** State changes were triggered by velocity instead of actual goal completion.

**Fix:** Separated:
- Goal detection (position/orientation)
- Stabilization (velocity ≈ 0)

---

## Observations

- Small threshold values significantly affect stability
- Angle normalization is critical for rotational control
- FSM-based design improves clarity and debuggability over implicit logic

---

## How to Run
```bash
colcon build
source install/setup.bash
ros2 run my_robot_cpp draw_square
```

---

## Project Structure
```
my_robot_cpp/
├── src/
│   └── draw_square.cpp
├── CMakeLists.txt
└── package.xml
```

---

## Future Work

- Generalize to N-sided polygon trajectories
- Introduce parameterized control (speed, thresholds, side length)
- Implement PID-based control for smoother motion
- Integrate with Gazebo for physics-based validation

---

## 🧠 Key Takeaways

This project highlights:
- Designing control systems using FSMs
- Debugging numerical issues in robotics (angle discontinuities)
- Structuring ROS2 nodes for predictable behavior

Focus was placed on understanding system behavior and debugging, rather than just achieving motion output.
