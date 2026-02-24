# 🐢 ros_tutorials

> A complete ROS2 learning repository covering the full **turtlesim** simulation engine in C++, three tutorial nodes (keyboard teleop, autonomous square drawing, turtle mimic), and a Python package demonstrating ROS2 services, parameters, and dynamic publishers.

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros)](https://docs.ros.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-informational?logo=c%2B%2B)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-yellow?logo=python)](https://www.python.org/)
[![Qt5](https://img.shields.io/badge/GUI-Qt5-green?logo=qt)](https://www.qt.io/)
[![License](https://img.shields.io/badge/License-BSD--3--Clause-blue.svg)](LICENSE)

---

## 📖 Overview

This repository is a hands-on exploration of core ROS2 concepts built around the classic **turtlesim** simulation. It is structured in two layers:

**Layer 1 — turtlesim core (C++):** A full reimplementation of the turtlesim engine, covering the complete ROS2 communication stack — topics, services, and actions — with a live Qt5 canvas rendering one or more turtles in real time.

**Layer 2 — tutorials (C++ & Python):** Three C++ nodes that drive the turtle autonomously (draw a square, mimic another turtle, keyboard control), plus a Python package (`my_package`) that demonstrates ROS2 services, parameters, and dynamic topic publishing through a temperature conversion example.

---

## ✨ What's Implemented

| Node | Language | What it does |
|------|----------|--------------|
| `turtlesim_node` | C++ | Core simulation — Qt5 canvas, multi-turtle, all services and actions |
| `turtle_teleop_key` | C++ | Keyboard control with arrow keys + absolute rotation via action client |
| `draw_square` | C++ | Autonomous closed-loop square drawing using pose feedback |
| `turtle_mimic` | C++ | Mirrors one turtle's motion onto another via topic remapping |
| `temperature_service` | Python | ROS2 service server — Celsius to Fahrenheit conversion |
| `temperature_client` | Python | ROS2 service client — requests a temperature conversion |
| `parameter_server` | Python | ROS2 parameter server with `conversion_mode` parameter |
| `dynamic_publisher` | Python | Publisher that reads startup parameters and dynamically updates |

---

## 🏗️ System Architecture

```
                        ┌──────────────────────────────────────────────────┐
                        │              turtlesim_node (C++)                │
                        │                                                  │
                        │  Services: /spawn  /kill  /clear  /reset         │
                        │  Params:   background_r/g/b  holonomic           │
                        │                                                  │
                        │  Per turtle <n>:                                 │
                        │    Sub:  cmd_vel  (geometry_msgs/Twist)          │
                        │    Pub:  pose     (turtlesim/msg/Pose)           │
                        │    Pub:  color_sensor (turtlesim/msg/Color)      │
                        │    Srv:  set_pen  teleport_absolute              │
                        │          teleport_relative                       │
                        │    Act:  rotate_absolute                         │
                        └──────────────┬───────────────────────────────────┘
                                       │
              ┌────────────────────────┼────────────────────────┐
              │                        │                        │
   ┌──────────▼──────────┐  ┌──────────▼──────────┐  ┌─────────▼──────────┐
   │  turtle_teleop_key  │  │    draw_square       │  │   turtle_mimic     │
   │  (C++)              │  │    (C++)             │  │   (C++)            │
   │                     │  │                      │  │                    │
   │  Pub: cmd_vel       │  │  Sub: turtle1/pose   │  │  Sub: input/pose   │
   │  Act: rotate_abs    │  │  Pub: turtle1/cmd_vel│  │  Pub: output/      │
   │  Params:            │  │  Cli: /reset         │  │       cmd_vel      │
   │   scale_linear      │  │  State machine:      │  │                    │
   │   scale_angular     │  │   FORWARD→STOP→      │  │  (remapped to      │
   └─────────────────────┘  │   TURN→STOP→...      │  │   turtle2/cmd_vel) │
                            └──────────────────────┘  └────────────────────┘

   ┌──────────────────────────────────────────────────────────────┐
   │                  my_package (Python)                         │
   │                                                              │
   │  temperature_publisher ──/temperature_data──► temperature_   │
   │  (Topic Publisher)        (Topic)              client        │
   │                                               (Service       │
   │                                                Caller)       │
   │                                                   │          │
   │  parameter_server ◄──reads── temperature_unit     │          │
   │  (Parameter Storage)       (Parameter)             │ Request  │
   │                                                   ▼          │
   │                                          /convert_temperature│
   │                                               (Service)      │
   │                                                   │          │
   │                                                   ▼          │
   │                                          temperature_service  │
   │                                          (Service Provider)  │
   └──────────────────────────────────────────────────────────────┘
```

---

## 📦 Repository Structure

```
ros_tutorials/
├── src/
│   ├── turtle.cpp                  # Turtle entity: motion physics, pen, action server
│   └── turtle_frame.cpp            # Qt5 frame: canvas, multi-turtle management
├── tutorials/
│   ├── draw_square.cpp             # Autonomous square-drawing node
│   ├── teleop_turtle_joy.cpp       # Keyboard teleop + rotate_absolute action client
│   └── mimic.cpp                   # Turtle mimic node (pose → cmd_vel bridge)
├── include/turtlesim/              # C++ header files
├── msg/
│   ├── Color.msg                   # RGB color under the turtle
│   └── Pose.msg                    # x, y, theta, linear_velocity, angular_velocity
├── srv/
│   ├── Kill.srv                    # Kill a named turtle
│   ├── Spawn.srv                   # Spawn turtle at x, y, theta
│   ├── SetPen.srv                  # Set pen color, width, on/off
│   ├── TeleportAbsolute.srv        # Jump to absolute pose
│   └── TeleportRelative.srv        # Move by relative offset
├── action/
│   └── RotateAbsolute.action       # Goal: theta | Feedback: remaining | Result: delta
├── launch/                         # Launch files
├── images/                         # Turtle sprites (one per ROS2 distro)
├── my_package/                     # Python tutorial package
│   ├── temperature_service.py      # Service server: Celsius → Fahrenheit
│   ├── temperature_client.py       # Service client: sends conversion requests
│   ├── parameter_server.py         # Parameter server: conversion_mode (CtoF/FtoC)
│   └── dynamic_publisher.py        # Publisher with startup params + dynamic updates
├── CMakeLists.txt
└── package.xml
```

---

## 📨 ROS2 Interface Reference

### turtlesim_node — Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `<turtle>/cmd_vel` | `geometry_msgs/Twist` | Subscribed | Linear and angular velocity |
| `<turtle>/pose` | `turtlesim/msg/Pose` | Published | Position, heading, velocity |
| `<turtle>/color_sensor` | `turtlesim/msg/Color` | Published | RGB pixel under turtle |

### turtlesim_node — Services

| Service | Type | Description |
|---------|------|-------------|
| `/spawn` | `turtlesim/srv/Spawn` | Spawn a turtle at x, y, theta |
| `/kill` | `turtlesim/srv/Kill` | Remove a named turtle |
| `/clear` | `std_srvs/srv/Empty` | Wipe all drawn paths |
| `/reset` | `std_srvs/srv/Empty` | Full simulation reset |
| `<turtle>/set_pen` | `turtlesim/srv/SetPen` | Set pen color, width, on/off |
| `<turtle>/teleport_absolute` | `turtlesim/srv/TeleportAbsolute` | Jump to x, y, theta |
| `<turtle>/teleport_relative` | `turtlesim/srv/TeleportRelative` | Move by relative offset |

### turtlesim_node — Actions

| Action | Type | Description |
|--------|------|-------------|
| `<turtle>/rotate_absolute` | `turtlesim/action/RotateAbsolute` | Rotate to exact heading with live feedback |

### turtlesim_node — Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `background_r` | `69` | Red channel of background (0–255) |
| `background_g` | `86` | Green channel of background (0–255) |
| `background_b` | `255` | Blue channel of background (0–255) |
| `holonomic` | `false` | Enable Y-axis strafe movement |

### turtle_teleop_key — Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `scale_linear` | `2.0` | Linear speed multiplier |
| `scale_angular` | `2.0` | Angular speed multiplier |

### my_package — Parameters

| Node | Parameter | Default | Description |
|------|-----------|---------|-------------|
| `parameter_server` | `conversion_mode` | `CtoF` | Temperature conversion direction (`CtoF` or `FtoC`) |

---

## 🛠️ Building & Running

### Prerequisites

```bash
# ROS2 (Jazzy or Humble) sourced
source /opt/ros/jazzy/setup.bash

# Qt5 development libraries
sudo apt install libqt5widgets5 qtbase5-dev
```

### Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/emins1856/ros_tutorials.git

cd ~/ros2_ws
colcon build --packages-select ros_tutorials my_package
source install/setup.bash
```

---

## 🚀 Running the Nodes

### Start turtlesim

```bash
ros2 run turtlesim turtlesim_node
```

### Keyboard control

```bash
ros2 run turtlesim turtle_teleop_key
```

| Key | Action |
|-----|--------|
| `↑ ↓ ← →` | Move forward/backward, rotate left/right |
| `g b v c d e r t` | Snap to absolute heading (0°, 45°, 90°, 135°, 180°, -135°, -90°, -45°) |
| `f` | Cancel current rotation goal |
| `q` | Quit |

### Autonomous square drawing

```bash
# Terminal 1
ros2 run turtlesim turtlesim_node

# Terminal 2
ros2 run turtlesim draw_square
```

The turtle will reset and autonomously trace a continuous square using a 4-state machine: `FORWARD → STOP_FORWARD → TURN → STOP_TURN → ...`

### Turtle mimic (two turtles)

```bash
# Spawn a second turtle first
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"

# Run mimic with topic remapping so turtle2 mirrors turtle1
ros2 run turtlesim mimic \
  --ros-args \
  --remap input/pose:=turtle1/pose \
  --remap output/cmd_vel:=turtle2/cmd_vel
```

Turtle2 will now mirror every movement of turtle1 in real time.

---

## 🌡️ Python Tutorial Package (my_package)

### Temperature conversion service

```bash
# Terminal 1 — start the service server
ros2 run my_package temperature_service
# [temperature_service]: Temperature conversion service is ready

# Terminal 2 — call the client (converts 25°C)
ros2 run my_package temperature_client
# [temperature_client]: 25C converted to 77F
```

### Parameter server

```bash
# Start the parameter server
ros2 run my_package parameter_server
# [parameter_server]: Parameter Server is running

# List all parameters
ros2 param list
# /parameter_server:
#   conversion_mode

# Get current value
ros2 param get /parameter_server conversion_mode
# String value is: CtoF

# Change conversion direction at runtime
ros2 param set /parameter_server conversion_mode FtoC
# Set parameter successful
```

### Dynamic publisher

```bash
ros2 run my_package dynamic_publisher
# [dynamic_publisher]: Publishing: Hello, ROS2!
# [dynamic_publisher]: Publishing: Hello, ROS2!
# ...
```

---

## 🔍 Key Implementation Details

**`draw_square` — closed-loop state machine:** The node subscribes to `turtle1/pose` and uses position and velocity feedback to precisely sequence four states. Goal positions are computed geometrically from the current pose (`cos(θ) × 2`, `sin(θ) × 2`) ensuring the square scales correctly regardless of starting orientation.

**`teleop_turtle_joy` — action client with preemption:** Arrow keys publish `Twist` messages directly. Letter keys (`g`, `r`, `t`, etc.) send `rotate_absolute` action goals for snap-to-heading rotation. The `f` key sends an async cancel request to the active goal handle. Speed is scaled by the `scale_linear` and `scale_angular` parameters, making it tunable without recompilation.

**`mimic` — decoupled via topic remapping:** The mimic node itself is completely generic — it subscribes to `input/pose` and publishes to `output/cmd_vel`. Coupling it to specific turtles is done entirely through `--remap` arguments at launch, demonstrating ROS2's namespace and remapping system without hardcoded topic names.

**`turtle.cpp` — 1-second velocity watchdog:** If no `cmd_vel` message arrives for more than 1 second, all velocities are zeroed automatically. This prevents runaway motion if a controlling node crashes or disconnects.

**`turtle.cpp` — action preemption:** Receiving a new `rotate_absolute` goal while one is active immediately aborts the old goal before accepting the new one. Incoming `cmd_vel` messages also abort any active rotation goal, with a warning logged.

**`turtle_frame.cpp` — persistent path layer:** Pen strokes are drawn onto a persistent `QImage` that is composited over the background on every Qt paint event, making paths survive across frames until `/clear` is called.

---

## 🐢 Turtle Sprites

A different turtle image is used for each ROS2 distribution, selected randomly on spawn:

`ardent` · `bouncy` · `crystal` · `dashing` · `eloquent` · `foxy` · `galactic` · `humble` · `iron` · `jazzy` · `rolling`

---

## 📚 Tech Stack

| Layer | Technology |
|-------|-----------|
| Robot middleware | [ROS2](https://docs.ros.org/) Jazzy |
| Core language | C++17 |
| Tutorial language | Python 3 (`rclpy`) |
| GUI framework | Qt5 |
| Build system | CMake + `ament_cmake` |
| Actions | `rclcpp_action` |
| Package indexing | `ament_index_cpp` |

---

## Author
Emin Samadov
Software Engineering (Robotics Systems) — Vilnius University
GitHub: github.com/emins1856

---

## 📄 License

Distributed under the **BSD 3-Clause License** — see [LICENSE](LICENSE) for details.
Original turtlesim source © Willow Garage, Inc.
