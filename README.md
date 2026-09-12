# 🐙 IRON-SQUID ⚙️
### Triradial Tendon-Driven Robotic Locomotion Platform

<p align="center">

<img src="https://img.shields.io/badge/ROS_2-Jazzy-blue?style=for-the-badge&logo=ros" />
<img src="https://img.shields.io/badge/Gazebo-Harmonic-orange?style=for-the-badge&logo=gazebo" />
<img src="https://img.shields.io/badge/Python-3.x-yellow?style=for-the-badge&logo=python" />
<img src="https://img.shields.io/badge/OpenSCAD-Parametric-red?style=for-the-badge" />
<img src="https://img.shields.io/badge/Robot-Tendon%20Driven-purple?style=for-the-badge" />

</p>

<p align="center">
  <b>🐙 A biomimetic, tendon-driven robotic platform for omnidirectional locomotion.</b>
</p>

<p align="center">
  Procedural CAD → Dynamic URDF → ROS 2 Control → Gazebo Physics → CPG Locomotion
</p>

---
https://github.com/user-attachments/assets/71cd2311-541b-48e9-b2af-36c75dae24e7



<img width="654" height="630" alt="image" src="https://github.com/user-attachments/assets/379650fb-ad61-41a9-9129-136add7131fc" />




## 🧠 Overview

**IRON-SQUID** is a ROS 2 + Gazebo simulation framework for a **triradial tendon-driven robotic platform** inspired by soft-bodied marine organisms.

Instead of using conventional rigid robotic legs, the platform uses **three flexible robotic arms**, each actuated through multiple tendons.

The objective is to investigate:

- 🐙 Biomimetic locomotion
- 🧵 Tendon-driven continuum mechanisms
- 🧮 Custom inverse kinematics
- 🌊 Central Pattern Generators (CPG)
- 🔄 Symmetric rowing gaits
- 🧭 Omnidirectional motion
- ⚙️ Physics-based robotic simulation
- 🤖 Multi-arm coordination
- 🧪 Rapid simulation-to-hardware development

The entire robot is generated parametrically, allowing mechanical dimensions, arm geometry, joint limits and simulation parameters to be modified without manually rebuilding the complete robot description.

---

# ✨ Key Features

## 🐙 Triradial Robot Architecture

The robot consists of **three independently controlled flexible arms** arranged radially around a central body.

```text
                         ARM 1
                           ▲
                           │
                           │
                    ┌──────┴──────┐
                    │             │
                    │    HEAD     │
                    │             │
                    └──────┬──────┘
                         ╱   ╲
                       ╱       ╲
                     ▼           ▼
                  ARM 2       ARM 3
```

Each arm can generate independent bending patterns while the central controller coordinates all three arms for locomotion.

---

# 🧵 Tendon-Driven Kinematics

Each arm is modeled using a multi-tendon bending system.

The local bending state is represented using two variables:

```text
bend_up
bend_left
```

These are converted into three tendon displacements using a custom IK model.

For a three-tendon arm:

```text
S₁ = bend_up

S₂ = -0.5 · bend_up
     + 0.866 · bend_left

S₃ = -0.5 · bend_up
     - 0.866 · bend_left
```

This allows a desired 2D bending direction to be transformed into individual tendon commands.

```text
                 Bend Direction
                       ▲
                       │
                       │
              ┌────────┴────────┐
              │       ARM       │
              │                 │
              │   ●         ●   │
              │       ●         │
              └─────────────────┘
                 │     │     │
                S₁    S₂    S₃
```

---

# 🌊 Biomimetic Locomotion

IRON-SQUID uses **Central Pattern Generator (CPG)** inspired control to generate periodic arm motion.

Instead of commanding every joint individually, high-level velocity commands are transformed into coordinated arm gaits.

```text
             / cmd_vel
                │
                ▼
        ┌─────────────────┐
        │  Motion Planner │
        └────────┬────────┘
                 │
                 ▼
        ┌─────────────────┐
        │      CPG        │
        │  Gait Generator │
        └────────┬────────┘
                 │
          ┌──────┼──────┐
          ▼      ▼      ▼
        ARM 1  ARM 2  ARM 3
          │      │      │
          ▼      ▼      ▼
        Tendon Tendon Tendon
          │      │      │
          └──────┼──────┘
                 ▼
              ROBOT
```

The controller supports different coordination patterns for:

- ➡️ Forward / backward translation
- ↔️ Lateral translation
- ⬆️ / ⬇️ Vertical motion
- 🔄 Yaw
- ↪️ Roll
- ↕️ Pitch

---

# 🛶 Symmetric Rowing Gait

For translational motion, the robot can coordinate multiple arms as opposing paddles.

For example, an X-axis gait can use:

```text
                  ARM 1
               ─── ↑ ↓ ───
                    │
                    │
              ┌─────┴─────┐
              │   HEAD    │
              └─────┬─────┘
                   ╱   ╲
                ↻         ↺
              ARM 2     ARM 3
```

Opposing arm motions are designed to reduce unwanted rotational forces while producing a net translational component.

The gait architecture is intentionally modular so that different propulsion strategies can be tested without changing the robot model.

---

# 🧩 Modular Simulation Architecture

The repository separates the robot into several layers.

```text
                 ┌──────────────────┐
                 │     OpenSCAD      │
                 │ Parametric Assets │
                 └────────┬─────────┘
                          │
                          ▼
                 ┌──────────────────┐
                 │   Python URDF    │
                 │    Generator     │
                 └────────┬─────────┘
                          │
                          ▼
                 ┌──────────────────┐
                 │       URDF       │
                 │ Robot Structure  │
                 └────────┬─────────┘
                          │
                          ▼
              ┌────────────────────────┐
              │     Gazebo Harmonic    │
              │   Physics Simulation   │
              └───────────┬────────────┘
                          │
                          ▼
              ┌────────────────────────┐
              │     ros2_control       │
              │  Joint Command Layer   │
              └───────────┬────────────┘
                          │
                          ▼
              ┌────────────────────────┐
              │     Motion / CPG       │
              │      Controller        │
              └────────────────────────┘
```

---

# 🏗️ Procedural Robot Generation

The robot description is not manually written joint-by-joint.

Instead, the complete three-arm robot is generated programmatically.

```text
OpenSCAD
   │
   ├── Head
   ├── Arm segments
   └── Mechanical geometry
          │
          ▼
       STL Meshes
          │
          ▼
      squid.py
          │
          ▼
      squid.urdf
          │
          ▼
     Gazebo Robot
```

This makes it easy to change:

- Number of sections
- Segment length
- Segment radius
- Head dimensions
- Joint limits
- Arm placement
- Mesh scaling
- Physical parameters

---

# 🦾 Robot Configuration

Current full robot architecture:

| Parameter | Value |
|---|---:|
| Number of arms | 3 |
| Sections / arm | 20 |
| Pitch joints / arm | 20 |
| Yaw joints / arm | 20 |
| Total controlled joints | 120 |
| Tendons / arm | 3 |
| Total tendon commands | 9 |
| Control loop | 30 Hz |
| CPG stroke frequency | 1 Hz |
| Circular gait radius | 60 mm |
| Maximum tendon command | 80 mm |

The exact mechanical parameters can be modified through the generator and controller configuration.

---

# 🎮 High-Level Control

The main locomotion interface uses the standard ROS 2:

```text
geometry_msgs/msg/Twist
```

Commands are received through:

```text
/cmd_vel
```

Example:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 1.0, y: 0.0, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

This creates a high-level X-axis motion request.

The controller converts the command into:

```text
cmd_vel
   ↓
CPG
   ↓
Arm bending
   ↓
Tendon IK
   ↓
9 tendon commands
   ↓
3 robotic arms
```

---

# 📡 ROS 2 Interfaces

## Input

```text
/cmd_vel
```

Message:

```text
geometry_msgs/msg/Twist
```

---

## Output

```text
/servo_commands
```

Message:

```text
std_msgs/msg/Float64MultiArray
```

The nine values represent:

```text
[ A1_T1, A1_T2, A1_T3,
  A2_T1, A2_T2, A2_T3,
  A3_T1, A3_T2, A3_T3 ]
```

Where:

```text
A1 = Arm 1
A2 = Arm 2
A3 = Arm 3

T1/T2/T3 = three tendons
```

---

# ⚙️ ROS 2 Control

The full robot uses:

```text
ros2_control
```

with:

```text
gz_ros2_control
```

The three arms can be controlled independently:

```text
/controller_manager

/arm1_controller
/arm2_controller
/arm3_controller

/joint_state_broadcaster
```

This separation allows individual arms to be tested before running the complete robot.

---

# 🧪 Simulation Modes

IRON-SQUID supports two primary simulation configurations.

## 🐙 Full Robot

Launch the complete three-arm platform:

```bash
ros2 launch octopus_description bot.launch.py
```

This loads:

- Full robot URDF
- Three arms
- Central body
- Gazebo world
- ros2_control
- Arm controllers
- Joint state broadcaster

---

## 🦾 Single Arm

For debugging and kinematic development:

```bash
ros2 launch octopus_description sim.launch.py
```

This is useful for testing:

- Tendon IK
- Joint limits
- Segment spacing
- Workspace
- Cartesian motion
- Individual arm behavior

---

# 📁 Repository Structure

```text
iron-squid/
│
├── CMakeLists.txt
├── package.xml
├── README.md
│
├── config/
│   ├── bot.yaml
│   └── arm_controllers.yaml
│
├── launch/
│   ├── bot.launch.py
│   └── sim.launch.py
│
├── meshes/
│   ├── *.scad
│   └── *.stl
│
├── scripts/
│   ├── space_walker.py
│   ├── servo_controller.py
│   ├── motion.py
│   └── cartesian_motion.py
│
├── urdf/
│   ├── squid.py
│   ├── cad2urdf.py
│   ├── squid.urdf
│   └── tapered_octopus_arm.urdf
│
└── worlds/
    └── oct.sdf
```

---

# 🛠️ Installation

## Requirements

Recommended environment:

```text
Ubuntu
ROS 2 Jazzy
Gazebo Harmonic
Python 3
OpenSCAD
ros2_control
gz_ros2_control
```

---

## 1. Clone the Repository

```bash
cd ~/ros2_ws/src

git clone \
https://github.com/rubikscube05/iron-squid.git \
octopus_description
```

---

## 2. Install Dependencies

```bash
sudo apt update

sudo apt install \
ros-jazzy-ros2-control \
ros-jazzy-ros2-controllers \
ros-jazzy-gz-ros2-control \
ros-jazzy-gz-ros2-control-demos
```

---

## 3. Build

```bash
cd ~/ros2_ws

colcon build \
--packages-select octopus_description \
--symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

---

# 🧬 Generate Robot Models

The full robot URDF is generated from:

```text
urdf/squid.py
```

Run:

```bash
python3 src/octopus_description/urdf/squid.py
```

For a single arm:

```bash
python3 src/octopus_description/urdf/cad2urdf.py
```

Validate the generated URDF:

```bash
check_urdf \
src/octopus_description/urdf/squid.urdf
```

---

# 🚀 Running the Simulation

## Start Full Robot

```bash
cd ~/ros2_ws

source install/setup.bash

ros2 launch octopus_description bot.launch.py
```

---

## Start Locomotion Controller

In another terminal:

```bash
source ~/ros2_ws/install/setup.bash

python3 \
~/ros2_ws/src/octopus_description/scripts/space_walker.py
```

---

# 🎮 Movement Examples

## Forward X

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 1.0, y: 0.0, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

---

## Backward X

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: -1.0, y: 0.0, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

---

## Lateral Y

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 1.0, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

---

## Vertical Z

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 1.0},
  angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

---

## Yaw

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: 1.0}}" -1
```

---

## Roll

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0},
  angular: {x: 1.0, y: 0.0, z: 0.0}}" -1
```

---

## Pitch

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0},
  angular: {x: 0.0, y: 1.0, z: 0.0}}" -1
```

---

# 🔍 Useful Debugging Commands

Check controllers:

```bash
ros2 control list_controllers
```

Check joint states:

```bash
ros2 topic echo /joint_states
```

Check tendon commands:

```bash
ros2 topic echo /servo_commands
```

Check controller command topics:

```bash
ros2 topic info /arm1_controller/commands -v
```

```bash
ros2 topic info /arm2_controller/commands -v
```

```bash
ros2 topic info /arm3_controller/commands -v
```

Check available ROS topics:

```bash
ros2 topic list
```

---

# 🧮 Kinematic Pipeline

The complete mathematical pipeline is:

```text
                    Twist Command
                         │
                         ▼
                 Desired Velocity
                         │
                         ▼
                CPG / Gait Generator
                         │
              ┌──────────┼──────────┐
              ▼          ▼          ▼
            Arm 1      Arm 2      Arm 3
              │          │          │
              ▼          ▼          ▼
          Local Bend  Local Bend  Local Bend
              │          │          │
              ▼          ▼          ▼
             IK         IK         IK
              │          │          │
          ┌───┴───┐  ┌───┴───┐  ┌───┴───┐
          │ │ │   │  │ │ │   │  │ │ │   │
         T1 T2 T3     T1 T2 T3     T1 T2 T3
          │ │ │       │ │ │       │ │ │
          └─┴─┴───────┴─┴─┴───────┴─┴─┴
                         │
                         ▼
                    ROS 2 Control
                         │
                         ▼
                       Gazebo
```

---

# 🧠 Design Philosophy

IRON-SQUID follows a simple principle:

> **Command the behavior, not every joint.**

Instead of manually specifying hundreds of joint trajectories, the controller operates at a higher abstraction level.

```text
HIGH LEVEL
     │
     │  "Move +X"
     ▼
LOCOMOTION
     │
     │  CPG / gait
     ▼
ARM BENDING
     │
     │  local coordinates
     ▼
TENDON IK
     │
     │  3 tendon commands
     ▼
JOINT CONTROL
     │
     ▼
PHYSICS
```

This makes the system easier to scale from simulation to a physical tendon-driven mechanism.

---

# 🔬 Development Goals

The project is intended as a platform for experimenting with:

### Kinematics

- Continuum robot modeling
- Tendon-driven IK
- Workspace mapping
- Multi-arm coordination
- Jacobian-based control

### Locomotion

- CPG locomotion
- Traveling waves
- Rowing gaits
- Paddle-based propulsion
- Symmetric force generation
- Omnidirectional locomotion

### Simulation

- Gazebo physics
- ros2_control
- Contact dynamics
- Joint constraints
- Collision modeling
- Simulation-to-real testing

### Hardware

Future hardware integration can replace simulated actuators with:

```text
ROS 2
  │
  ▼
Servo Controller
  │
  ▼
Motor / Servo Driver
  │
  ▼
Tendon Actuation
  │
  ▼
Continuum Arm
```

---

# 🗺️ Roadmap

## ✅ Completed

- [x] Parametric OpenSCAD assets
- [x] Procedural URDF generation
- [x] Three-arm robot architecture
- [x] Tendon IK
- [x] Gazebo simulation
- [x] ROS 2 control integration
- [x] Individual arm simulation
- [x] Full robot simulation
- [x] CPG-based motion controller
- [x] `/cmd_vel` interface
- [x] Multi-arm control architecture

## 🚧 In Development

- [ ] Improved omnidirectional gait generation
- [ ] Physically optimized rowing gait
- [ ] Automatic gait calibration
- [ ] Contact-aware locomotion
- [ ] Better continuum-arm dynamics
- [ ] Force/torque feedback
- [ ] Closed-loop locomotion

## 🔮 Future

- [ ] Real tendon-driven hardware
- [ ] Servo/motor integration
- [ ] Embedded controller
- [ ] IMU feedback
- [ ] Motion tracking
- [ ] Simulation-to-real transfer
- [ ] Autonomous navigation
- [ ] Reinforcement-learning locomotion
- [ ] Learned CPG parameters
- [ ] Vision-based navigation

---

# 📊 Project Architecture

```text
             ┌─────────────────────────┐
             │      USER / AUTONOMY    │
             └────────────┬────────────┘
                          │
                       /cmd_vel
                          │
                          ▼
             ┌─────────────────────────┐
             │    SPACE WALKER / CPG   │
             └────────────┬────────────┘
                          │
                   2D Arm Bending
                          │
                          ▼
             ┌─────────────────────────┐
             │       Tendon IK         │
             └────────────┬────────────┘
                          │
                     9 Commands
                          │
                          ▼
             ┌─────────────────────────┐
             │    SERVO CONTROLLER     │
             └────────────┬────────────┘
                          │
                   120 Joint Commands
                          │
                          ▼
             ┌─────────────────────────┐
             │      ROS 2 CONTROL      │
             └────────────┬────────────┘
                          │
                          ▼
             ┌─────────────────────────┐
             │    GAZEBO HARMONIC      │
             │      PHYSICS ENGINE     │
             └─────────────────────────┘
```

---

# 🧪 Research Potential

IRON-SQUID can serve as a testbed for research in:

- Continuum robotics
- Soft robotics
- Biomimetic locomotion
- Tendon-driven mechanisms
- Multi-agent coordination
- Central pattern generators
- Nonholonomic systems
- Holonomic locomotion
- Robot learning
- Sim-to-real transfer
- Physics-based control

The architecture intentionally separates **mechanical design, kinematics, locomotion and low-level control**, making it possible to experiment with one layer without completely rewriting the others.

---

# 🤝 Contributing

Contributions are welcome.

Potential areas include:

- New locomotion gaits
- Better IK algorithms
- CPG implementations
- Gazebo plugins
- Physical actuator interfaces
- CAD improvements
- Visualization tools
- Robot learning
- Simulation-to-real methods

### Development workflow

```bash
git clone <repository>

cd iron-squid

git checkout -b feature/my-new-gait

# Make changes

colcon build

# Test in Gazebo

git add .

git commit -m "Add new locomotion gait"

git push
```

---

# 📜 License

MIT License

Copyright (c) 2026 Vaibhav Vishwakarma

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

# 👨‍🔬 Project

**IRON-SQUID**

A research and development platform for exploring:

> **🐙 Biomimetic robotic locomotion through tendon-driven continuum mechanisms.**

---

<p align="center">

### 🐙 Build. Simulate. Bend. Move.

**IRON-SQUID ⚙️**

</p>
