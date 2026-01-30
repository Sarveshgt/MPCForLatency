<div align="center">

# 🤖 Model Predictive Control for Robots under Communication Latency and Packet Loss

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.8+-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**Robust teleoperation through predictive control in unreliable network conditions**

[📄 Research Paper](assets/MPC_Report_IEEE.docx.pdf) • [🎥 Demo](#demo) • [🚀 Quick Start](#quick-start)

</div>

---

## 🎯 Overview

Modern teleoperation and cloud-based robotic solutions face critical challenges due to **network latency** and **packet loss**. Traditional control algorithms assume perfect communication, leading to unpredictable behavior and potential safety hazards when network conditions degrade.

This project implements a **localized Model Predictive Control (MPC)** system that enables robots to operate smoothly and safely even under adverse network conditions by predicting future states and executing intelligent fallback behaviors.

![MPC System Demo](assets/Untitled.gif)

### ✨ Key Features

- 🎯 **Predictive Control**: MPC-based trajectory planning for smooth operation
- 🌐 **Network Resilience**: Handles latency and packet loss gracefully
- 👻 **Ghost Robot Visualization**: See predicted vs actual robot behavior
- 📊 **Real-time Performance Monitoring**: Track latency, packet loss, and control accuracy
- ⌨️ **Keyboard Teleoperation**: Intuitive control interface
- 🔍 **RViz Integration**: Comprehensive visualization of robot state and predictions

---

## 🧠 Problem Statement

**Challenge**: Networked robotic systems relying on teleoperation or cloud-based control are fundamentally constrained by the reliability of the underlying communication network. Traditional controllers typically assume continuous command availability and experience significant performance drops, instability, or unpredictable motion when variable latency or "stale" control inputs occur.

Specifically, this research addresses the **"blindness" robots face during stochastic network events**, such as:
- 5G/WiFi cellular handovers
- Variable latency causing delayed commands
- Packet loss during network congestion
- Standard reactive controllers malfunctioning under these conditions

**Solution**: Our localized supervisory MPC framework maintains stable motion by predicting short-term targets and executing locally-optimized control commands, bridging communication gaps without requiring modifications to the external command source.

---

## 🔬 Algorithm Architecture

The system utilizes a **supervisory control architecture** implemented in **ROS 2 Humble** consisting of two primary layers:

### 🌐 Network Fault Injection Middleware

A custom layer that replicates real-world wireless network conditions using a **First-In-First-Out (FIFO) buffer**. It employs burst logic to apply a **2000ms latency spike** to every command packet, simulating intermittent congestion during cellular handovers.

### 🤖 Autonomous Safety Controller (TurtleController)

This node integrates network monitoring with an MPC solver, operating in two modes:

- **Passthrough Mode**: Used during normal communication - commands are directly forwarded
- **MPC Mode**: Activated when the delay flag is raised - uses a history buffer of the last 10 received commands to simulate forward motion and infer a "predicted goal state"

```
┌─────────────────────────────────────────────────────────────┐
│            Network Fault Injection Middleware                │
│         (FIFO Buffer + 2000ms Latency Simulation)           │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│              Network Delay Status Monitor                    │
│                (Detect Communication Gaps)                   │
└────────────────────┬────────────────────────────────────────┘
                     │
           ┌─────────┴─────────┐
           ▼                   ▼
    ┌─────────────┐     ┌─────────────┐
    │  Passthrough │     │  MPC Mode   │
    │     Mode     │     │  (Active)   │
    └─────────────┘     └──────┬──────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │  History Buffer      │
                    │  (Last 10 Commands)  │
                    └──────────┬───────────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │  Goal Prediction     │
                    │  + MPC Optimization  │
                    └──────────┬───────────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │  Safe Trajectory     │
                    │  Execution           │
                    └──────────────────────┘
```

### 📐 System Components

| Component | Description |
|-----------|-------------|
| **Turtle Controller** | Main MPC controller with trajectory optimization using SLSQP solver |
| **Network Delay Node** | FIFO-based middleware simulating 2000ms latency spikes |
| **Ghost Controller** | Visualizes predicted robot behavior during MPC mode |
| **Keyboard Teleop** | User input interface for robot control |
| **Performance Plotter** | Real-time metrics visualization |
| **RViz Visualizer** | 3D visualization of robot state and predictions |

---

## 🔧 Technical Details

### Kinematic Model

The robot is modeled as a **planar unicycle** with state **x = [x, y, θ]** and control inputs **u = [v, ω]**:
- **x, y**: Position in 2D plane
- **θ**: Heading angle
- **v**: Linear velocity
- **ω**: Angular velocity

### MPC Formulation

The discrete-time dynamics are solved using a **forward Euler approximation** with time step Δt = 0.1s:

```
x_{k+1} = x_k + v_k · cos(θ_k) · Δt
y_{k+1} = y_k + v_k · sin(θ_k) · Δt
θ_{k+1} = θ_k + ω_k · Δt
```

### Optimization Approach

The system uses the **Sequential Least Squares Programming (SLSQP)** solver to minimize a cost function balancing:

#### 1. Goal Tracking
```
J_goal = ||p_robot - p_goal||²
```

#### 2. Obstacle Avoidance
Employs an **exponential barrier function** to create a mathematical repulsive force:
```
J_obstacle = exp(-α · d_obstacle)
```
where α controls the strength of repulsion and d_obstacle is the distance to the nearest obstacle.

### Constraints
- **Linear velocity**: |v| ≤ 2.0 m/s
- **Angular velocity**: |ω| ≤ 2.0 rad/s
- **Prediction horizon**: N = 10 steps
- **Safety margin**: 0.3m minimum obstacle clearance

---

## 🖼️ System Visualization

<div align="center">

### Control Interface
<img src="assets/Screenshot 2025-12-19 at 5.56.09 PM.png" width="700px" alt="Control Interface"/>

### Performance Metrics
<img src="assets/Screenshot 2025-12-19 at 5.56.36 PM.png" width="700px" alt="Performance Metrics"/>

</div>

---

## 🚀 Quick Start

### Prerequisites

- **ROS2 Humble** or later
- **Python 3.8+**
- Required Python packages: `numpy`, `scipy`, `matplotlib`

### Installation

```bash
# Clone the repository
git clone https://github.com/siddharths09/MPCForLatency.git
cd MPCForLatency

# Build the ROS2 package
colcon build --packages-select my_turtle_controller

# Source the workspace
source install/setup.bash
```

### Running the System

```bash
# Launch the complete MPC system
ros2 launch my_turtle_controller mpc_system.launch.py

# In a new terminal, start keyboard teleoperation
ros2 run my_turtle_controller keyboard_teleop
```

### Configuration

Adjust network conditions and MPC parameters in the launch file:
- **Latency**: Simulated network delay (ms)
- **Packet Loss**: Percentage of dropped packets
- **MPC Horizon**: Prediction time window
- **Control Frequency**: Update rate (Hz)

---

## 📊 Performance Results

The framework was evaluated in a **ROS 2 simulation (RViz)** comparing a standard teleoperated robot (Red) against the MPC-enabled robot (Blue):

### Standard Robot Performance (Baseline)
- ❌ Upon a **2-second network delay**, the robot immediately dropped to **zero velocity**
- ❌ Failed to execute any motion during communication gap
- ❌ Unpredictable behavior upon reconnection

### MPC Robot Performance (Our Approach)
- ✅ Successfully **masked the lag** by utilizing its **10-step prediction horizon** (1.0s at 10Hz)
- ✅ Maintained a **smooth, continuous trajectory** toward the inferred goal
- ✅ Autonomously navigated away from obstacles during disconnection
- ✅ Maintained a **0.3m safety margin** around obstacles
- ✅ Seamless transition back to passthrough mode upon network recovery

### Key Metrics
- **Latency Tolerance**: Successfully handles 2000ms delay spikes
- **Prediction Horizon**: 10 steps (1.0 second lookahead)
- **Safety Distance**: 0.3m obstacle clearance maintained
- **Control Frequency**: 10Hz update rate

---

## 🎥 Demo

[![Demo Video](https://img.youtube.com/vi/UDsfPFw9gRc/0.jpg)](https://www.youtube.com/watch?v=UDsfPFw9gRc)

**[▶️ Watch the full demo video on YouTube](https://www.youtube.com/watch?v=UDsfPFw9gRc)**

The system demonstrates robust performance across various network conditions:

1. **Normal Operation**: Smooth tracking with minimal latency
2. **High Latency**: MPC predicts and compensates for delays
3. **Packet Loss**: Continues operation using predicted states
4. **Network Recovery**: Seamless transition back to normal control

---

## 🎓 Research Contributions

This work presents several key innovations:

1. **Localized Supervisory Framework**: Developed an MPC layer that maintains stable motion without requiring modifications to the external command source, enabling plug-and-play integration with existing teleoperation systems.

2. **Intent Inference Strategy**: Introduced a novel method to predict short-term targets by simulating the robot's history buffer during "blind" periods, allowing the robot to continue meaningful motion even without fresh commands.

3. **Dynamic Fail-Safe Mechanism**: Demonstrated a minimum viable proof-of-concept for bridging communication gaps in high-latency environments such as:
   - Space exploration and planetary rovers
   - Autonomous vehicle platooning
   - Remote surgery and telemedicine
   - Industrial automation over wireless networks

4. **Real-time Obstacle Avoidance**: Integrated safety-critical obstacle avoidance directly into the MPC optimization, ensuring the robot maintains safe operation even during network failures.

---

## 📚 Research & Documentation

For detailed technical information, algorithm derivations, and experimental results, please refer to our [IEEE-format research paper](assets/MPC_Report_IEEE.docx.pdf).

---

## 👥 Team

<table>
  <tr>
    <td align="center">
      <strong>Aaditya Shrivastava</strong>
    </td>
    <td align="center">
      <strong>Kush Patel</strong>
    </td>
    <td align="center">
      <strong>Sarvesh Samadhan Vichare</strong>
    </td>
    <td align="center">
      <strong>Siddharth Singh</strong>
    </td>
  </tr>
</table>

---

## 🛠️ Technical Stack

- **Framework**: ROS2 (Robot Operating System)
- **Language**: Python 3
- **Optimization**: SciPy (Sequential Quadratic Programming)
- **Visualization**: RViz, Matplotlib
- **Simulation**: TurtleSim

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

---

## 📧 Contact

For questions or collaboration opportunities, please reach out to the team members.

---

<div align="center">

**⭐ Star this repository if you find it helpful!**

Made with ❤️ by the MPC Team

</div>
