# Hexakinetica Motor Configurator 🤖

![License](https://img.shields.io/badge/license-AGPLv3-blue.svg)
![Standard](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![Platform](https://img.shields.io/badge/platform-Linux-lightgrey.svg)
![Status](https://img.shields.io/badge/status-Active-green.svg)

**Hexakinetica Motor Configurator** is a high-performance C++ application designed for configuring, testing, and controlling industrial robot motors. This software is an integral part of the **Hexakinetica** project and is specifically used for configuring the motors of the **HexaArm Medium**, **HexaArm Medium Pro**, and **HexaArm Mini pro** industrial robot series.

[HexaLabs MotorConfigurator](MotorConfigurator.jpg)
> 🎥 **Watch on YouTube**: [Demo Video](https://www.youtube.com/watch?v=gU8G4GgxzSM)


## ✨ Key Features

*   **Motor Configuration**: Complete setup and tuning capabilities for HexaArm series robot motors.
*   **EtherCAT Communication**: High-speed, real-time communication with EtherCAT servo drives, complete with Distributed Clocks (DC) synchronization.
*   **Motion Control**: Support for Cyclic Synchronous Position (CSP) and Cyclic Synchronous Velocity (CSV) modes.
*   **Real-time Telemetry and Diagnostics**: Live monitoring of motor states, positions, velocities, and hardware errors through an intuitive UI.
*   **Parameter Management**: Seamless reading and writing of motor parameters via EtherCAT CoE (CAN application protocol over EtherCAT).

## 🏗 Architecture

The project is modularly structured to ensure high reliability and easy maintenance:

### 1. Drivers Layer (`MotorTester_app/drivers`)
Hardware abstraction and protocol implementations.
*   **EtherCAT Master**: Core real-time communication interface.
*   **Axis Adapters**: Low-level abstractions for managing individual drives.

### 2. Motion Core (`MotorTester_app/motion_core`)
Real-time control algorithms and trajectory generation.
*   **Trajectory Generators**: Interpolation for CS axis modes.
*   **Control Loops**: Management of state machines and synchronized motion batches.

### 3. User Interface (`MotorTester_app/ui`)
Graphical components for telemetry, charts, and interaction.
*   **Axis Workspace**: Individual dashboard for specific motor diagnostics.
*   **Parameter Editor**: Tree-view for modifying drive Object Dictionary.

### 4. Applications (`MotorTester_app/apps`)
Ready-to-run executables tying together UI, Core, and Drivers.
.

---

## 🚀 Getting Started

### Prerequisites
*   Linux OS (Ubuntu 22.04 recommended)
*   CMake 3.10+
*   C++17 compatible compiler (GCC/Clang)
*   EtherCAT network interface

### Build
```bash
mkdir -p build && cd build
cmake ../MotorTester_app
cmake --build . --config Release
```

### Run Application

- ⚠️ **Safety Warning:** Ensure the robotic arm is in a safe position and E-stops are accessible before enabling the drives.

To launch the configurator:
```bash
sudo ./build/MotorTester_app
```
*(Root privileges are typically required for raw socket access by the EtherCAT master)*

---

## 🛡 Safety Behavior

The configurator enforces several layers of safety to prevent unintended motion:

1. **Explicit Initialization Sequence**
   - The EtherCAT state machine must explicitly transition through INIT -> PRE-OP -> SAFE-OP -> OP.
2. **Pre-arm Safety Checks**
   - No motion commands are sent until the application is fully armed and communication is verified.
3. **Hardware & Software Limits**
   - The application monitors drive errors and will immediately trigger a safe stop if anomalies are detected.
