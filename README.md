# AUV Controller, Simulator, and Visualizer

## 1. Introduction
This ROS2 package provides a comprehensive framework for designing and implementing advanced control architectures for Autonomous Underwater Vehicles (AUVs). It integrates essential components, including a high-fidelity simulator for testing vehicle dynamics and a visualizer for real-time feedback and monitoring. This package is tailored to facilitate both research and operational deployment of AUVs in various underwater applications.

## 2. Software Architecture
The following figure illustrates the software architecture of the package:

![Software Architecture](docs/SoftwareArchitecture.svg)

### Nodes

1. **/joy_node**
   - Reads input from a joystick and publishes joystick commands to the KCL node.

2. **/kcl**
   - KCL stands for Kinematic Control Layer. It consists of a Finite State Machine (FSM) controlled by the interface node through a service. It currently supports the following states:
     - **IDLE**: Sets desired velocities to zero.
     - **HOLD**: Maintains the AUV’s position using a PID controller.
     - **RETURN_HOME**: Drives the AUV to the home pose `[0, 0, 0, 0, 0, 0]` using PID pose control.
     - **JOYSTICK**: Maps joystick input directly to desired velocities.
     - **TRAJECTORY_FOLLOWING**: Allows setting a waypoint and a time to reach the goal. The AUV follows a path based on a 5th-degree polynomial function for its pose, deriving velocities and accelerations.
     - **PATH_FOLLOWING**: In this mode, the AUV follows predefined paths generated using various path-planning algorithms such as Helix3D, Serpentine2D, and Serpentine3D. Upon entering this state, the system selects the appropriate path based on user-defined parameters, initializes PID controllers, and computes the AUV’s desired velocity and orientation. It adjusts dynamically to ensure the vehicle aligns with the path direction and follows it accurately, updating parameters like cross-track and vertical errors to maintain optimal performance. The process concludes once the path is completed or interrupted.

3. **/dcl**
   - Processes the desired pose and velocity from the KCL node and computes the desired forces using Quadratic Programming (QP). If QP fails, it resorts to a pseudo-inverse for control allocation, mapping, and limiting thrust forces. As a safety feature, it subscribes to the KCL state and sets desired forces to zero if the state is IDLE.

4. **/simulator**
   - Simulates the AUV’s behavior based on dynamics governed by Fossen’s equations. It calculates the actual acceleration using a pseudo-inverse and integrates this to determine velocity and pose. The actual values are published as feedback for the KCL, functioning as Software In The Loop (SITL). The simulator also publishes the AUV’s pose to the visualizer at each simulation time step.

5. **/visualizer**
   - Visualizes the AUV’s pose and path in RViz, using markers and mesh models for enhanced visualization. It subscribes to pose and path topics, rendering the current pose, goal pose, and planned path dynamically. If a 3D model is available, it uses it for visualization; otherwise, it defaults to simpler shapes. The node also publishes a default pose if no data is received, ensuring consistent graphical feedback.

6. **/interface**
   - Provides a user interface for interacting with the control system, allowing dynamic state changes, trajectory input, and path configuration. It supports state transitions through manual input, handles service requests for control commands, and enables customization of path-planning parameters such as serpentine and helix configurations. The interface ensures user-defined inputs are validated and processed for smooth system operation.

## 3. Installation

### Prerequisites
Ensure the following dependencies are installed:

1. **ROS2 Humble**
   - Install ROS2 Humble by following the [official documentation](https://docs.ros.org/en/humble/index.html).

2. **qpOASES**
   - Clone and install qpOASES:
     ```bash
     git clone https://github.com/coin-or/qpOASES.git
     cd qpOASES
     mkdir build && cd build
     cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
     make
     sudo make install
     ```

3. **graal_utils**
   - Install Graal LAB utilities using the bash script provided [here](https://bitbucket.org/isme_robotics/graal_utils/src/master/).

4. **EIGEN3**
   - Install Eigen3:
     ```bash
     sudo apt-get install libeigen3-dev
     ```

### Building the Package
After installing all dependencies, build the package as follows:

```bash
# Clone the package repository into your ROS2 workspace
cd ~/ros2_ws/src
git clone git@bitbucket.org:isme_robotics/auv_core_ros2.git

# Build the package
cd ~/ros2_ws
colcon build
```

### Running the Package
Once built, launch the package using one of the following commands:

```bash
ros2 launch x300_core_launch.py
```
or

```bash
ros2 launch BlueROV_core_launch
```

## 4. Author Details

**LinkedIn:** [Youssef Attia](https://www.linkedin.com/in/youssefattia98/)

**Email:** [youssef.attia@edu.unige.it](mailto:youssef.attia@edu.unige.it)

## 5. License
This project is licensed under the MIT License. See the `LICENSE` file for more details.
