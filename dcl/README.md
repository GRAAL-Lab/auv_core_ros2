# Dynamic Control Layer Package

## Overview
The **Dynamic Control Layer (DCL)** node provides advanced control for underwater vehicles by computing the optimal forces for the thrusters based on desired and actual states. It leverages dynamic modeling, optimization, and ROS 2 communication for real-time control.

### Key Features
- Computes optimal thruster forces using QP (Quadratic Programming).
- Supports 6-DOF (Degrees of Freedom) dynamic modeling.
- Integrates with the `UnderwaterVehicleModel` for accurate dynamics.
- Configurable parameters for vehicle-specific properties.

## Installation

### Prerequisites
To build and use the `Dynamic Control Layer` node, ensure the following dependencies are installed:

- [auv_core_helper](https://bitbucket.org/isme_robotics/auv_core_ros2/src/main/) (helper pkg)
- [Eigen](https://eigen.tuxfamily.org/) (for linear algebra computations)
- [PkgConfig](https://github.com/pkgconf/pkgconf) (for configuration management)
- [ROS 2](https://docs.ros.org/) (tested with Humble)
- [qpOASES](https://github.com/coin-or/qpOASES) (for quadratic programming optimization)

The 6DOF_model library depends on the shared library version of qpOASES. Follow these steps to install qpOASES from its GitHub repository.

```bash
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
mkdir build && cd build
cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
make
sudo make install
```
### Building the Node
   ```bash
   git clone https://bitbucket.org/isme_robotics/auv_core_ros2/src/main/
   colcon build --packages-select dcl
   source install/setup.bash
   ```


## Usage

### Launching the DCL Node
To run the `Dynamic Control Layer` node, use the following command:
```bash
ros2 run dcl dynamic_control_layer_node --ros-args -p config_name:=x300
```

### Parameters
The `Dynamic Control Layer` node supports the following configurable parameters:
- `config_name`: Name of the configuration file used to load vehicle-specific dynamic properties.

### Topics
The `Dynamic Control Layer` node interacts with the following topics:
- **Publishers**:
  - `/forces_desired` ([`std_msgs/msg/Float64MultiArray`](http://docs.ros.org/en/api/std_msgs/html/msg/Float64MultiArray.html)): Publishes the computed thruster forces.

- **Subscriptions**:
  - `/pose_desired` ([`geometry_msgs/msg/PoseStamped`](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html)): Receives the desired pose of the vehicle.
  - `/velocity_desired` ([`geometry_msgs/msg/Twist`](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)): Receives the desired velocity of the vehicle.
  - `/pose_actual` ([`geometry_msgs/msg/PoseStamped`](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html)): Receives the actual pose of the vehicle.
  - `/velocity_actual` ([`geometry_msgs/msg/Twist`](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)): Receives the actual velocity of the vehicle.
  - `/kcl_state` ([`std_msgs/msg/String`](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)): Receives the current state of the vehicle.



## Author Details

**LinkedIn:** [Youssef Attia](https://www.linkedin.com/in/youssefattia98/)

**Email:** [youssef.attia@edu.unige.it](mailto:youssef.attia@edu.unige.it)

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.
