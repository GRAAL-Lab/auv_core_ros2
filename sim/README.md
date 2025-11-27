# Simulator package

## Overview
The **Simulator** node provides a simulation framework for underwater vehicles. It simulates the dynamics, kinematics, and interaction of the vehicle with external forces, leveraging the `UnderwaterVehicleModel` library and ROS 2.

### Key Features
- Simulates 6-DOF underwater vehicle dynamics.
- Publishes pose, velocity, and acceleration data.
- Integrates external forces and updates the vehicle's state.
- Supports configurable simulation parameters through ROS 2 parameters.

## Installation

### Prerequisites
To build and use the `Simulator` node, ensure the following dependencies are installed:

- [auv_core_helper](https://bitbucket.org/isme_robotics/auv_core_ros2/src/main/) (helper pkg)
- [Eigen](https://eigen.tuxfamily.org/) (for linear algebra computations)
- [PkgConfig](https://github.com/pkgconf/pkgconf) (for configuration management)
- [RML Library](https://bitbucket.org/isme_robotics/rml/src/master/)
- [ROS 2](https://docs.ros.org/) (tested with Humble)

### Building the Node
   ```bash
   git clone https://bitbucket.org/isme_robotics/auv_core_ros2/src/main/
   colcon build --packages-select sim
   source install/setup.bash
   ```

## Usage

### Launching the Simulator Node
To run the `Simulator` node, use the following command:
```bash
ros2 run sim simulator_node --ros-args -p config_name:=x300 -p params_file:=sim/param/params.yaml

```

### Parameters
The simulator node supports the following configurable parameters:
- `config_name`: Name of the configuration file for dynamics model parameters.
- `current_y_velocity`: Y-axis current velocity in the World frame.
- `current_z_velocity`: Z-axis current velocity in the World frame.
- `simulation_dt`: Simulation time step.

### Topics
The `Simulator` node interacts with the following topics:
- **Publishers**:
  - `/pose_actual` ([`geometry_msgs/msg/PoseStamped`](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html)): Current pose of the vehicle.
  - `/velocity_actual` ([`geometry_msgs/msg/Twist`](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)): Current velocity in the body frame.
  - `/acceleration_actual` ([`geometry_msgs/msg/Twist`](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)): Current acceleration in the body frame.

- **Subscriptions**:
  - `/forces_desired` ([`std_msgs/msg/Float64MultiArray`](http://docs.ros.org/en/api/std_msgs/html/msg/Float64MultiArray.html)): Desired forces for the vehicle.
  - `/kcl_state` ([`std_msgs/msg/String`](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)): Current state of the vehicle.

## Author Details

**LinkedIn:** [Youssef Attia](https://www.linkedin.com/in/youssefattia98/)

**Email:** [youssef.attia@edu.unige.it](mailto:youssef.attia@edu.unige.it)


## License
This project is licensed under the MIT License. See the `LICENSE` file for details.
