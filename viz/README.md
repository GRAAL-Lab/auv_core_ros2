# Visualizer package

## Overview
The **Visualizer** node provides a real-time visualization of underwater vehicle dynamics and planned paths using RViz. It publishes 3D models, paths, and poses to facilitate debugging and demonstration of AUV simulations or real-World operations.

### Key Features
- Visualizes vehicle pose using 3D meshes or primitive shapes.
- Publishes paths and goal positions in RViz.
- Integrates with ROS 2 and TF2 for seamless visualization and transformations.
- Customizable mesh models and RViz configurations.

## Installation

### Prerequisites
To build and use the `Visualizer` node, ensure the following dependencies are installed:

- [auv_core_helper](https://bitbucket.org/isme_robotics/auv_core_ros2/src/main/) (helper pkg)
- [Eigen](https://eigen.tuxfamily.org/) (for linear algebra computations)
- [ROS 2](https://docs.ros.org/) (tested with Humble)
- [RViz 2](https://docs.ros.org/en/rolling/Tutorials/RViz2.html)

### Building the Node
   ```bash
   git clone https://bitbucket.org/isme_robotics/auv_core_ros2/src/main/
   colcon build --packages-select viz
   source install/setup.bash
   ```

## Usage

### Launching the Visualizer Node
To run the `Visualizer` node, use the following command:
```bash
ros2 run viz visualizer_node --ros-args -p config_name:=x300

```

### Parameters
The `Visualizer` node supports the following configurable parameters:
- `config_name`: Name of the configuration file or model used to load specific visual settings.

### Topics
The `Visualizer` node interacts with the following topics:
- **Publishers**:
  - `/visualization_marker` ([`visualization_msgs/msg/Marker`](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html)): Publishes the AUV pose as a 3D marker or mesh.
  - `/visualization_marker_array` ([`visualization_msgs/msg/MarkerArray`](http://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html)): Publishes the planned path as a marker array.

- **Subscriptions**:
  - `/pose_actual` ([`geometry_msgs/msg/PoseStamped`](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html)): Receives the current pose of the vehicle.
  - `/pose_goal` ([`geometry_msgs/msg/PoseStamped`](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html)): Receives the goal pose for the vehicle.
  - `/planned_path` ([`nav_msgs/msg/Path`](http://docs.ros.org/en/api/nav_msgs/html/msg/Path.html)): Receives the planned path of the vehicle.

### RViz Integration
- The node automatically launches RViz with a predefined configuration file (`auv.rviz`).
- Users can customize the configuration by editing the `auv.rviz` file located in the package's `config` directory.

### Mesh Models
- The node uses `.dae` files for 3D visualization of the vehicle. These files should be placed in the `models` directory of the package.
- The `config_name` parameter determines which mesh file is loaded (e.g., `models/<config_name>.dae`).


## Author Details

**LinkedIn:** [Youssef Attia](https://www.linkedin.com/in/youssefattia98/)

**Email:** [youssef.attia@edu.unige.it](mailto:youssef.attia@edu.unige.it)


## License
This project is licensed under the MIT License. See the `LICENSE` file for details.
