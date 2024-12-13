    #include "viz/visualizer.hpp"
    #include <Eigen/Geometry>  // Ensure Eigen is included for quaternion operations
    #include <cmath>           // For M_PI

    Visualizer::Visualizer()
        : Node("visualizer_node") {
        // Declare and retrieve the "config_name" parameter
        this->declare_parameter<std::string>("config_name", "default_value");  // Provide a default value if needed
        this->get_parameter("config_name", configName_);

        // Get package share directory
        std::string packagePath = ament_index_cpp::get_package_share_directory("viz");

        // Construct the .dae mesh file path
        std::string meshFilePath = packagePath + "/models/" + configName_ + ".dae";

        // Check if the .dae file exists
        std::ifstream meshFileCheck(meshFilePath);
        if (meshFileCheck.good()) {
            meshFile_ = "file://" + meshFilePath;
        } else {
            meshFile_.clear(); // Clear the mesh file path if not found
            RCLCPP_WARN(this->get_logger(), "Mesh file not found: %s", meshFilePath.c_str());
        }

        // Start RViz
        std::string rvizConfigFile = packagePath + "/config/auv.rviz";
        std::string command = "rviz2 -d " + rvizConfigFile + " &";
        system(command.c_str());

        // Create ROS 2 publishers
        markerPublisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        pathPublisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
        tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Create ROS 2 subscriptions
        poseSubscription_ = this->create_subscription<auv_core_helper::msg::PoseStamped>(
            auv_core_helper::topicnames::pose_actual, 1, 
            std::bind(&Visualizer::PoseCallback, this, std::placeholders::_1));

        poseGoalSubscription_ = this->create_subscription<auv_core_helper::msg::PoseStamped>(
            auv_core_helper::topicnames::pose_goal, 1, 
            std::bind(&Visualizer::PoseGoalCallback, this, std::placeholders::_1));

        pathSubscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "planned_path", 1, 
            std::bind(&Visualizer::PathCallback, this, std::placeholders::_1));

        // Timer to publish a default pose if no data is received
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
            if (!firstPoseReceived_) {
                PublishPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            }
        });
    }

    void Visualizer::PoseCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg) {
        firstPoseReceived_ = true;
        PublishPose(msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw);
    }

    void Visualizer::PoseGoalCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped goalTransform;
        goalTransform.header.stamp = this->get_clock()->now();
        goalTransform.header.frame_id = "world";
        goalTransform.child_frame_id = "goal_frame";

        goalTransform.transform.translation.x = msg->x;
        goalTransform.transform.translation.y = msg->y;
        goalTransform.transform.translation.z = msg->z;

        Eigen::Quaterniond quaternion(
            Eigen::AngleAxisd(msg->yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(msg->pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(msg->roll, Eigen::Vector3d::UnitX()));
        goalTransform.transform.rotation.x = quaternion.x();
        goalTransform.transform.rotation.y = quaternion.y();
        goalTransform.transform.rotation.z = quaternion.z();
        goalTransform.transform.rotation.w = quaternion.w();

        tfBroadcaster_->sendTransform(goalTransform);
    }

    void Visualizer::PathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        visualization_msgs::msg::Marker lineStrip;
        lineStrip.header.frame_id = "world";
        lineStrip.header.stamp = this->get_clock()->now();
        lineStrip.ns = "path";
        lineStrip.id = 0;
        lineStrip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        lineStrip.action = visualization_msgs::msg::Marker::ADD;
        lineStrip.scale.x = 0.05; // Line width
        lineStrip.color.a = 1.0;
        lineStrip.color.g = 1.0; // Green color

        for (const auto& poseStamped : msg->poses) {
            geometry_msgs::msg::Point p;
            p.x = poseStamped.pose.position.x;
            p.y = poseStamped.pose.position.y;
            p.z = poseStamped.pose.position.z;
            lineStrip.points.push_back(p);
        }

        visualization_msgs::msg::MarkerArray markerArray;
        markerArray.markers.push_back(lineStrip);
        pathPublisher_->publish(markerArray);
    }

    void Visualizer::PublishPose(double x, double y, double z, double roll, double pitch, double yaw) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "AUV";
        marker.id = 0;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;

        // Create the base quaternion from yaw, pitch, roll for the transform
        Eigen::Quaterniond original_quaternion(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

        // Create the quaternion for the marker's orientation
        Eigen::Quaterniond marker_quaternion = original_quaternion;

        // // Apply additional 90-degree rotation around Z-axis if configName_ is "BlueROV"
        // if (configName_ == "BlueROV") {
        //     // 90 degrees in radians
        //     Eigen::Quaterniond additional_rotation(Eigen::AngleAxisd( M_PI / 2, Eigen::Vector3d::UnitZ()));
        //     // Apply the additional rotation relative to the frame's orientation
        //     marker_quaternion = original_quaternion * additional_rotation;
        // }

        // Set the marker's orientation
        marker.pose.orientation.x = marker_quaternion.x();
        marker.pose.orientation.y = marker_quaternion.y();
        marker.pose.orientation.z = marker_quaternion.z();
        marker.pose.orientation.w = marker_quaternion.w();

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        if (!meshFile_.empty()) {
            marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            marker.mesh_resource = meshFile_;
            marker.mesh_use_embedded_materials = true;
        } else {
            marker.type = visualization_msgs::msg::Marker::CUBE;
        }

        markerPublisher_->publish(marker);

        // Create and send the transform for the frame without modifying the orientation
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "world";
        transform.child_frame_id = "auv_base_link";

        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;
        transform.transform.rotation.x = original_quaternion.x();
        transform.transform.rotation.y = original_quaternion.y();
        transform.transform.rotation.z = original_quaternion.z();
        transform.transform.rotation.w = original_quaternion.w();

        tfBroadcaster_->sendTransform(transform);
    }

    void Visualizer::PublishLightSource(double x, double y, double z) {
        visualization_msgs::msg::Marker lightMarker;
        lightMarker.header.frame_id = "world";
        lightMarker.header.stamp = this->get_clock()->now();
        lightMarker.ns = "light_source";
        lightMarker.id = 1;
        lightMarker.type = visualization_msgs::msg::Marker::SPHERE;
        lightMarker.action = visualization_msgs::msg::Marker::ADD;
        lightMarker.pose.position.x = x + 1.0;
        lightMarker.pose.position.y = y;
        lightMarker.pose.position.z = z + 1.0;
        lightMarker.scale.x = 0.2;
        lightMarker.scale.y = 0.2;
        lightMarker.scale.z = 0.2;
        lightMarker.color.a = 1.0;
        lightMarker.color.r = 1.0;
        lightMarker.color.g = 1.0;
        lightMarker.color.b = 0.5;

        markerPublisher_->publish(lightMarker);
    }
