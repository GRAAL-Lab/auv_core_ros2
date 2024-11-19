#include "states/PathPlanningState.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <filesystem>
#include <limits>

// Constructor
PathPlanningState::PathPlanningState(fsm::FSM* fsm)
    : BaseAUVState(fsm, "PATH_PLANNING") {}

// OnEntry: Called when entering the path planning state
fsm::retval PathPlanningState::OnEntry() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Entering PATH_PLANNING state");

    // Retrieve home path and ensure directory exists
    std::string home_path = futils::get_homepath();
    std::filesystem::create_directories(home_path);

    // Select path planning mode
    if (ctrlData->path_planning_2d_3d == 3) {
        RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Path Planning: 3D Helix");
        path = sisl::PathFactory::NewHelicalPath(
            ctrlData->helix_startPos, ctrlData->helix_axisPos, ctrlData->helix_axisDir,
            ctrlData->helix_frequency, ctrlData->helix_numQuadrants, ctrlData->helix_counterClockwise);
    } else if (ctrlData->path_planning_2d_3d == 1) {
        RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Path Planning: 2D Serpentine");
        if (ctrlData->serpentine_polygon_vertices_.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("PathPlanningState"), "No polygon vertices provided for 2D serpentine path.");
            return fsm::fail;
        }
        auto direction = ctrlData->serpentine_direction_ ? sisl::Path::Direction::Backward : sisl::Path::Direction::Forward;
        path = sisl::PathFactory::NewSerpentine(
            ctrlData->serpentine_angle_, direction, ctrlData->serpentine_offset_,
            ctrlData->serpentine_polygon_vertices_);
    } else if (ctrlData->path_planning_2d_3d == 2) {
        RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Path Planning: 3D Serpentine");
        if (ctrlData->serpentine_polygon_vertices_.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("PathPlanningState"), "No polygon vertices provided for 3D serpentine path.");
            return fsm::fail;
        }
        auto direction = ctrlData->serpentine_direction_ ? sisl::Path::Direction::Backward : sisl::Path::Direction::Forward;
        path = sisl::PathFactory::New3DSerpentine(
            ctrlData->serpentine_angle_, direction, ctrlData->serpentine_offset_,
            ctrlData->serpentine_polygon_vertices_, ctrlData->dive_depth_,
            ctrlData->curvature_, ctrlData->dip_num_points_, ctrlData->dive_length_);
    }

    if (!path) {
        RCLCPP_ERROR(rclcpp::get_logger("PathPlanningState"), "Failed to create path.");
        isCurveSet_ = false;
        return fsm::fail;
    }

    // Sample and convert path to nav_msgs::Path
    auto sampledPointsSharedPtr = path->Sampling(1500);
    if (!sampledPointsSharedPtr || sampledPointsSharedPtr->empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("PathPlanningState"), "Failed to sample points from SISL path.");
        return fsm::fail;
    }

    nav_msgs::msg::Path nav_path;
    nav_path.header.frame_id = "map";
    nav_path.header.stamp = rclcpp::Clock().now();

    for (const auto& point : *sampledPointsSharedPtr) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = point.x();
        pose_stamped.pose.position.y = point.y();
        pose_stamped.pose.position.z = point.z();
        pose_stamped.pose.orientation.w = 1.0; // Identity quaternion
        nav_path.poses.push_back(pose_stamped);
    }
    ctrlData->planned_path_ = nav_path;

    // Save sampled points for debugging or visualization
    PersistenceManager::SaveObj(path->Sampling(30000), home_path + "/path.txt");

    // Initialize PID controllers
    initializePIDControllers();

    // Initialize other member variables
    isCurveSet_ = true;
    last_update_time = std::chrono::system_clock::now();

    RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Path successfully created and vehicle is ready.");
    return fsm::ok;
}

// Execute: Called repeatedly while in the path planning state
fsm::retval PathPlanningState::Execute() noexcept {
    if (!isCurveSet_) {
        RCLCPP_WARN(rclcpp::get_logger("PathPlanningState"), "Curve not set. Waiting...");
        return fsm::ok;
    }

    if (!isVehicleOnPathDirection) {
        // Align vehicle with initial path direction
        alignVehicleToPath();
        return fsm::ok;
    }

    // Update vehicle's goal position and orientation based on the path
    updateVehicleGoal();

    return fsm::ok;
}

// OnExit: Called when exiting the path planning state
fsm::retval PathPlanningState::OnExit() noexcept {
    ctrlData->planned_path_.poses.clear();
    isVehicleOnPathDirection = false;
    closestPointAbscissa = 0.0;
    currentAbscissa_ = 0.0;
    RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Exiting PATH_PLANNING state.");
    return fsm::ok;
}

// Initialize PID controllers
void PathPlanningState::initializePIDControllers() {
    ctb::PIDGains gainsX = {ctrlData->gainsX_(0), ctrlData->gainsX_(1), ctrlData->gainsX_(2)};
    ctb::PIDGains gainsY = {ctrlData->gainsY_(0), ctrlData->gainsY_(1), ctrlData->gainsY_(2)};
    ctb::PIDGains gainsZ = {ctrlData->gainsZ_(0), ctrlData->gainsZ_(1), ctrlData->gainsZ_(2)};
    ctb::PIDGains gainsRoll = {ctrlData->gainsRoll_(0), ctrlData->gainsRoll_(1), ctrlData->gainsRoll_(2)};
    ctb::PIDGains gainsPitch = {ctrlData->gainsPitch_(0), ctrlData->gainsPitch_(1), ctrlData->gainsPitch_(2)};
    ctb::PIDGains gainsYaw = {ctrlData->gainsYaw_(0), ctrlData->gainsYaw_(1), ctrlData->gainsYaw_(2)};
    ctb::PIDGains gainsDelta = {0.0, 2.0, 0.0};

    pidX.Initialize(gainsX, 0.0, 2.0);
    pidY.Initialize(gainsY, 0.0, 2.0);
    pidZ.Initialize(gainsZ, 0.0, 2.0);
    pidRoll.Initialize(gainsRoll, 0.0, 2.0);
    pidPitch.Initialize(gainsPitch, 0.0, 2.0);
    pidYaw.Initialize(gainsYaw, 0.0, 2.0);
    pidDelta.Initialize(gainsDelta, 0.001, 0.1);

    RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "PID controllers initialized.");
}

// Align vehicle with the initial direction of the path
void PathPlanningState::alignVehicleToPath() {
    const auto& poses = ctrlData->planned_path_.poses;

    if (poses.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("PathPlanningState"), "Planned path is empty.");
        return;
    }

    Eigen::Vector3d firstPoint(poses[0].pose.position.x, poses[0].pose.position.y, poses[0].pose.position.z);
    Eigen::Vector3d secondPoint(poses[1].pose.position.x, poses[1].pose.position.y, poses[1].pose.position.z);

    Eigen::Vector3d pathDirection = (secondPoint - firstPoint).normalized();
    double yawDirection = atan2(pathDirection.y(), pathDirection.x());
    double pitchDirection = -atan2(pathDirection.z(), sqrt(pathDirection.x() * pathDirection.x() + pathDirection.y() * pathDirection.y()));

    ctrlData->pose_goal_ << firstPoint.x(), firstPoint.y(), firstPoint.z(), 0.0, pitchDirection, yawDirection;

    double xError = ctrlData->pose_goal_(0) - ctrlData->pose_actual_(0);
    double yError = ctrlData->pose_goal_(1) - ctrlData->pose_actual_(1);
    double zError = ctrlData->pose_goal_(2) - ctrlData->pose_actual_(2);

    if (fabs(xError) < 0.1 && fabs(yError) < 0.1 && fabs(zError) < 0.1) {
        RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Vehicle aligned with path direction.");
        isVehicleOnPathDirection = true;
    }
}

// Update the vehicle's goal position and orientation along the path
void PathPlanningState::updateVehicleGoal() {
    if (!path) {
        RCLCPP_ERROR(rclcpp::get_logger("PathPlanningState"), "Path is null. Unable to update vehicle goal.");
        return;
    }

    // Compute the interval for finding the closest point
    double intervalEnd = std::min(currentAbscissa_ + delta, path->EndParameter());
    closestPointAbscissa = path->FindAbscissaClosestPointOnInterval(ctrlData->pose_actual_.head(3), currentAbscissa_, intervalEnd);

    Eigen::Vector3d currentPoint = path->At(closestPointAbscissa); // Closest point on the path
    double goalAbscissa = closestPointAbscissa + delta;           // Look-ahead goal point
    goalAbscissa = std::clamp(goalAbscissa, path->StartParameter(), path->EndParameter());
    Eigen::Vector3d nextPoint = path->At(goalAbscissa);           // Next goal point on the path

    // Calculate the direction vector from the closest point to the next point
    Eigen::Vector3d direction = (nextPoint - currentPoint).normalized();

    // Calculate desired heading (yaw) and pitch angles
    double desiredHeading = atan2(direction.y(), direction.x());
    double desiredPitch = -atan2(direction.z(), sqrt(direction.x() * direction.x() + direction.y() * direction.y()));

    // Limit pitch to avoid extreme angles
    desiredPitch = std::clamp(desiredPitch, -0.610865, 0.610865); // ±35 degrees in radians

    // Calculate cross-track and vertical-track errors
    updateHeadingPitch(
        ctrlData->pose_actual_.head(3), nextPoint, currentPoint,
        delta, epsilon, desiredHeading, desiredPitch, crossTrackError, verticalTrackError);

    if (ctrlData->path_planning_2d_3d == 1) { // 2D serpentine mode
        desiredPitch = 0.0; // Ensure pitch remains zero for 2D paths
    }

    // Limit adjusted pitch angle
    desiredPitch = std::clamp(desiredPitch, -0.610865, 0.610865); // ±35 degrees in radians

    // Update the goal pose
    ctrlData->pose_goal_(0) = nextPoint.x();  // X coordinate
    ctrlData->pose_goal_(1) = nextPoint.y();  // Y coordinate
    ctrlData->pose_goal_(2) = nextPoint.z();  // Z coordinate
    ctrlData->pose_goal_(3) = 0.0;            // Roll
    ctrlData->pose_goal_(4) = desiredPitch;   // Pitch
    ctrlData->pose_goal_(5) = desiredHeading; // Yaw

    // Compute errors in position and orientation
    positionXError = ctrlData->pose_goal_(0) - ctrlData->pose_actual_(0);
    positionYError = ctrlData->pose_goal_(1) - ctrlData->pose_actual_(1);
    positionZError = ctrlData->pose_goal_(2) - ctrlData->pose_actual_(2);
    rollError = ctrlData->pose_goal_(3) - ctrlData->pose_actual_(3);
    yawError = ctrlData->pose_goal_(5) - ctrlData->pose_actual_(5);
    pitchError = ctrlData->pose_goal_(4) - ctrlData->pose_actual_(4);

    // Normalize angular errors
    ctb::NormalizeAngle(rollError);
    ctb::NormalizeAngle(yawError);
    ctb::NormalizeAngle(pitchError);

    // Log errors for debugging
    RCLCPP_DEBUG(rclcpp::get_logger("PathPlanningState"),
                 "Position Errors: X=%.3f, Y=%.3f, Z=%.3f | Angular Errors: Roll=%.3f, Pitch=%.3f, Yaw=%.3f",
                 positionXError, positionYError, positionZError, rollError, pitchError, yawError);

    // Update desired velocities using PID controllers
    ctrlData->velocity_desired_(0) = -pidX.Compute(0, positionXError); // Longitudinal velocity
    ctrlData->velocity_desired_(1) = -pidY.Compute(0, positionYError); // Lateral velocity
    ctrlData->velocity_desired_(2) = -pidZ.Compute(0, positionZError); // Depth velocity
    ctrlData->velocity_desired_(3) = -pidRoll.Compute(0, rollError);   // Roll angular velocity
    ctrlData->velocity_desired_(4) = -pidPitch.Compute(0, pitchError); // Pitch angular velocity
    ctrlData->velocity_desired_(5) = -pidYaw.Compute(0, yawError);     // Yaw angular velocity

    // Evaluate derivatives to check alignment and dynamic path-following behavior
    Eigen::Vector3d currentPosDot = path->Derivate(1, closestPointAbscissa).front();
    Eigen::Vector3d goalPosDot = path->Derivate(1, goalAbscissa).front();

    Eigen::Vector3d currentDirection = currentPosDot / currentPosDot.norm();
    Eigen::Vector3d goalDirection = goalPosDot / goalPosDot.norm();

    double tangentsDifferenceNorm = (goalDirection - currentDirection).norm();

    // Adjust delta dynamically based on errors
    double baseSetpoint = deltaMax;
    double errorMagnitude = (10.0 * fabs(crossTrackError)) + (10.0 * fabs(verticalTrackError)) + (30.0 * fabs(tangentsDifferenceNorm));
    double IOutput = pidDelta.Compute(baseSetpoint - errorMagnitude, delta);
    delta += IOutput;
    delta = std::clamp(delta, deltaMin, deltaMax);

    RCLCPP_DEBUG(rclcpp::get_logger("PathPlanningState"), "Delta adjusted: %.3f | Tangent Difference Norm: %.3f", delta, tangentsDifferenceNorm);

    // Compute the percentage of the path completed
    double path_completed = (closestPointAbscissa / path->EndParameter()) * 100.0;
    RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Path completed: %.2f%%", path_completed);

    // Update current abscissa for the next cycle
    currentAbscissa_ = closestPointAbscissa;

    // Check if the path has ended
    if (currentAbscissa_ >= path->EndParameter()) {
        RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Path has ended. Switching to HOLD state.");
        fsm_->SetNextState(States::HOLD);
    }
}



bool PathPlanningState::updateHeadingPitch(
    const Eigen::Vector3d& currentPos,
    const Eigen::Vector3d& goalPos,
    const Eigen::Vector3d& closestPos,
    double Delta,
    double epsilon,
    double& DesiredHeading,
    double& DesiredPitch,
    double& crossTrackError,
    double& verticalTrackError)
{
    // Calculate the direction vector along the path
    Eigen::Vector3d pathDirection = goalPos - closestPos;
    if (pathDirection.norm() > std::numeric_limits<double>::epsilon()) {
        pathDirection.normalize();
    } else {
        std::cerr << "Warning: Attempting to normalize a zero vector in pathDirection" << std::endl;
        return false; // Early return or handle as necessary
    }

    // Calculate cross-track and vertical-track vectors
    Eigen::Vector3d crossTrackVector = Eigen::Vector3d(0, 0, 1).cross(pathDirection);
    if (crossTrackVector.norm() > std::numeric_limits<double>::epsilon()) {
        crossTrackVector.normalize();
    } else {
        std::cerr << "Warning: Attempting to normalize a zero vector in crossTrackVector" << std::endl;
        return false; // Early return or handle as necessary
    }

    Eigen::Vector3d verticalTrackVector = crossTrackVector.cross(pathDirection);
    if (verticalTrackVector.norm() > std::numeric_limits<double>::epsilon()) {
        verticalTrackVector.normalize();
    } else {
        std::cerr << "Warning: Attempting to normalize a zero vector in verticalTrackVector" << std::endl;
        return false; // Early return or handle as necessary
    }

    // Calculate position errors
    Eigen::Vector3d positionError = currentPos - closestPos;
    crossTrackError = positionError.dot(crossTrackVector);
    verticalTrackError = positionError.dot(verticalTrackVector);

    // Time since last update
    auto current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> time_elapsed = current_time - last_update_time;
    last_update_time = current_time;

    // ALOS ON FOR HEADING
    std::cout << "Cross-track error:" << crossTrackError << std::endl;
    if (fabs(crossTrackError) > epsilon) {
        double modulationFactor = Delta / sqrt(Delta * Delta + crossTrackError * crossTrackError);
        beta_hat_c += gamma_crosstrack * modulationFactor * crossTrackError * time_elapsed.count();
        beta_hat_c = std::clamp(beta_hat_c, -0.4, 0.4);
        DesiredHeading -= beta_hat_c - atan2(crossTrackError, Delta);
    }

    // ALOS ON FOR PITCH
    std::cout << "Vertical-track error:" << verticalTrackError << std::endl;
    if (fabs(verticalTrackError) > epsilon) {
        double modulation_factor_vertical = Delta / sqrt(Delta * Delta + verticalTrackError * verticalTrackError);
        theta_hat_c += gamma_verticaltrack * modulation_factor_vertical * verticalTrackError * time_elapsed.count();
        theta_hat_c = std::clamp(theta_hat_c, -0.83, 0.83);
        DesiredPitch -= theta_hat_c - atan2(verticalTrackError, Delta);
    }

    return true;
}
