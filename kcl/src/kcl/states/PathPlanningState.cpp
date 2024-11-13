#include "states/PathPlanningState.hpp"

// Constructor
PathPlanningState::PathPlanningState(fsm::FSM* fsm) : BaseAUVState(fsm, "PATH_PLANNING") {
}

fsm::retval PathPlanningState::OnEntry() {
    std::cout << "Entering Path Planning State" << std::endl;
    std::string home_path = futils::get_homepath();
    std::filesystem::create_directories(home_path);
    if (ctrlData->path_planning_2d_3d == 3){
        std::cout << "Path Planning Helix 3D" << std::endl;
        path = sisl::PathFactory::NewHelicalPath(ctrlData->helix_startPos, ctrlData->helix_axisPos, ctrlData->helix_axisDir, ctrlData->helix_frequency, ctrlData->helix_numQuadrants, ctrlData->helix_counterClockwise);
    }
    else if (ctrlData->path_planning_2d_3d == 1){
        std::cout << "Path Planning 2D serpentine" << std::endl;
        // Check if the polygon vertices are provided
        if (ctrlData->serpentine_polygon_vertices_.empty()) {
            return fsm::fail;
        }
        // Determine the direction of the serpentine path
        sisl::Path::Direction direction = ctrlData->serpentine_direction_ ? sisl::Path::Direction::Backward : sisl::Path::Direction::Forward;
        path = sisl::PathFactory::NewSerpentine(ctrlData->serpentine_angle_, direction, ctrlData->serpentine_offset_, ctrlData->serpentine_polygon_vertices_);
    }
    else if (ctrlData->path_planning_2d_3d == 2){
        std::cout << "Path Planning 3D serpentine" << std::endl;
        // Check if the polygon vertices are provided
        if (ctrlData->serpentine_polygon_vertices_.empty()) {
            return fsm::fail;
        }
        // Determine the direction of the serpentine path
        sisl::Path::Direction direction = ctrlData->serpentine_direction_ ? sisl::Path::Direction::Backward : sisl::Path::Direction::Forward;
        path = sisl::PathFactory::New3DSerpentine(ctrlData->serpentine_angle_, direction, ctrlData->serpentine_offset_, ctrlData->serpentine_polygon_vertices_, ctrlData->dive_depth_, ctrlData->curvature_, ctrlData->dip_num_points_, ctrlData->dive_length_);
    }

    if (!path) {
        isCurveSet_ = false;
        return fsm::fail;
    }


    //Convert the path to a nav_msgs::Path
    nav_msgs::msg::Path nav_path;
    nav_path.header.frame_id = "map";
    nav_path.header.stamp = rclcpp::Clock().now();

    auto sampledPointsSharedPtr = path->Sampling(1500); // Adjust the sampling resolution as needed
    if (!sampledPointsSharedPtr || sampledPointsSharedPtr->empty()) {
        throw std::runtime_error("Failed to sample points from SISL path.");
    }

    for (const auto& point : *sampledPointsSharedPtr) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = point.x();
        pose_stamped.pose.position.y = point.y();
        pose_stamped.pose.position.z = point.z();
        // Assuming no orientation information; set to identity quaternion
        pose_stamped.pose.orientation.w = 1.0;
        nav_path.poses.push_back(pose_stamped);
    }

    ctrlData->planned_path_ = nav_path;

    isCurveSet_ = true;
    std::cout << "Path set and vehicle is ready" << std::endl;
    PersistenceManager::SaveObj(path->Sampling(30000), home_path + "/path.txt");

    // Initialize PID controllers
    ctb::PIDGains gainsX = {ctrlData->gainsX_(0), ctrlData->gainsX_(1), ctrlData->gainsX_(2), ctrlData->gainsX_(3), ctrlData->gainsX_(4), ctrlData->gainsX_(5)};
    ctb::PIDGains gainsY = {ctrlData->gainsY_(0), ctrlData->gainsY_(1), ctrlData->gainsY_(2), ctrlData->gainsY_(3), ctrlData->gainsY_(4), ctrlData->gainsY_(5)};
    ctb::PIDGains gainsZ = {ctrlData->gainsZ_(0), ctrlData->gainsZ_(1), ctrlData->gainsZ_(2), ctrlData->gainsZ_(3), ctrlData->gainsZ_(4), ctrlData->gainsZ_(5)};
    ctb::PIDGains gainsRoll = {ctrlData->gainsRoll_(0), ctrlData->gainsRoll_(1), ctrlData->gainsRoll_(2), ctrlData->gainsRoll_(3), ctrlData->gainsRoll_(4), ctrlData->gainsRoll_(5)};
    ctb::PIDGains gainsPitch = {ctrlData->gainsPitch_(0), ctrlData->gainsPitch_(1), ctrlData->gainsPitch_(2), ctrlData->gainsPitch_(3), ctrlData->gainsPitch_(4), ctrlData->gainsPitch_(5)};
    ctb::PIDGains gainsYaw = {ctrlData->gainsYaw_(0), ctrlData->gainsYaw_(1), ctrlData->gainsYaw_(2), ctrlData->gainsYaw_(3), ctrlData->gainsYaw_(4), ctrlData->gainsYaw_(5)};

    ctb::PIDGains gainsDelta = {0.0, 2.0, 0.0, 0.0, 0.0, 0.001};
    pidX.Initialize(gainsX, 0.0, 2); // Initialize the PID controller for longitudinal position
    pidY.Initialize(gainsY, 0.0, 2); // Initialize the PID controller for lateral position
    pidZ.Initialize(gainsZ, 0.0, 2); // Initialize the PID controller for depth position
    pidRoll.Initialize(gainsRoll, 0.0, 2); // Initialize the PID controller for roll
    pidPitch.Initialize(gainsPitch, 0.0, 2); // Initialize the PID controller for pitch
    pidYaw.Initialize(gainsYaw, 0.0, 2); // Initialize the PID controller for yaw

    pidDelta.Initialize(gainsDelta, 0.001, 0.1); // Initialize the PID controller for delta

    last_update_time = std::chrono::system_clock::now(); // Initialize time tracking

    return fsm::ok;
}



fsm::retval PathPlanningState::Execute() {
    static bool initial_time_set = false;
    static rclcpp::Time previous_time;

    if (!isCurveSet_) {
        std::cout << "Curve is not set. Waiting for curve to be set." << std::endl;
        return fsm::ok; // Keep trying in the next cycles
    }

    // Check if the simulation time has been published
    while (ctrlData->time_actual_.nanoseconds() < 0) {
        std::cout << "Waiting for simulation time to be published..." << std::endl;
        return fsm::ok; // Keep trying in the next cycles
    }

    const auto& poses = ctrlData->planned_path_.poses;

    if (!initial_time_set) {
        previous_time = ctrlData->time_actual_;
        initial_time_set = true;
    }

    // Ensure the vehicle has reached the path and is aligned
    if (!isVehicleOnPathDirection) {
        // Assuming 'poses' is a std::vector or similar container of poses with position and orientation data
        Eigen::Vector3d firstPoint(poses[0].pose.position.x, poses[0].pose.position.y, poses[0].pose.position.z);
        Eigen::Vector3d secondPoint(poses[1].pose.position.x, poses[1].pose.position.y, poses[1].pose.position.z);

        // Calculate normalized direction vector
        Eigen::Vector3d pathDirection = (secondPoint - firstPoint).normalized();

        // Calculate yaw (rotation around z-axis)
        double yawDirection = atan2(pathDirection.y(), pathDirection.x());

        // Calculate pitch (rotation around y-axis, angle from horizontal plane)
        double pitchDirection = -atan2(pathDirection.z(), sqrt(pathDirection.x() * pathDirection.x() + pathDirection.y() * pathDirection.y()));
        // Prepare goal pose (translation and rotation)
        Eigen::Matrix<double, 6, 1> poseGoal;
        poseGoal << firstPoint.x(), firstPoint.y(), firstPoint.z(), 0, pitchDirection, yawDirection;
        ctrlData->pose_goal_ = poseGoal;

        // Compute errors and desired velocities for x, y, pitch, and yaw
        positionXError = ctrlData->pose_goal_(0) - ctrlData->pose_actual_(0);
        positionYError = ctrlData->pose_goal_(1) - ctrlData->pose_actual_(1);
        positionZError = ctrlData->pose_goal_(2) - ctrlData->pose_actual_(2);
        rollError = ctrlData->pose_goal_(3) - ctrlData->pose_actual_(3);
        yawError = ctrlData->pose_goal_(5) - ctrlData->pose_actual_(5);
        pitchError = ctrlData->pose_goal_(4) - ctrlData->pose_actual_(4);
        ctb::NormalizeAngle(rollError);
        ctb::NormalizeAngle(yawError);
        ctb::NormalizeAngle(pitchError);
        // std::cout << "Position X error: " << positionXError << std::endl;
        // std::cout << "Position Y error: " << positionYError << std::endl;
        // std::cout << "Position Z error: " << positionZError << std::endl;
        // std::cout << "Roll error: " << rollError << std::endl;
        // std::cout << "Pitch error: " << pitchError << std::endl;
        // std::cout << "Yaw error: " << yawError << std::endl;

        // Set desired velocities
        ctrlData->velocity_desired_(0) = -pidX.Compute(0, positionXError);
        ctrlData->velocity_desired_(1) = -pidY.Compute(0, positionYError);
        ctrlData->velocity_desired_(2) = -pidZ.Compute(0, positionZError);
        ctrlData->velocity_desired_(3) = -pidRoll.Compute(0, rollError);
        ctrlData->velocity_desired_(4) = -pidPitch.Compute(0, pitchError);
        ctrlData->velocity_desired_(5) = -pidYaw.Compute(0, yawError);



        // Check if vehicle has reached the starting point and aligned with path direction
        if (positionXError < 0.1 && positionYError < 0.1 && positionZError < 0.1 && rollError < 0.1 && yawError < 0.1 && pitchError < 0.1){
            std::cout << "Vehicle reached starting point and in direction of the path" << std::endl;
                ctrlData->velocity_desired_.setZero();
                isVehicleOnPathDirection = true;
        }

    }
    else {
        double intervalEnd = std::min(currentAbscissa_ + delta, path->EndParameter());
        closestPointAbscissa = path->FindAbscissaClosestPointOnInterval(ctrlData->pose_actual_.head(3), currentAbscissa_, intervalEnd);

        Eigen::Vector3d currentPoint = path->At(closestPointAbscissa); // returns x,y coordinates (UTM) of the goal position
        double goalAbscissa = closestPointAbscissa + delta; // delta should be set to 1 initially, can be adjusted dynamically later
        goalAbscissa = std::clamp(goalAbscissa, path->StartParameter(), path->EndParameter());
        Eigen::Vector3d nextPoint = path->At(goalAbscissa); // returns x,y coordinates (UTM) of the goal position

        // Calculate the direction vector
        Eigen::Vector3d direction = (nextPoint - currentPoint).normalized();
        double pi_h = atan2(direction.y(), direction.x());
        // Calculate the pitch angle
        double pi_p = -atan2(direction.z(), sqrt(direction.x() * direction.x() + direction.y() * direction.y()));


        // Calculate cross-track error
        double psi_d = pi_h;
        double theta_psi_d = pi_p;
        pi_p = std::clamp(pi_p, -0.610865, 0.610865);
        updateHeadingPitch(ctrlData->pose_actual_.head(3), nextPoint, currentPoint, delta, epsilon, psi_d, theta_psi_d, crossTrackError, verticalTrackError);
        if (ctrlData->path_planning_2d_3d == 1){
            theta_psi_d = pi_p;
        }
        theta_psi_d = std::clamp(theta_psi_d, -0.610865, 0.610865);

        // Update the goal position and orientation
        ctrlData->pose_goal_(0) = nextPoint.x(); // x coordinate
        ctrlData->pose_goal_(1) = nextPoint.y(); // y coordinate
        ctrlData->pose_goal_(2) = nextPoint.z(); // z coordinate
        ctrlData->pose_goal_(3) = 0; // roll
        //ALOS OFF
        ctrlData->pose_goal_(4) = pi_p; // pitch
        ctrlData->pose_goal_(5) = pi_h; // yaw

        // ALOS ON
        // ctrlData->pose_goal_(4) = theta_psi_d;
        // ctrlData->pose_goal_(5) = psi_d;


        // Compute errors and desired velocities for x, y, pitch, and yaw
        positionXError = ctrlData->pose_goal_(0) - ctrlData->pose_actual_(0);
        positionYError = ctrlData->pose_goal_(1) - ctrlData->pose_actual_(1);
        positionZError = ctrlData->pose_goal_(2) - ctrlData->pose_actual_(2);
        rollError = ctrlData->pose_goal_(3) - ctrlData->pose_actual_(3);
        yawError = ctrlData->pose_goal_(5) - ctrlData->pose_actual_(5);
        pitchError = ctrlData->pose_goal_(4) - ctrlData->pose_actual_(4);
        ctb::NormalizeAngle(rollError);
        ctb::NormalizeAngle(yawError);
        ctb::NormalizeAngle(pitchError);
        std::cout << "Position X error: " << positionXError << std::endl;
        std::cout << "Position Y error: " << positionYError << std::endl;
        std::cout << "Position Z error: " << positionZError << std::endl;
        std::cout << "Roll error: " << rollError << std::endl;
        std::cout << "Pitch error: " << pitchError << std::endl;
        std::cout << "Yaw error: " << yawError << std::endl;


        ctrlData->velocity_desired_(0) = -pidX.Compute(0, positionXError);
        ctrlData->velocity_desired_(1) = -pidY.Compute(0, positionYError);
        ctrlData->velocity_desired_(2) = -pidZ.Compute(0, positionZError);
        ctrlData->velocity_desired_(3) = -pidRoll.Compute(0, rollError);
        ctrlData->velocity_desired_(4) = -pidPitch.Compute(0, pitchError);
        ctrlData->velocity_desired_(5) = -pidYaw.Compute(0, yawError);

        // Evaluate derivatives at points of interest
        Eigen::Vector3d currentPosDot = path->Derivate(1, closestPointAbscissa).front();
        Eigen::Vector3d goalPosDot = path->Derivate(1, goalAbscissa).front();

        // Normalize the direction vectors
        Eigen::Vector3d currentDirection = currentPosDot / currentPosDot.norm();
        Eigen::Vector3d goalDirection = goalPosDot / goalPosDot.norm();

        // Evaluate tangent difference norm. double showing how much the vehicle is aligned with the path direction but not if the path is changing
        double tangentsDifferenceNorm = (goalDirection - currentDirection).norm();


        // Constants for setpoint calculation
        double baseSetpoint = deltaMax; // Base value of delta when there is no error

        // Calculate dynamic setpoint based on errors
        double weightCrosstrack = 10.0;
        double weightVerticalTrack = 10.0;
        double weightTangentDifference = 30.0;
        if (ctrlData->path_planning_2d_3d == 1){
            weightVerticalTrack = 0.0;
        }
        double errorMagnitude = weightTangentDifference * fabs(tangentsDifferenceNorm) +
                                weightCrosstrack * fabs(crossTrackError) +
                                weightVerticalTrack * fabs(verticalTrackError);
        double setPoint = baseSetpoint - errorMagnitude; // Dynamic adjustment
        double error = setPoint - delta; // Assuming you have a method to get the current value of delta
        // Compute output using the PID controller
        double IOutput = pidDelta.Compute(setPoint, -error); // Compute based on the dynamically calculated setpoint and the current delta
        delta += IOutput; // Scale the PID output to modulate the control response

        // Limit delta between min (deltaMin) and max (deltaMax)
        delta = std::clamp(delta, deltaMin, deltaMax);
        std::cout << "Delta: " << delta << std::endl;
        std::cout << "Time: " << ctrlData->time_actual_.seconds() << std::endl;
        //compute how much of path is completed in %
        double path_completed = (closestPointAbscissa / path->EndParameter()) * 100;
        std::cout << "Path completed: " << path_completed << "%" << std::endl;

        // Update the current abscissa to the abscissa of the closest point, preparing for the next update cycle
        currentAbscissa_ = closestPointAbscissa;
        if (currentAbscissa_ >= path->EndParameter()) {// this has an segmentation error when path has finished, check it
            std::cout << "Path has ended" << std::endl;
            fsm_->SetNextState(States::HOLD);
        }


    }
    return fsm::ok;
}


fsm::retval PathPlanningState::OnExit() {
    ctrlData->planned_path_.poses.clear();
    isVehicleOnPathDirection = false;
    closestPointAbscissa = 0.0;
    currentAbscissa_ = 0.0;
    return fsm::ok;
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
        double modulation_factor = Delta / sqrt(Delta * Delta + crossTrackError * crossTrackError);
        beta_hat_c += gamma_crosstrack * modulation_factor * crossTrackError * time_elapsed.count();
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



