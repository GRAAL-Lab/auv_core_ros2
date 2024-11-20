#include "states/PathPlanningState.hpp"

// Constructor
PathPlanningState::PathPlanningState(fsm::FSM* fsm) : BaseAUVState(fsm, "PATH_PLANNING") {
}

fsm::retval PathPlanningState::OnEntry() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Entering PATH_PLANNING state");
    std::string home_path = futils::get_homepath();
    std::filesystem::create_directories(home_path);
    if (ctrlData->pathPlanningMode == 3){
        RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Path Planning Helix 3D");
        path = sisl::PathFactory::NewHelicalPath(ctrlData->helixStartPos, ctrlData->helixAxisPos, ctrlData->helixAxisDir, ctrlData->helixFrequency, ctrlData->helixNumQuadrants, ctrlData->helixCounterClockwise);
    }
    else if (ctrlData->pathPlanningMode == 1){
        RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Path Planning Serpentine 2D");
        // Check if the polygon vertices are provided
        if (ctrlData->serpentinePolygonVertices.empty()) {
            return fsm::fail;
        }
        // Determine the direction of the serpentine path
        sisl::Path::Direction direction = ctrlData->serpentineDirection ? sisl::Path::Direction::Backward : sisl::Path::Direction::Forward;
        path = sisl::PathFactory::NewSerpentine(ctrlData->serpentineAngle, direction, ctrlData->serpentineOffset, ctrlData->serpentinePolygonVertices);
    }
    else if (ctrlData->pathPlanningMode == 2){
        RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Path Planning 3D Serpentine");
        // Check if the polygon vertices are provided
        if (ctrlData->serpentinePolygonVertices.empty()) {
            return fsm::fail;
        }
        // Determine the direction of the serpentine path
        sisl::Path::Direction direction = ctrlData->serpentineDirection ? sisl::Path::Direction::Backward : sisl::Path::Direction::Forward;
        path = sisl::PathFactory::New3DSerpentine(ctrlData->serpentineAngle, direction, ctrlData->serpentineOffset, ctrlData->serpentinePolygonVertices, ctrlData->diveDepth, ctrlData->curvature, ctrlData->dipNumPoints, ctrlData->diveLength);
    }

    if (!path) {
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
    ctrlData->plannedPath = nav_path;
    isCurveSet_ = true;
    RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Path set and vehicle is ready");
    // PersistenceManager::SaveObj(path->Sampling(30000), home_path + "/path.txt");

    // Initialize PID controllers
    ctb::PIDGains gainsX = {ctrlData->gainsX(0), ctrlData->gainsX(1), ctrlData->gainsX(2), ctrlData->gainsX(3), ctrlData->gainsX(4), ctrlData->gainsX(5)};
    ctb::PIDGains gainsY = {ctrlData->gainsY(0), ctrlData->gainsY(1), ctrlData->gainsY(2), ctrlData->gainsY(3), ctrlData->gainsY(4), ctrlData->gainsY(5)};
    ctb::PIDGains gainsZ = {ctrlData->gainsZ(0), ctrlData->gainsZ(1), ctrlData->gainsZ(2), ctrlData->gainsZ(3), ctrlData->gainsZ(4), ctrlData->gainsZ(5)};
    ctb::PIDGains gainsRoll = {ctrlData->gainsRoll(0), ctrlData->gainsRoll(1), ctrlData->gainsRoll(2), ctrlData->gainsRoll(3), ctrlData->gainsRoll(4), ctrlData->gainsRoll(5)};
    ctb::PIDGains gainsPitch = {ctrlData->gainsPitch(0), ctrlData->gainsPitch(1), ctrlData->gainsPitch(2), ctrlData->gainsPitch(3), ctrlData->gainsPitch(4), ctrlData->gainsPitch(5)};
    ctb::PIDGains gainsYaw = {ctrlData->gainsYaw(0), ctrlData->gainsYaw(1), ctrlData->gainsYaw(2), ctrlData->gainsYaw(3), ctrlData->gainsYaw(4), ctrlData->gainsYaw(5)};

    ctb::PIDGains gainsDelta = {0.0, 2.0, 0.0, 0.0, 0.0, 0.001};
    pidX_.Initialize(gainsX, 0.0, 2); // Initialize the PID controller for longitudinal position
    pidY_.Initialize(gainsY, 0.0, 2); // Initialize the PID controller for lateral position
    pidZ_.Initialize(gainsZ, 0.0, 2); // Initialize the PID controller for depth position
    pidRoll_.Initialize(gainsRoll, 0.0, 2); // Initialize the PID controller for roll
    pidPitch_.Initialize(gainsPitch, 0.0, 2); // Initialize the PID controller for pitch
    pidYaw_.Initialize(gainsYaw, 0.0, 2); // Initialize the PID controller for yaw

    pidDelta_.Initialize(gainsDelta, 0.001, 0.1); // Initialize the PID controller for delta

    last_update_time = std::chrono::system_clock::now(); // Initialize time tracking

    return fsm::ok;
}



fsm::retval PathPlanningState::Execute() noexcept { 
    static bool initial_time_set = false;
    static rclcpp::Time previous_time;

    if (!isCurveSet_) {
        RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Curve is not set. Waiting for curve to be set.");
        return fsm::ok; // Keep trying in the next cycles
    }

    // Check if the simulation time has been published
    while (ctrlData->timeActual.nanoseconds() < 0) {
        RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Waiting for simulation time to be published...");
        return fsm::ok; // Keep trying in the next cycles
    }

    const auto& poses = ctrlData->plannedPath.poses;

    if (!initial_time_set) {
        previous_time = ctrlData->timeActual;
        initial_time_set = true;
    }

    // Ensure the vehicle has reached the path and is aligned
    if (!isVehicleOnPathDirection_) {
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
        ctrlData->poseGoal = poseGoal;

        // Compute errors and desired velocities for x, y, pitch, and yaw
        positionXError_ = ctrlData->poseGoal(0) - ctrlData->poseActual(0);
        positionYError_ = ctrlData->poseGoal(1) - ctrlData->poseActual(1);
        positionZError_ = ctrlData->poseGoal(2) - ctrlData->poseActual(2);
        rollError_ = ctrlData->poseGoal(3) - ctrlData->poseActual(3);
        yawError_ = ctrlData->poseGoal(5) - ctrlData->poseActual(5);
        pitchError_ = ctrlData->poseGoal(4) - ctrlData->poseActual(4);
        ctb::NormalizeAngle(rollError_);
        ctb::NormalizeAngle(yawError_);
        ctb::NormalizeAngle(pitchError_);

        // Set desired velocities
        ctrlData->velocityDesired(0) = -pidX_.Compute(0, positionXError_);
        ctrlData->velocityDesired(1) = -pidY_.Compute(0, positionYError_);
        ctrlData->velocityDesired(2) = -pidZ_.Compute(0, positionZError_);
        ctrlData->velocityDesired(3) = -pidRoll_.Compute(0, rollError_);
        ctrlData->velocityDesired(4) = -pidPitch_.Compute(0, pitchError_);
        ctrlData->velocityDesired(5) = -pidYaw_.Compute(0, yawError_);



        // Check if vehicle has reached the starting point and aligned with path direction
        if (positionXError_ < 0.1 && positionYError_ < 0.1 && positionZError_ < 0.1 && rollError_ < 0.1 && yawError_ < 0.1 && pitchError_ < 0.1){
            RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Vehicle reached starting point and in direction of the path");
                ctrlData->velocityDesired.setZero();
                isVehicleOnPathDirection_ = true;
        }

    }
    else {
        double intervalEnd = std::min(currentAbscissa_ + delta_, path->EndParameter());
        closestPointAbscissa_ = path->FindAbscissaClosestPointOnInterval(ctrlData->poseActual.head(3), currentAbscissa_, intervalEnd);

        Eigen::Vector3d currentPoint = path->At(closestPointAbscissa_); // returns x,y coordinates (UTM) of the goal position
        double goalAbscissa = closestPointAbscissa_ + delta_; // delta should be set to 1 initially, can be adjusted dynamically later
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
        updateHeadingPitch(ctrlData->poseActual.head(3), nextPoint, currentPoint, delta_, epsilon, psi_d, theta_psi_d, crossTrackError_, verticalTrackError_);
        if (ctrlData->pathPlanningMode == 1){
            theta_psi_d = pi_p;
        }
        theta_psi_d = std::clamp(theta_psi_d, -0.610865, 0.610865);

        // Update the goal position and orientation
        ctrlData->poseGoal(0) = nextPoint.x(); // x coordinate
        ctrlData->poseGoal(1) = nextPoint.y(); // y coordinate
        ctrlData->poseGoal(2) = nextPoint.z(); // z coordinate
        ctrlData->poseGoal(3) = 0; // roll
        //ALOS OFF
        ctrlData->poseGoal(4) = pi_p; // pitch
        ctrlData->poseGoal(5) = pi_h; // yaw

        // ALOS ON
        // ctrlData->poseGoal(4) = theta_psi_d;
        // ctrlData->poseGoal(5) = psi_d;


        // Compute errors and desired velocities for x, y, pitch, and yaw
        positionXError_ = ctrlData->poseGoal(0) - ctrlData->poseActual(0);
        positionYError_ = ctrlData->poseGoal(1) - ctrlData->poseActual(1);
        positionZError_ = ctrlData->poseGoal(2) - ctrlData->poseActual(2);
        rollError_ = ctrlData->poseGoal(3) - ctrlData->poseActual(3);
        yawError_ = ctrlData->poseGoal(5) - ctrlData->poseActual(5);
        pitchError_ = ctrlData->poseGoal(4) - ctrlData->poseActual(4);
        ctb::NormalizeAngle(rollError_);
        ctb::NormalizeAngle(yawError_);
        ctb::NormalizeAngle(pitchError_);


        ctrlData->velocityDesired(0) = -pidX_.Compute(0, positionXError_);
        ctrlData->velocityDesired(1) = -pidY_.Compute(0, positionYError_);
        ctrlData->velocityDesired(2) = -pidZ_.Compute(0, positionZError_);
        ctrlData->velocityDesired(3) = -pidRoll_.Compute(0, rollError_);
        ctrlData->velocityDesired(4) = -pidPitch_.Compute(0, pitchError_);
        ctrlData->velocityDesired(5) = -pidYaw_.Compute(0, yawError_);

        // Evaluate derivatives at points of interest
        Eigen::Vector3d currentPosDot = path->Derivate(1, closestPointAbscissa_).front();
        Eigen::Vector3d goalPosDot = path->Derivate(1, goalAbscissa).front();

        // Normalize the direction vectors
        Eigen::Vector3d currentDirection = currentPosDot / currentPosDot.norm();
        Eigen::Vector3d goalDirection = goalPosDot / goalPosDot.norm();

        // Evaluate tangent difference norm. double showing how much the vehicle is aligned with the path direction but not if the path is changing
        double tangentsDifferenceNorm = (goalDirection - currentDirection).norm();


        double baseSetpoint = deltaMax_;
        double errorMagnitude = (10.0 * fabs(crossTrackError_)) + (10.0 * fabs(verticalTrackError_)) + (30.0 * fabs(tangentsDifferenceNorm));
        double IOutput = pidDelta_.Compute(baseSetpoint - errorMagnitude, delta_);
        delta_ += IOutput; // Scale the PID output to modulate the control response

        delta_ = std::clamp(delta_, deltaMin_, deltaMax_);
        RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Delta: %f", delta_);
        RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Time: %f", ctrlData->timeActual.seconds());
        //compute how much of path is completed in %
        double path_completed = (closestPointAbscissa_ / path->EndParameter()) * 100;
        RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Path completed: %f", path_completed);

        // Update the current abscissa to the abscissa of the closest point, preparing for the next update cycle
        currentAbscissa_ = closestPointAbscissa_;
        if (currentAbscissa_ >= path->EndParameter()) {// this has an segmentation error when path has finished, check it
            RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Path has ended");
            fsm_->SetNextState(States::HOLD);
        }

    }
    return fsm::ok;
}


fsm::retval PathPlanningState::OnExit() noexcept {
    ctrlData->plannedPath.poses.clear();
    isVehicleOnPathDirection_ = false;
    closestPointAbscissa_ = 0.0;
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
    double& crossTrackError_,
    double& verticalTrackError_)
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
    crossTrackError_ = positionError.dot(crossTrackVector);
    verticalTrackError_ = positionError.dot(verticalTrackVector);

    // Time since last update
    auto current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> time_elapsed = current_time - last_update_time;
    last_update_time = current_time;

    // ALOS ON FOR HEADING
    RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Cross-track error: %f", crossTrackError_);
    if (fabs(crossTrackError_) > epsilon) {
        double modulation_factor = Delta / sqrt(Delta * Delta + crossTrackError_ * crossTrackError_);
        betaHat_c += gamma_crosstrack * modulation_factor * crossTrackError_ * time_elapsed.count();
        betaHat_c = std::clamp(betaHat_c, -0.4, 0.4);
        DesiredHeading -= betaHat_c - atan2(crossTrackError_, Delta);
    }

    // ALOS ON FOR PITCH
    RCLCPP_INFO(rclcpp::get_logger("PathPlanningState"), "Vertical-track error: %f", verticalTrackError_);
    if (fabs(verticalTrackError_) > epsilon) {
        double modulation_factor_vertical = Delta / sqrt(Delta * Delta + verticalTrackError_ * verticalTrackError_);
        thetaHat_c += gamma_verticaltrack * modulation_factor_vertical * verticalTrackError_ * time_elapsed.count();
        thetaHat_c = std::clamp(thetaHat_c, -0.83, 0.83);
        DesiredPitch -= thetaHat_c - atan2(verticalTrackError_, Delta);
    }

    return true;
}