#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "states/waypoint_navigation_state.hpp"

void Require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void CheckNavigation(const Eigen::Matrix<double, 6, 1>& goal, double initialYaw, bool useBlueROV) {
    fsm::FSM fsm;
    WaypointNavigationState state(&fsm);
    auto data = std::make_shared<auv::ControlData>();
    state.ctrlData = data;
    for (auto* gains : {&data->gainsX, &data->gainsY, &data->gainsZ,
                        &data->gainsRoll, &data->gainsPitch, &data->gainsYaw}) {
        *gains << 1.0, 0.0, 0.0, 0.0, 10.0, 1.0;
    }
    data->maxVelocity << 1.0, 0.0, 0.5, 1.0, 1.0, 1.0;
    data->minVelocity = -data->maxVelocity;
    if (useBlueROV) {
        LoadParamsFromConf("BlueROV", nullptr, nullptr, nullptr,
            &data->gainsX, &data->gainsY, &data->gainsZ,
            &data->gainsRoll, &data->gainsPitch, &data->gainsYaw,
            &data->maxVelocity, &data->minVelocity);
    }
    data->waypointUseHeading = true;
    data->poseActual(5) = initialYaw;
    data->poseGoal = goal;
    Require(state.OnEntry() == fsm::ok, "Closed-loop test state entry failed");

    // Integrate body-frame commands using BlueROV limits or no lateral actuator authority.
    for (int step = 0; step < 6000; ++step) {
        Require(state.Execute() == fsm::ok, "Closed-loop execution failed");
        Require(data->velocityDesired.allFinite(), "Controller produced nonfinite velocity");
        if (useBlueROV) {
            rml::SaturateVector(data->maxVelocity, data->minVelocity, data->velocityDesired);
        } else {
            for (int axis = 0; axis < 6; ++axis) {
                data->velocityDesired(axis) = std::clamp(data->velocityDesired(axis),
                    data->minVelocity(axis), data->maxVelocity(axis));
            }
        }
        rml::EulerRPY rpy(data->poseActual(3), data->poseActual(4), data->poseActual(5));
        Eigen::Vector3d velocityWorld = rpy.ToRotationMatrix().matrix() * data->velocityDesired.head<3>();
        data->poseActual.head<3>() += data->dt * velocityWorld;
        data->poseActual(5) += data->dt * data->velocityDesired(5);
        Require((data->poseGoal - goal).norm() < 1e-9, "Transit overwrote desired goal pose");
    }
    Require((data->poseActual.head<3>() - goal.head<3>()).norm() < 0.15,
            "Vehicle did not reach waypoint with the configured velocity limits");
    Require(std::abs(ctb::AngleDifference(goal(5), data->poseActual(5))) < 0.02,
            "Vehicle did not align to desired final heading after arrival");
}

int main() {
    try {
        fsm::FSM fsm;
        WaypointNavigationState state(&fsm);
        auto data = std::make_shared<auv::ControlData>();
        state.ctrlData = data;
        for (auto* gains : {&data->gainsX, &data->gainsY, &data->gainsZ,
                            &data->gainsRoll, &data->gainsPitch, &data->gainsYaw}) {
            *gains << 1.0, 0.0, 0.0, 10.0, 1.0, 0.0;
        }
        data->maxVelocity.setConstant(1.0);
        data->minVelocity.setConstant(-1.0);
        const double pi = std::acos(-1.0);
        data->poseGoal << 0.0, 4.0, -2.0, 0.0, 0.0, 0.0;
        Require(state.OnEntry() == fsm::ok, "State entry failed");
        state.Execute();
        Require(data->velocityDesired.head<3>().norm() < 1e-9,
                "Vehicle translated before aligning");
        Require(data->velocityDesired(5) > 0.0, "Heading command has wrong sign");

        data->poseActual(5) = pi / 2.0;
        state.Execute();
        Require(data->velocityDesired(0) > 0.0 && data->velocityDesired(2) < 0.0,
                "Navigation command does not approach waypoint in body axes");
        Require(std::abs(data->velocityDesired(1)) < 1e-9,
                "World-to-body rotation is incorrect");

        data->poseActual(5) = 0.0;
        state.Execute();
        Require(data->velocityDesired.head<3>().norm() < 1e-9,
                "Vehicle did not stop to realign after heading loss");

        data->poseActual.head<3>() = data->poseGoal.head<3>();
        data->poseActual(5) = pi / 2.0;
        state.Execute();
        Require(std::abs(data->velocityDesired(5)) < 1e-9,
                "Automatic heading changed at arrival");
        data->poseActual = data->poseGoal;
        state.Execute();
        Require(data->velocityDesired.norm() < 1e-9, "Final pose is not stationary");
        data->poseActual(0) += 0.3;
        state.Execute();
        Require(data->velocityDesired(1) > 0.0, "Final hold does not correct drift in body axes");

        // A chosen final yaw must not replace the travel bearing or force sideways navigation.
        data->waypointUseHeading = true;
        data->poseActual.setZero();
        data->poseGoal << 0.0, 4.0, 0.0, 0.0, 0.0, pi;
        state.OnEntry();
        state.Execute();
        Require(data->velocityDesired.head<3>().norm() < 1e-9,
                "Transit did not align before translation");
        Require(data->poseGoal(5) == pi, "Alignment overwrote chosen final yaw");
        data->poseActual(5) = pi / 2.0;
        state.Execute();
        Require(data->velocityDesired(0) > 0.0 &&
                std::abs(data->velocityDesired(1)) < 1e-9 &&
                std::abs(data->velocityDesired(5)) < 1e-9,
                "Final heading forced sideways transit instead of forward navigation");
        data->poseActual.head<3>() = data->poseGoal.head<3>();
        state.Execute();
        Require(data->velocityDesired(5) > 0.0, "Arrival did not turn toward desired goal yaw");
        data->poseActual(5) = pi;
        state.Execute();
        Require(data->velocityDesired.norm() < 1e-9,
                "Chosen heading was not preserved at arrival");
        data->waypointUseHeading = false;

        data->poseActual.setZero();
        data->poseGoal << 0.0, 4.0, 0.0, 0.0, 0.0, 0.0;
        state.OnEntry();
        Require(std::abs(data->poseGoal(5) - pi / 2.0) < 1e-9,
                "Automatic mode did not set the initial bearing as its final heading");
        data->poseActual(0) = 1.0;
        state.Execute();
        Require(std::abs(data->poseGoal(5) - pi / 2.0) < 1e-9,
                "Changing navigation bearing overwrote the desired automatic goal heading");

        data->poseActual.setZero();
        data->poseActual(5) = pi - 0.02;
        data->poseGoal << -4.0, -0.08, 0.0, 0.0, 0.0, 0.0;
        state.OnEntry();
        state.Execute();
        Require(data->velocityDesired(0) > 0.0,
                "Heading wraparound incorrectly blocks navigation");

        data->poseGoal << 0.0, 4.0, 0.0, 0.0, 0.0, 0.0;
        data->poseActual.setZero();
        state.OnEntry();
        state.Execute();
        Require(data->velocityDesired.head<3>().norm() < 1e-9,
                "New waypoint did not restart alignment");
        // DVL altitude replaces waypoint Z and remains active at the destination.
        data->seabedAltitudeHoldEnabled = true;
        data->seabedAltitudeGoal = 2.0;
        data->shallowDepthLimit = -0.5;
        data->vehicleDepth = -4.0;
        data->seabedAltitudeActual = 1.0;
        data->poseActual.setZero();
        data->poseActual(2) = -4.0;
        data->poseGoal << 4.0, 0.0, -20.0, 0.0, 0.0, 0.0;
        state.OnEntry();
        state.Execute();
        Require(data->velocityDesired(2) > 0.0, "Low seabed altitude did not command ascent");
        Require(data->poseGoal(2) == data->poseActual(2), "Altitude hold did not replace waypoint Z");
        data->vehicleDepth = -0.6;
        state.Execute();
        Require(data->velocityDesired(2) > 0.0 && data->velocityDesired(2) <= 0.05 + 1e-9,
                "Ascent was not limited near the shallow-depth ceiling");
        data->vehicleDepth = -0.5;
        state.Execute();
        Require(std::abs(data->velocityDesired(2)) < 1e-9, "Ascent was allowed at the depth ceiling");
        data->vehicleDepth = -4.0;
        data->seabedAltitudeActual = 3.0;
        state.Execute();
        Require(data->velocityDesired(2) < 0.0, "High seabed altitude did not command descent");
        data->vehicleDepth = std::numeric_limits<double>::quiet_NaN();
        state.Execute();
        Require(std::abs(data->velocityDesired(2)) < 1e-9,
                "Missing pressure feedback did not inhibit vertical motion");
        data->vehicleDepth = -4.0;
        data->seabedAltitudeActual = std::numeric_limits<double>::quiet_NaN();
        state.Execute();
        Require(std::abs(data->velocityDesired(2)) < 1e-9,
                "Missing DVL feedback did not inhibit vertical motion");
        data->seabedAltitudeActual = 2.0;
        data->poseActual(0) = 4.0;
        state.Execute();
        data->seabedAltitudeActual = 1.0;
        state.Execute();
        Require(data->velocityDesired(2) > 0.0, "Seabed clearance was not maintained after arrival");
        data->seabedAltitudeHoldEnabled = false;
        data->poseGoal(2) = -6.0;
        state.OnEntry();
        state.Execute();
        Require(data->velocityDesired(2) < 0.0, "Disabling altitude hold did not restore waypoint Z control");
        data->poseActual(0) = std::numeric_limits<double>::quiet_NaN();
        Require(state.Execute() == fsm::fail && data->velocityDesired.isZero(),
                "Invalid pose did not stop motion");
        state.OnExit();
        Require(data->velocityDesired.isZero(), "State exit left motion commanded");

        Eigen::Matrix<double, 6, 1> goal;
        goal << 4.0, 4.0, -2.0, 0.0, 0.0, -pi / 2.0;
        CheckNavigation(goal, 0.0, true);
        CheckNavigation(goal, 0.0, false);
        goal << -4.0, -1.0, 0.0, 0.0, 0.0, pi - 0.02;
        CheckNavigation(goal, -pi + 0.02, true);
        CheckNavigation(goal, -pi + 0.02, false);
        goal << 0.0, 0.0, -2.0, 0.0, 0.0, pi / 2.0;
        CheckNavigation(goal, 0.0, true);
        CheckNavigation(goal, 0.0, false);
        std::cout << "Waypoint navigation behavior checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
