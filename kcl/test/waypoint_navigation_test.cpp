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
        Require(data->velocityDesired(5) < 0.0,
                "Arrival did not restore requested final yaw");
        data->poseActual = data->poseGoal;
        state.Execute();
        Require(data->velocityDesired.norm() < 1e-9, "Final pose is not stationary");
        data->poseActual(0) += 0.3;
        state.Execute();
        Require(data->velocityDesired(0) < 0.0, "Final hold does not correct drift");

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
        data->poseActual(0) = std::numeric_limits<double>::quiet_NaN();
        Require(state.Execute() == fsm::fail && data->velocityDesired.isZero(),
                "Invalid pose did not stop motion");
        state.OnExit();
        Require(data->velocityDesired.isZero(), "State exit left motion commanded");
        std::cout << "Waypoint navigation behavior checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
