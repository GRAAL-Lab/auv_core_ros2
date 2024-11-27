#pragma once

#include "states/base_auv_state.hpp"
#include <vector>
#include <string>
#include <memory>

/// The `JoystickState` class represents a state where the AUV is controlled via joystick input.
class JoystickState : public BaseAUVState {
private:


    float joystickData[12][2][2] = {0}; // Initialized to 0 for safety
    std::shared_ptr<sensor_msgs::msg::Joy> joystickIdle;
    bool calibrationDone = false;
    bool forwardCalibrated = false;
    bool backwardCalibrated = false;
    bool rightCalibrated = false;
    bool leftCalibrated = false;
    bool upCalibrated = false;
    bool downCalibrated = false;

    bool yawRightCalibrated = false;
    bool yawLefteCalibrated = false;
    bool rollRightCalibrated = false;
    bool rollLeftCalibrated = false;
    bool pitchUpCalibrated = false;
    bool pitchDownCalibrated = false;
    bool idleCalibrated = false;

    int confirmationButton = 0;
    bool xFound = false;
    float incrementSpeed = 0.1f;
    




public:
    explicit JoystickState(fsm::FSM* fsm);

    fsm::retval OnEntry() noexcept override;
    fsm::retval Execute() noexcept override;
    fsm::retval OnExit() noexcept override;

private:
    void CalibrateJoystick();
    void MapJoystickToVelocityVelocites();
    double MapValue(double value, double inMin, double inMax, double outMin, double outMax);
};
