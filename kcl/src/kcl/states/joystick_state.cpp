#include "states/joystick_state.hpp"

// Constructor
JoystickState::JoystickState(fsm::FSM* fsm)
    : BaseAUVState(fsm, "JOYSTICK") {}

// OnEntry: Initialize joystick state
fsm::retval JoystickState::OnEntry() noexcept {
    if (!ctrlData->joystickMsg) {
        RCLCPP_ERROR(rclcpp::get_logger("JoystickState"), "Joystick not connected or node not running.");
        return fsm::fail;
    }
    RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Entering JOYSTICK state");
    ctrlData->poseGoal.setZero();
    return fsm::ok;
}

// Execute: Process joystick input
fsm::retval JoystickState::Execute() noexcept {
    if (ctrlData->joystickMsg) {
        if (!calibrationDone) {
            CalibrateJoystick();
        } else {
            std::cout << "Calibration Results:" << std::endl;
            for (int action = 0; action < 12; ++action) {
                std::cout << "Action " << action 
                          << " | Idle Value: " << joystickData[action][0][0]
                          << " | Calibrated Value: " << joystickData[action][1][0]
                          << " | Axis: " << joystickData[action][1][1] 
                          << std::endl;
            }
            MapJoystickToVelocityVelocites();
        }
        return fsm::ok;
    }
    else{
        RCLCPP_ERROR(rclcpp::get_logger("JoystickState"), "Joystick not connected or node not running.");
        return fsm::fail;
    }
    
}


// OnExit: Cleanup
fsm::retval JoystickState::OnExit() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Exiting JOYSTICK state");
    return fsm::ok;
}

void JoystickState::CalibrateJoystick() {
    static bool buttonPressed = false; // Tracks if the confirmation button is currently pressed
    
    if (!xFound) {
        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Please press the 'X' button");
        for (std::size_t i = 0; i < ctrlData->joystickMsg->buttons.size(); ++i) {
            if (ctrlData->joystickMsg->buttons[i] == 1) {
                joystickIdle = ctrlData->joystickMsg;
                confirmationButton = i;
                xFound = true;
                return;
            }
        }
    }
    else{

        // Check the button press state to handle debounce
        if (ctrlData->joystickMsg->buttons[confirmationButton] == 1) {
            if (!buttonPressed) {
                buttonPressed = true; // Button press detected
            }
        } else {
            buttonPressed = false; // Reset button state upon release
        }

        if (!forwardCalibrated) {
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Please put joystick in FORWARD and press 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->axes.size(); ++i) {
                if(joystickIdle->axes[i]>=0){
                    if (abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i]) > 0.4 && buttonPressed) {
                        joystickData[0][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[0][1][1] = static_cast<float>(i);           // Axis index
                        forwardCalibrated = true;
                        buttonPressed = false; // Reset button state
                        return;
                    }
                }
                else if (joystickIdle->axes[i]<=-1){
                if ((abs(ctrlData->joystickMsg->axes[i] - joystickIdle->axes[i])/2 ) > 0.4 && buttonPressed) {
                        joystickData[0][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[0][1][1] = static_cast<float>(i);           // Axis index
                        forwardCalibrated = true;
                        buttonPressed = false; // Reset button state
                        return;
                    }
                }
            }
        }
        else if (!backwardCalibrated && forwardCalibrated) {
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Please put joystick in BACKWARD and press 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->axes.size(); ++i) {
                if (joystickIdle->axes[i] <= 0){
                    //idle is 0 and moves to 1 or -1
                    if (abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i]) > 0.4 && buttonPressed) {
                        if (joystickData[0][1][0]*ctrlData->joystickMsg->axes[i]<0){
                            //make sure its the opposite direction before calibrating
                            joystickData[1][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                            joystickData[1][1][1] = static_cast<float>(i);           // Axis index
                            backwardCalibrated = true;
                            buttonPressed = false; // Reset button state
                            return;
                        }
                        
                    }
                }
                else if (joystickIdle->axes[i]>=1){
                    //idle is 1 and moves to -1
                    if ((abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i])/2 ) > 0.4 && buttonPressed) {
                        if (joystickData[0][1][0]*ctrlData->joystickMsg->axes[i]<0){
                            //make sure its the opposite direction before calibrating
                            joystickData[1][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                            joystickData[1][1][1] = static_cast<float>(i);           // Axis index
                            backwardCalibrated = true;
                            buttonPressed = false; // Reset button state
                            return;
                        }
                    }
                }
            }
        }  
        
        else if (!yawRightCalibrated && backwardCalibrated) {
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Please put joystick in YAW CLOCKWISE and press 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->axes.size(); ++i) {
                if (joystickIdle->axes[i] <=0){
                    if (abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i]) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1])) {
                        joystickData[2][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[2][1][1] = static_cast<float>(i);           // Axis index
                        yawRightCalibrated = true;
                        buttonPressed = false; // Reset button state
                        return;
                    }
                }
                else if (joystickIdle->axes[i]>=0){
                    if ((abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i])/2 ) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1])) {
                        joystickData[2][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[2][1][1] = static_cast<float>(i);           // Axis index
                        yawRightCalibrated = true;
                        buttonPressed = false; // Reset button state
                        return;
                    }
                }
            }
        }  
        else if (!yawLefteCalibrated && yawRightCalibrated) {
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Please put joystick in YAW COUNTERCLOCKWISE and press 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->axes.size(); ++i) {
                if (joystickIdle->axes[i]<=0){
                    if (abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i]) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1])) {
                            if (joystickData[2][1][0]*ctrlData->joystickMsg->axes[i]<0){
                                //make sure its the opposite direction before calibrating

                                joystickData[3][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                                joystickData[3][1][1] = static_cast<float>(i);           // Axis index
                                yawLefteCalibrated = true;
                                buttonPressed = false; // Reset button state
                                return;
                            }
                    }
                }
                else if (joystickIdle->axes[i]>=0){
                    if ((abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i])/2 ) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1])) {
                            if (joystickData[2][1][0]*ctrlData->joystickMsg->axes[i]<0){
                                //make sure its the opposite direction before calibrating
                                joystickData[3][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                                joystickData[3][1][1] = static_cast<float>(i);           // Axis index
                                yawLefteCalibrated = true;
                                buttonPressed = false; // Reset button state
                                return;
                            }
                    }
                }
            }
            
        }
        
        else if (!upCalibrated && yawLefteCalibrated){
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Please put joystick in UP and press 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->axes.size(); ++i) {

                if (joystickIdle->axes[i]<=0){
                    if (abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i]) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[3][1][1])) {
                        joystickData[4][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[4][1][1] = static_cast<float>(i);           // Axis index
                        upCalibrated = true;
                        buttonPressed = false; // Reset button state
                        return;
                    }
                }
                else if (joystickIdle->axes[i]>=0){
                    if ((abs(ctrlData->joystickMsg->axes[i] - joystickIdle->axes[i])/2 ) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[3][1][1])) {
                        joystickData[4][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[4][1][1] = static_cast<float>(i);           // Axis index
                        upCalibrated = true;
                        buttonPressed = false; // Reset button state
                        return;
                    }
                }
            }
        }
        else if (!downCalibrated && upCalibrated){
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Please put joystick in DOWN and press 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->axes.size(); ++i) {
                if (abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i]) > 0.4 && buttonPressed &&
                    static_cast<int>(i) != static_cast<int>(joystickData[1][1][1]) &&
                    static_cast<int>(i) != static_cast<int>(joystickData[3][1][1])) {
                        if(joystickData[4][1][0] * ctrlData->joystickMsg->axes[i] < 0){
                            //make sure its the opposite direction before calibrating
                            joystickData[5][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                            joystickData[5][1][1] = static_cast<float>(i);           // Axis index
                            downCalibrated = true;
                            buttonPressed = false; // Reset button state
                            return;
                        }
                }
                else if (joystickIdle->axes[i]<=-1){
                    if ((abs(ctrlData->joystickMsg->axes[i] - joystickIdle->axes[i])/2 ) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[3][1][1]) ) {
                            if (joystickData[4][1][0] * ctrlData->joystickMsg->axes[i] < 0){
                                //make sure its the opposite direction before calibrating
                                joystickData[5][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                                joystickData[5][1][1] = static_cast<float>(i);           // Axis index
                                downCalibrated = true;
                                buttonPressed = false; // Reset button state
                                return;
                            }
                    }
                }
            }
        }
        
        else if (!rightCalibrated && downCalibrated){
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Please put joystick in RIGHT and press 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->axes.size(); ++i) {
                if (joystickIdle ->axes[i]<=0){
                    if (abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i]) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[3][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[5][1][1])) {
                        joystickData[6][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[6][1][1] = static_cast<float>(i);           // Axis index
                        rightCalibrated = true;
                        buttonPressed = false; // Reset button state
                        return;
                    }
                }
                else if (joystickIdle->axes[i]>=0){
                    if ((abs(ctrlData->joystickMsg->axes[i] - joystickIdle->axes[i])/2 ) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[3][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[5][1][1])) {
                        joystickData[6][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[6][1][1] = static_cast<float>(i);           // Axis index
                        rightCalibrated = true;
                        buttonPressed = false; // Reset button state
                        return;
                    }
                }
            }
        }
        else if (!leftCalibrated && rightCalibrated){
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Please put joystick in LEFT and press 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->axes.size(); ++i) {
                if (joystickIdle->axes[i]<=0){
                    if (abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i]) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[3][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[5][1][1])) {
                            if (joystickData[6][1][0] * ctrlData->joystickMsg->axes[i] < 0){
                                //make sure its the opposite direction before calibrating
                                joystickData[7][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                                joystickData[7][1][1] = static_cast<float>(i);           // Axis index
                                leftCalibrated = true;
                                buttonPressed = false; // Reset button state
                                return;
                            }
                    }
                }
                else if (joystickIdle->axes[i]>=0){
                    if ((abs(ctrlData->joystickMsg->axes[i] - joystickIdle->axes[i])/2 ) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[3][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[5][1][1])) {
                            if (joystickData[6][1][0] * ctrlData->joystickMsg->axes[i] < 0){
                                //make sure its the opposite direction before calibrating
                                joystickData[7][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                                joystickData[7][1][1] = static_cast<float>(i);           // Axis index
                                leftCalibrated = true;
                                buttonPressed = false; // Reset button state
                                return;
                            }
                    }
                }
            }
        }
        
        else if (!rollRightCalibrated && leftCalibrated) {
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Please put joystick in ROLL RIGHT and press 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->axes.size(); ++i) {
                if (joystickIdle->axes[i] <= 0) {
                    if (abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i]) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[3][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[5][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[7][1][1])) {
                        joystickData[8][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[8][1][1] = static_cast<float>(i);           // Axis index
                        rollRightCalibrated = true;
                        buttonPressed = false; // Reset button state
                        return;
                    }
                } else if (joystickIdle->axes[i] >= 0) {
                    if ((abs(ctrlData->joystickMsg->axes[i] - joystickIdle->axes[i]) / 2) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[3][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[5][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[7][1][1])) {
                        joystickData[8][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[8][1][1] = static_cast<float>(i);           // Axis index
                        rollRightCalibrated = true;
                        buttonPressed = false; // Reset button state
                        return;
                    }
                }
            }
        }
        
        else if (!rollLeftCalibrated && rollRightCalibrated) {
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Please put joystick in ROLL LEFT and press 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->axes.size(); ++i) {
                if (joystickIdle->axes[i] <= 0) {
                    if (abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i]) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[3][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[5][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[7][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[8][1][1])) {
                        joystickData[9][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[9][1][1] = static_cast<float>(i);           // Axis index
                        rollLeftCalibrated = true;
                        buttonPressed = false; // Reset button state
                        return;
                    }
                } else if (joystickIdle->axes[i] >= 0) {
                    if ((abs(ctrlData->joystickMsg->axes[i] - joystickIdle->axes[i]) / 2) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[3][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[5][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[7][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[8][1][1])) {
                        joystickData[9][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[9][1][1] = static_cast<float>(i);           // Axis index
                        rollLeftCalibrated = true;
                        buttonPressed = false; // Reset button state
                        return;
                    }
                }
            }
        }
        
        else if (!pitchUpCalibrated && rollLeftCalibrated){
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PLEASE PRESS BUTTOM FOR PITCH UP and press 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->buttons.size(); ++i) {
                if (ctrlData->joystickMsg->buttons[i] == 1 && buttonPressed && static_cast<int>(i) != confirmationButton) {
                    joystickData[10][1][0] = static_cast<float>(ctrlData->joystickMsg->buttons[i]);  // Calibrated value
                    joystickData[10][1][1] = static_cast<float>(i);           // button index
                    pitchUpCalibrated = true;
                    buttonPressed = false; // Reset button state
                    return;
                }
            }
        }   
        else if (!pitchDownCalibrated && pitchUpCalibrated){
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Please put joystick in PITCH DOWN and press 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->buttons.size(); ++i) {
                if (ctrlData->joystickMsg->buttons[i] == 1 && buttonPressed  && static_cast<int>(i) != confirmationButton && static_cast<int>(i) != static_cast<int>(joystickData[10][1][1])) {
                    joystickData[11][1][0] = static_cast<float>(ctrlData->joystickMsg->buttons[i]);  // Calibrated value
                    joystickData[11][1][1] = static_cast<float>(i);           // button index
                    pitchDownCalibrated = true;
                    buttonPressed = false; // Reset button state
                    return;
                }
            }
        }
        
        else if (!idleCalibrated && pitchDownCalibrated) {
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Please put joystick in IDLE and press 'X'.");
            if (buttonPressed){
                buttonPressed = false; // Reset button state
                for (std::size_t i = 0; i < 12; ++i) {
                        if (i < 11) { // For axes (first 11 calibration states)
                            joystickData[i][2][0] = ctrlData->joystickMsg->axes[i]; // Record idle axis value
                            joystickData[i][2][1] = i;                              // Store the axis index
                        } else { // For buttons (last 2 calibration states: pitch up/down)
                            joystickData[i][2][0] = static_cast<float>(ctrlData->joystickMsg->buttons[i]); // Record idle button state
                            joystickData[i][2][1] = i;                                                    // Store the button index
                        }
                }
                idleCalibrated = true; // Mark idle calibration as complete
                calibrationDone = true;
                RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "CALIBRATION COMPLETE.");
                return;
            }
        }
    }
}

void JoystickState::MapJoystickToVelocityVelocites() {
    // Forward/Backward -> Action 0 and 1
    ctrlData->velocityDesired[0] = MapValue(
        ctrlData->joystickMsg->axes[static_cast<int>(joystickData[0][1][1])],
        joystickData[static_cast<int>(joystickData[0][1][1])][2][0],
        joystickData[0][1][0],
        ((ctrlData->maxVelocity[0] + ctrlData->minVelocity[0]) / 2),
        ctrlData->maxVelocity[0]) + MapValue(
        ctrlData->joystickMsg->axes[static_cast<int>(joystickData[1][1][1])],
        joystickData[1][1][0],
        joystickData[static_cast<int>(joystickData[1][1][1])][2][0],
        ctrlData->minVelocity[0],
        ((ctrlData->maxVelocity[0] + ctrlData->minVelocity[0]) / 2));

    // Left/Right -> Action 6 and 7
    ctrlData->velocityDesired[1] = MapValue(
        ctrlData->joystickMsg->axes[static_cast<int>(joystickData[6][1][1])],
        joystickData[static_cast<int>(joystickData[6][1][1])][2][0],
        joystickData[6][1][0],
        ((ctrlData->maxVelocity[1] + ctrlData->minVelocity[1]) / 2),
        ctrlData->maxVelocity[1]) + MapValue(
        ctrlData->joystickMsg->axes[static_cast<int>(joystickData[7][1][1])],
        joystickData[7][1][0],
        joystickData[static_cast<int>(joystickData[7][1][1])][2][0],
        ctrlData->minVelocity[1],
        ((ctrlData->maxVelocity[1] + ctrlData->minVelocity[1]) / 2));

    // Up/Down -> Action 4 and 5
    ctrlData->velocityDesired[2] = MapValue(
        ctrlData->joystickMsg->axes[static_cast<int>(joystickData[4][1][1])],
        joystickData[static_cast<int>(joystickData[4][1][1])][2][0],
        joystickData[4][1][0],
        ((ctrlData->maxVelocity[2] + ctrlData->minVelocity[2]) / 2),
        ctrlData->maxVelocity[2]) + MapValue(
        ctrlData->joystickMsg->axes[static_cast<int>(joystickData[5][1][1])],
        joystickData[5][1][0],
        joystickData[static_cast<int>(joystickData[5][1][1])][2][0],
        ctrlData->minVelocity[2],
        ((ctrlData->maxVelocity[2] + ctrlData->minVelocity[2]) / 2));

    // Roll -> Action 8 and 9
    ctrlData->velocityDesired[3] = MapValue(
        ctrlData->joystickMsg->axes[static_cast<int>(joystickData[8][1][1])], // Axis for R2
        joystickData[8][1][0], // R2 idle value (inMin)
        joystickData[8][2][0], // R2 max value (inMax)
        ctrlData->minVelocity[3], // Minimum roll velocity (outMin)
        ctrlData->maxVelocity[3]  // Maximum roll velocity (outMax)
        )
        +
        MapValue(
        ctrlData->joystickMsg->axes[static_cast<int>(joystickData[9][1][1])], // Axis for L2
        joystickData[9][1][0], // L2 idle value (inMin)
        joystickData[9][2][0], // L2 max value (inMax)
        ctrlData->minVelocity[3], // Minimum roll velocity (outMin)
        ctrlData->maxVelocity[3]  // Maximum roll velocity (outMax)
        );


    // // Pitch -> Action 10 and 11 (buttons, increments on press)
    if (ctrlData->joystickMsg->buttons[static_cast<int>(joystickData[10][1][1])] > 0.5) { // Button 10 pressed
        ctrlData->velocityDesired[4] += incrementSpeed; // Increment pitch velocity
        if (ctrlData->velocityDesired[4] > ctrlData->maxVelocity[4]) ctrlData->velocityDesired[4] = ctrlData->maxVelocity[4];
    } else if (ctrlData->joystickMsg->buttons[static_cast<int>(joystickData[11][1][1])] > 0.5) { // Button 11 pressed
        ctrlData->velocityDesired[4] -= incrementSpeed; // Decrement pitch velocity
        if (ctrlData->velocityDesired[4] < ctrlData->minVelocity[4]) ctrlData->velocityDesired[4] = ctrlData->minVelocity[4];
    }



    // Yaw -> Action 2 and 3
    ctrlData->velocityDesired[5] = MapValue(
        ctrlData->joystickMsg->axes[static_cast<int>(joystickData[2][1][1])],
        joystickData[static_cast<int>(joystickData[2][1][1])][2][0],
        joystickData[2][1][0],
        ((ctrlData->maxVelocity[5] + ctrlData->minVelocity[5]) / 2),
        ctrlData->maxVelocity[5]) + MapValue(
        ctrlData->joystickMsg->axes[static_cast<int>(joystickData[3][1][1])],
        joystickData[3][1][0],
        joystickData[static_cast<int>(joystickData[3][1][1])][2][0],
        ctrlData->minVelocity[5],
        ((ctrlData->maxVelocity[5] + ctrlData->minVelocity[5]) / 2));
}

double JoystickState::MapValue(double value, double inMin, double inMax, double outMin, double outMax) {
    // Ensure input range is ordered correctly
    if (inMin > inMax) {
        std::swap(inMin, inMax);
    }

    // Ensure output range is ordered correctly
    if (outMin > outMax) {
        std::swap(outMin, outMax);
    }

    // Check if value is out of the input range
    if (value < inMin || value > inMax) {
        // std::cerr << "Value out of range." << std::endl;
        return (outMin + outMax) / 2.0; // Return midpoint of output range
    }
    // Map the value
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
