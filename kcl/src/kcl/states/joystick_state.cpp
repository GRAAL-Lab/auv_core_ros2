#include "states/joystick_state.hpp"
#include <cmath>
#include <iostream>

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
    if (!calibrationDone) {
        for (int action = 0; action < 12; ++action) {
            std::cout << "Action " << action 
                      << " | Idle Value: " << joystickData[action][0][0]
                      << " | Calibrated Value: " << joystickData[action][1][0]
                      << " | Axis: " << joystickData[action][1][1] 
                      << std::endl;
        }
        //print joystick axes
        for (std::size_t i = 0; i < ctrlData->joystickMsg->axes.size(); ++i) {
            std::cout << "Axis " << i << ": " << ctrlData->joystickMsg->axes[i] << std::endl;
        }
        CalibrateJoystick();
    } else {
        std::cout << "Calibration Results:" << std::endl;

        // Print the calibrated values and their corresponding axis along with idle values
        for (int action = 0; action < 12; ++action) {
            std::cout << "Action " << action 
                      << " | Idle Value: " << joystickData[action][0][0]
                      << " | Calibrated Value: " << joystickData[action][1][0]
                      << " | Axis: " << joystickData[action][1][1] 
                      << std::endl;
        }

        // Indicate calibration completion
        std::cout << "Calibration done!" << std::endl;
    }
    return fsm::ok;
}


// OnExit: Cleanup
fsm::retval JoystickState::OnExit() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Exiting JOYSTICK state");
    return fsm::ok;
}

void JoystickState::CalibrateJoystick() {
    static bool buttonPressed = false; // Tracks if the confirmation button is currently pressed

    if (!ctrlData || !ctrlData->joystickMsg) {
        RCLCPP_ERROR(rclcpp::get_logger("JoystickState"), "Joystick message is null. Cannot calibrate.");
        return;
    }

    if (!xFound) {
        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PLEASE PRESS THE X BUTTON");
        for (std::size_t i = 0; i < ctrlData->joystickMsg->buttons.size(); ++i) {
            if (ctrlData->joystickMsg->buttons[i] == 1) {
                joystickIdle = ctrlData->joystickMsg;
                confirmationButton = i;
                xFound = true;
                RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "X BUTTON DETECTED AT INDEX %u.", confirmationButton);
                return;
            }
        }
    }
    else{

        // Check the button press state to handle debounce
        if (ctrlData->joystickMsg->buttons[confirmationButton] == 1) {
            if (!buttonPressed) {
                buttonPressed = true; // Button press detected
                std::cout << "Button pressed" << std::endl;
            }
        } else {
            buttonPressed = false; // Reset button state upon release
        }

        if (!forwardCalibrated) {
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PLEASE PUT JOYSTICK IN FORWARD AND PRESS 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->axes.size(); ++i) {
                if(joystickIdle->axes[i]>=0){
                    if (abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i]) > 0.4 && buttonPressed) {
                        joystickData[0][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[0][1][1] = static_cast<float>(i);           // Axis index
                        forwardCalibrated = true;
                        buttonPressed = false; // Reset button state
                        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "FORWARD CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[0][1][0]);
                        return;
                    }
                }
                else if (joystickIdle->axes[i]<=-1){
                if ((abs(ctrlData->joystickMsg->axes[i] - joystickIdle->axes[i])/2 ) > 0.4 && buttonPressed) {
                        joystickData[0][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[0][1][1] = static_cast<float>(i);           // Axis index
                        forwardCalibrated = true;
                        buttonPressed = false; // Reset button state
                        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "FORWARD CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[0][1][0]);
                        return;
                    }
                }
            }
        }
        else if (!backwardCalibrated && forwardCalibrated) {
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PLEASE PUT JOYSTICK IN BACKWARD AND PRESS 'X'.");
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
                            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "BACKWARD CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[1][1][0]);
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
                            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "BACKWARD CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[1][1][0]);
                            return;
                        }
                    }
                }
            }
        }  
        
        else if (!yawRightCalibrated && backwardCalibrated) {
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PLEASE PUT JOYSTICK IN YAW CLOCKWISE AND PRESS 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->axes.size(); ++i) {
                if (joystickIdle->axes[i] <=0){
                    if (abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i]) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1])) {
                        joystickData[2][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[2][1][1] = static_cast<float>(i);           // Axis index
                        yawRightCalibrated = true;
                        buttonPressed = false; // Reset button state
                        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "YAW CLOCKWISE CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[2][1][0]);
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
                        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "YAW CLOCKWISE CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[2][1][0]);
                        return;
                    }
                }
            }
        }  
        else if (!yawLefteCalibrated && yawRightCalibrated) {
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PLEASE PUT JOYSTICK IN YAW COUNTERCLOCKWISE AND PRESS 'X'.");
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
                                RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "YAW COUNTERCLOCKWISE CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[3][1][0]);
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
                                RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "YAW COUNTERCLOCKWISE CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[3][1][0]);
                                return;
                            }
                    }
                }
            }
            
        }
        
        else if (!upCalibrated && yawLefteCalibrated){
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PLEASE PUT JOYSTICK IN UP AND PRESS 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->axes.size(); ++i) {

                if (joystickIdle->axes[i]<=0){
                    if (abs(joystickIdle->axes[i] - ctrlData->joystickMsg->axes[i]) > 0.4 && buttonPressed &&
                        static_cast<int>(i) != static_cast<int>(joystickData[1][1][1]) &&
                        static_cast<int>(i) != static_cast<int>(joystickData[3][1][1])) {
                        joystickData[4][1][0] = ctrlData->joystickMsg->axes[i];  // Calibrated value
                        joystickData[4][1][1] = static_cast<float>(i);           // Axis index
                        upCalibrated = true;
                        buttonPressed = false; // Reset button state
                        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "UP CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[4][1][0]);
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
                        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "UP CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[4][1][0]);
                        return;
                    }
                }
            }
        }
        else if (!downCalibrated && upCalibrated){
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PLEASE PUT JOYSTICK IN DOWN AND PRESS 'X'.");
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
                            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "DOWN CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[5][1][0]);
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
                                RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "DOWN CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[5][1][0]);
                                return;
                            }
                    }
                }
            }
        }
        
        else if (!rightCalibrated && downCalibrated){
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PLEASE PUT JOYSTICK IN RIGHT AND PRESS 'X'.");
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
                        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "RIGHT CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[6][1][0]);
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
                        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "RIGHT CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[6][1][0]);
                        return;
                    }
                }
            }
        }
        else if (!leftCalibrated && rightCalibrated){
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PLEASE PUT JOYSTICK IN LEFT AND PRESS 'X'.");
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
                                RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "LEFT CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[7][1][0]);
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
                                RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "LEFT CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[7][1][0]);
                                return;
                            }
                    }
                }
            }
        }
        
        else if (!rollRightCalibrated && leftCalibrated) {
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PLEASE PUT JOYSTICK IN ROLL RIGHT AND PRESS 'X'.");
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
                        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "ROLL RIGHT CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[8][1][0]);
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
                        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "ROLL RIGHT CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[8][1][0]);
                        return;
                    }
                }
            }
        }
        
        else if (!rollLeftCalibrated && rollRightCalibrated) {
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PLEASE PUT JOYSTICK IN ROLL LEFT AND PRESS 'X'.");
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
                        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "ROLL LEFT CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[9][1][0]);
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
                        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "ROLL LEFT CALIBRATION COMPLETE: Axis %zu, Value %.2f", i, joystickData[9][1][0]);
                        return;
                    }
                }
            }
        }
        
        else if (!pitchUpCalibrated && rollLeftCalibrated){
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PLEASE PRESS BUTTOM FOR PITCH UP AND PRESS 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->buttons.size(); ++i) {
                if (ctrlData->joystickMsg->buttons[i] == 1 && buttonPressed && static_cast<int>(i) != confirmationButton) {
                    joystickData[10][1][0] = static_cast<float>(ctrlData->joystickMsg->buttons[i]);  // Calibrated value
                    joystickData[10][1][1] = static_cast<float>(i);           // button index
                    pitchUpCalibrated = true;
                    buttonPressed = false; // Reset button state
                    RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PITCH UP CALIBRATION COMPLETE: Button %zu, Value %.2f", i, joystickData[10][1][0]);
                    return;
                }
            }
        }   
        else if (!pitchDownCalibrated && pitchUpCalibrated){
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PLEASE PUT JOYSTICK IN PITCH DOWN AND PRESS 'X'.");
            for (std::size_t i = 0; i < ctrlData->joystickMsg->buttons.size(); ++i) {
                if (ctrlData->joystickMsg->buttons[i] == 1 && buttonPressed  && static_cast<int>(i) != confirmationButton && static_cast<int>(i) != static_cast<int>(joystickData[10][1][1])) {
                    joystickData[11][1][0] = static_cast<float>(ctrlData->joystickMsg->buttons[i]);  // Calibrated value
                    joystickData[11][1][1] = static_cast<float>(i);           // button index
                    pitchDownCalibrated = true;
                    buttonPressed = false; // Reset button state
                    RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PITCH DOWN CALIBRATION COMPLETE: Button %zu, Value %.2f", i, joystickData[11][1][0]);
                    return;
                }
            }
        }
        
        else if (!idleCalibrated && pitchDownCalibrated) {
            RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "PLEASE PUT JOYSTICK IN IDLE AND PRESS 'X'.");
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
