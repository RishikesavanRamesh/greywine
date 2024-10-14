#ifndef ROBOCLAW_COMMS_HPP
#define ROBOCLAW_COMMS_HPP

#include <iostream>
#include <chrono>
#include <thread>
#include <stdexcept>
#include "claw_api/claw_api.hpp"

class RoboClawComms {
public:
    RoboClawComms() 
        : address_(0x80) {}

    void connect(const std::string& device_name) {
        roboclaw_.openPort(device_name);
        stop();  // It's good to ensure motors are stopped upon connection
        std::cout << "Connected to RoboClaw." << std::endl;
    }

    void disconnect() {
        roboclaw_.closePort();
        std::cout << "Disconnected from RoboClaw." << std::endl;
    }

    bool isConnected() {
        return roboclaw_.isPortOpen();
    }

    void sendCommand(uint8_t leftCommand, uint8_t rightCommand) {
        if (!isConnected()) {
            throw std::runtime_error("RoboClaw not connected.");
        }

        // Adjust commands based on the ranges specified
        if (leftCommand == rightCommand) {
            if (leftCommand == 64) {
                roboclaw_.Stop();
            } else if (leftCommand < 64) {
                roboclaw_.DriveForwardOrBackward(address_, leftCommand); // Assuming < 64 means backward
            } else {
                roboclaw_.DriveForwardOrBackward(address_, leftCommand); // Assuming > 64 means forward
            }
        } else {
            // Drive both motors
            roboclaw_.DriveM1(address_, leftCommand);  // Note: leftCommand is passed to M1
            roboclaw_.DriveM2(address_, rightCommand); // Note: rightCommand is passed to M2
        }
    }

    void turnRight(uint8_t command) {
        stop();
        roboclaw_.TurnRightMixed(address_, command);
    }

    void turnLeft(uint8_t command) {
        stop();
        roboclaw_.TurnLeftMixed(address_, command);
    }

    void driveForward(uint8_t command) {
        stop();
        roboclaw_.DriveForward(address_, command);
    }

    void driveBackward(uint8_t command) {
        stop();
        roboclaw_.DriveBackward(address_, command); // This should call the correct method for backward
    }

    void stop() {
        if (!isConnected()) {
            throw std::runtime_error("RoboClaw not connected.");
        }
        roboclaw_.Stop();
    }

    void read_encoder_values(int& left_encoder, int& right_encoder) {
        left_encoder = 10;  // Replace with actual encoder reading logic
        right_encoder = 10; // Replace with actual encoder reading logic
    }

private:
    RoboClaw roboclaw_;
    const uint8_t address_;  // Constant address for RoboClaw
};

#endif // ROBOCLAW_COMMS_HPP
