#ifndef CLAW_API_HPP
#define CLAW_API_HPP

#include <iostream>
#include <libserial/SerialStream.h>
#include <cstdint>
#include <cstdarg>
#include <stdexcept>
#include <thread>
#include <chrono>

#define M1FORWARD 0
#define M1BACKWARD 1
#define M2FORWARD 4
#define M2BACKWARD 5
#define DRIVEM1 6
#define DRIVEM2 7
#define DRIVEFORWARD 8
#define DRIVEBACKWARD 9
#define TURNRIGHT 10
#define TURNLEFT 11
#define DRIVEFORWARDORBACKWARD 12
#define TURNLEFTORRIGHT 13

#define MAXRETRY 3
#define TIMEOUT 10 // Milliseconds

class RoboClaw {
public:
    RoboClaw();
    void openPort(const std::string& device_name);
    void closePort();
    bool isPortOpen();
    bool ForwardM1(uint8_t address, uint8_t speed);
    bool BackwardM1(uint8_t address, uint8_t speed);
    bool ForwardM2(uint8_t address, uint8_t speed);
    bool BackwardM2(uint8_t address, uint8_t speed);
    bool DriveM1(uint8_t address, uint8_t speed);
    bool DriveM2(uint8_t address, uint8_t speed);
    bool DriveForward(uint8_t address, uint8_t speed);
    bool DriveBackward(uint8_t address, uint8_t speed);
    bool TurnRightMixed(uint8_t address, uint8_t speed);
    bool TurnLeftMixed(uint8_t address, uint8_t speed);
    bool TurnLeftOrRight(uint8_t address, uint8_t speed);
    bool DriveForwardOrBackward(uint8_t address, uint8_t speed);
    bool Stop();
    ~RoboClaw();

private:
    std::string port_;
    LibSerial::SerialStream serialStream_;
    uint16_t crc;

    bool write_n(uint8_t cnt, ...);
    void crc_update(uint8_t data);
    void crc_clear();
    uint16_t crc_get();
    size_t write(uint8_t byte);
    uint8_t read(int timeout_ms);
};

#endif // CLAW_API_HPP
