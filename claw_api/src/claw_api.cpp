#include "claw_api/claw_api.hpp"

RoboClaw::RoboClaw() : crc(0) {}

void RoboClaw::openPort(const std::string& device_name) {
    port_ = device_name;
    serialStream_.Open(port_);
    if (!serialStream_.IsOpen()) {
        throw std::runtime_error("Unable to open USB port");
    }
    serialStream_.SetBaudRate(LibSerial::BaudRate::BAUD_38400);
    serialStream_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serialStream_.SetParity(LibSerial::Parity::PARITY_NONE);
    serialStream_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
}

void RoboClaw::closePort() {
    if (serialStream_.IsOpen()) {
        serialStream_.Close();
    }
}

bool RoboClaw::isPortOpen() {
    return serialStream_.IsOpen();
}

bool RoboClaw::ForwardM1(uint8_t address, uint8_t speed) {
    return write_n(3, address, M1FORWARD, speed);
}

bool RoboClaw::BackwardM1(uint8_t address, uint8_t speed) {
    return write_n(3, address, M1BACKWARD, speed);
}

bool RoboClaw::ForwardM2(uint8_t address, uint8_t speed) {
    return write_n(3, address, M2FORWARD, speed);
}

bool RoboClaw::BackwardM2(uint8_t address, uint8_t speed) {
    return write_n(3, address, M2BACKWARD, speed);
}

bool RoboClaw::DriveM1(uint8_t address, uint8_t speed) {
    return write_n(3, address, DRIVEM1, speed);
}

bool RoboClaw::DriveM2(uint8_t address, uint8_t speed) {
    return write_n(3, address, DRIVEM2, speed);
}

bool RoboClaw::DriveForward(uint8_t address, uint8_t speed) {
    return write_n(3, address, DRIVEFORWARD, speed);
}

bool RoboClaw::DriveBackward(uint8_t address, uint8_t speed) {
    return write_n(3, address, DRIVEBACKWARD, speed);
}

bool RoboClaw::TurnRightMixed(uint8_t address, uint8_t speed) {
    return write_n(3, address, TURNRIGHT, speed);
}

bool RoboClaw::TurnLeftMixed(uint8_t address, uint8_t speed) {
    return write_n(3, address, TURNLEFT, speed);
}

bool RoboClaw::TurnLeftOrRight(uint8_t address, uint8_t speed) {
    if (speed > 127) speed = 127; 
    return write_n(3, address, TURNLEFTORRIGHT, speed);
}

bool RoboClaw::DriveForwardOrBackward(uint8_t address, uint8_t speed) {
    if (speed > 127) speed = 127; 
    return write_n(3, address, DRIVEFORWARDORBACKWARD, speed);
}

bool RoboClaw::Stop() {
    return write_n(3, 0x80, 0x13, 64);
}

RoboClaw::~RoboClaw() {
    closePort();
}

bool RoboClaw::write_n(uint8_t cnt, ...) {
    uint8_t trys = MAXRETRY;
    do {
        crc_clear();
        va_list marker;
        va_start(marker, cnt);

        for (uint8_t index = 0; index < cnt; index++) {
            uint8_t data = static_cast<uint8_t>(va_arg(marker, int));
            crc_update(data);
            write(data);
        }
        va_end(marker);

        uint16_t crcValue = crc_get();
        write(crcValue >> 8);
        write(crcValue);

        if (read(TIMEOUT) == 0xFF) {
            return true;
        }
    } while (trys--);
    return false;
}

void RoboClaw::crc_update(uint8_t data) {
    crc ^= (static_cast<uint16_t>(data) << 8);
    for (int i = 0; i < 8; i++) {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }
}

void RoboClaw::crc_clear() {
    crc = 0;
}

uint16_t RoboClaw::crc_get() {
    return crc;
}

size_t RoboClaw::write(uint8_t byte) {
    serialStream_.write(reinterpret_cast<const char*>(&byte), 1);
    return 1; // Indicate that 1 byte was written
}

uint8_t RoboClaw::read(int timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();
    uint8_t byte;

    while (true) {
        if (serialStream_.read(reinterpret_cast<char*>(&byte), 1)) {
            return byte; // Return the byte if read successfully
        }

        // Check for timeout
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() >= timeout_ms) {
            throw std::runtime_error("Read timeout");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Small delay to prevent busy waiting
    }
}
