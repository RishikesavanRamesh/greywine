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
#define CONFIG 98

#define TURNLEFTORRIGHT 12
#define DRIVEFORWARDORBACKWARD 12


#define MAXRETRY 3
#define TIMEOUT 1000 // Milliseconds

class RoboClaw {
public:
    RoboClaw(const std::string& device_name) : port_(device_name), crc(0) {}

    void openPort() {
        serialStream_.Open(port_);
        if (!serialStream_.IsOpen()) {
            throw std::runtime_error("Unable to open USB port");
        }
        serialStream_.SetBaudRate(LibSerial::BaudRate::BAUD_38400);
        serialStream_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serialStream_.SetParity(LibSerial::Parity::PARITY_NONE);
        serialStream_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    }

    void closePort() {
        if (serialStream_.IsOpen()) {
            serialStream_.Close();
        }
    }

    bool ForwardM1(uint8_t address, uint8_t speed) {
        return write_n(3, address, M1FORWARD, speed);
    }

    bool BackwardM1(uint8_t address, uint8_t speed) {
        return write_n(3, address, M1BACKWARD, speed);
    }

    bool ForwardM2(uint8_t address, uint8_t speed) {
        return write_n(3, address, M2FORWARD, speed);
    }

    bool BackwardM2(uint8_t address, uint8_t speed) {
        return write_n(3, address, M2BACKWARD, speed);
    }

    bool DriveM1(uint8_t address, uint8_t speed) {
        return write_n(3, address, DRIVEM1, speed);
    }

    bool DriveM2(uint8_t address, uint8_t speed) {
        return write_n(3, address, DRIVEM2, speed);
    }

    
    bool DriveForward(uint8_t address, uint8_t speed) {
        return write_n(3, address, DRIVEFORWARD, speed);
    }

    bool DriveBackward(uint8_t address, uint8_t speed) {
        return write_n(3, address, DRIVEFORWARD, speed);
    }

    bool TurnRightMixed(uint8_t address, uint8_t speed) {
        return write_n(3, address, TURNRIGHT, speed);
    }

    bool TurnLeftMixed(uint8_t address, uint8_t speed) {
        return write_n(3, address, TURNLEFT, speed);
    }

    bool TurnLeftOrRight(uint8_t address, uint8_t speed) {
        // Ensure speed is between 0 and 127
        if (speed > 127) speed = 127; 
        return write_n(3, address, TURNLEFTORRIGHT, speed);
    }

    bool DriveForwardOrBackward(uint8_t address, uint8_t speed) {
        // Ensure speed is between 0 and 127
        if (speed > 127) speed = 127; 
        return write_n(3, address, DRIVEFORWARDORBACKWARD, speed);
    }



    bool Stop(){
        return write_n(3, 0x80, 0x13, 64);
    }

    ~RoboClaw() {
        closePort();
    }

private:
    std::string port_;
    LibSerial::SerialStream serialStream_;
    uint16_t crc;

    bool write_n(uint8_t cnt, ...) {
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

    void crc_update(uint8_t data) {
        crc ^= (static_cast<uint16_t>(data) << 8);
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    void crc_clear() {
        crc = 0;
    }

    uint16_t crc_get() {
        return crc;
    }

    size_t write(uint8_t byte) {
        serialStream_.write(reinterpret_cast<const char*>(&byte), 1);
        return 1; // Indicate that 1 byte was written
    }

    uint8_t read(int timeout_ms) {
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
};
int main() {
    RoboClaw roboClaw("/dev/ttyACM0");

    try {
        roboClaw.openPort();
        std::cout << "Port opened successfully!" << std::endl;


        // Activate mixed mode
        if (roboClaw.DriveForward(0x80, 0)) {
            std::cout << "Mixed mode setdrive successfully!" << std::endl;
        } else {
            std::cerr << "Failed to set mixed mode!" << std::endl;
        }

        // Activate mixed mode
        if (roboClaw.TurnRightMixed(0x80, 0)) {
            std::cout << "Mixed mode setturn successfully!" << std::endl;
        } else {
            std::cerr << "Failed to setturn mixed mode!" << std::endl;
        }


        // Turn left
        if (roboClaw.TurnLeftMixed(0x80, 30)) {
            std::cout << "turning left mixed..." << std::endl;
        } else {
            std::cerr << "Failed to send drive command!" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait for 2 seconds

        // Turn left
        if (roboClaw.TurnRightMixed(0x80, 30)) {
            std::cout << "turning right Mixed ..." << std::endl;
        } else {
            std::cerr << "Failed to send drive command!" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait for 2 seconds
        

        // Activate mixed mode
        if (roboClaw.TurnRightMixed(0x80, 0)) {
            std::cout << "Mixed mode setturn successfully!" << std::endl;
        } else {
            std::cerr << "Failed to setturn mixed mode!" << std::endl;
        }


        if (roboClaw.DriveForward(0x80, 30)) {
            std::cout << "Driving M1 ..." << std::endl;
        } else {
            std::cerr << "Failed to send drive command!" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait for 2 seconds


        // Activate mixed mode
        if (roboClaw.TurnRightMixed(0x80, 0)) {
            std::cout << "Mixed mode setturn successfully!" << std::endl;
        } else {
            std::cerr << "Failed to setturn mixed mode!" << std::endl;
        }


        // Turn left
        if (roboClaw.DriveBackward(0x80, 30)) {
            std::cout << "Driving M1 ..." << std::endl;
        } else {
            std::cerr << "Failed to send drive command!" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait for 2 seconds

        // Activate mixed mode
        if (roboClaw.DriveForward(0x80, 0)) {
            std::cout << "Mixed mode setdrive successfully!" << std::endl;
        } else {
            std::cerr << "Failed to set mixed mode!" << std::endl;
        }

        // Activate mixed mode
        if (roboClaw.TurnRightMixed(0x80, 0)) {
            std::cout << "Mixed mode setturn successfully!" << std::endl;
        } else {
            std::cerr << "Failed to setturn mixed mode!" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1; // Failure
    }

    return 0; // Success
}
