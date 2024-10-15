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


#define READ_ENCODER_SPEED_M1 18
#define READ_ENCODER_SPEED_M2 19 // New command for M2 encoder speed
#define READ_ENC_M1 20 // New command for M1 encoder reading
#define READ_ENC_M2 21 // New command for M2 encoder reading

#define MAXRETRY 3
#define TIMEOUT 10 // Milliseconds

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


    bool ReadEncoderSpeedM1(uint8_t address, int32_t &speed) {
        return readEncoderSpeed(address, speed, READ_ENCODER_SPEED_M1);
    }

    bool ReadEncoderSpeedM2(uint8_t address, int32_t &speed) {
        return readEncoderSpeed(address, speed, READ_ENCODER_SPEED_M2);
    }

    bool ReadEncM1(uint8_t address, int32_t &encoderValue) {
        return readEncoderValue(address, encoderValue, READ_ENC_M1);
    }

    bool ReadEncM2(uint8_t address, int32_t &encoderValue) {
        return readEncoderValue(address, encoderValue, READ_ENC_M2);
    }

    void Stop(){
        TurnRightMixed(0x80,0);
        // ForwardM1(0x80,0);   
        // ForwardM2(0x80,0);    
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

   
    bool readEncoderSpeed(uint8_t address, int32_t &speed, uint8_t command) {
        crc_clear();
        std::cout << "Sending command to read encoder speed..." << std::endl;

        write(address);
        write(command);
        
        uint8_t response[6];
        for (int i = 0; i < 6; i++) {
            response[i] = read(TIMEOUT);
        }

        uint16_t crcValue = crc_get();
        uint16_t receivedCrc = (static_cast<uint16_t>(response[4]) << 8) | response[5];

        // Always print the response, regardless of CRC match
        speed = (static_cast<int32_t>(response[0]) << 24) |
                (static_cast<int32_t>(response[1]) << 16) |
                (static_cast<int32_t>(response[2]) << 8) |
                (static_cast<int32_t>(response[3]));

        // Print whether the CRC matches or not
        if (crcValue == receivedCrc) {
            std::cout << "CRC matches while reading encoder speed." << std::endl;
        } else {
            std::cerr << "Error: CRC mismatch while reading encoder speed." << std::endl;
        }

        std::cout << "Encoder Speed: " << speed << std::endl;

        return true; // Always return true to indicate that we processed the response
    }

    bool readEncoderValue(uint8_t address, int32_t &encoderValue, uint8_t command) {
        crc_clear();
        std::cout << "Sending command to read encoder value..." << std::endl;

        write(address);
        write(command);
        
        uint8_t response[6];
        for (int i = 0; i < 6; i++) {
            response[i] = read(TIMEOUT);
        }

        uint16_t crcValue = crc_get();
        uint16_t receivedCrc = (static_cast<uint16_t>(response[4]) << 8) | response[5];

        // Always print the response, regardless of CRC match
        encoderValue = (static_cast<int32_t>(response[0]) << 24) |
                       (static_cast<int32_t>(response[1]) << 16) |
                       (static_cast<int32_t>(response[2]) << 8) |
                       (static_cast<int32_t>(response[3]));

        // Print whether the CRC matches or not
        if (crcValue == receivedCrc) {
            std::cout << "CRC matches while reading encoder value." << std::endl;
        } else {
            std::cerr << "Error: CRC mismatch while reading encoder value." << std::endl;
        }

        std::cout << "Encoder Value: " << encoderValue << std::endl;

        return true; // Always return true to indicate that we processed the response
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
    try {
        RoboClaw roboClaw("/dev/ttyACM0"); // Change this to your actual device name
        roboClaw.openPort();

        const uint8_t address = 0x80; // Replace with your actual address
        const uint8_t speed = 20; // Adjust the speed as necessary

        for (int i = 0; i < 1; ++i) {
            roboClaw.TurnRightMixed(address, speed);
            std::this_thread::sleep_for(std::chrono::seconds(2));

            roboClaw.TurnLeftMixed(address, speed);
            std::this_thread::sleep_for(std::chrono::seconds(2));

            int32_t encoderSpeed;
            if (roboClaw.ReadEncoderSpeedM1(address, encoderSpeed)) {
                std::cout << "M1 Encoder Speed: " << encoderSpeed << " pulses per second." << std::endl;
            } else {
                std::cerr << "Failed to read M1 encoder speed." << std::endl;
            }
            if (roboClaw.ReadEncoderSpeedM2(address, encoderSpeed)) {
                std::cout << "M2 Encoder Speed: " << encoderSpeed << " pulses per second." << std::endl;
            } else {
                std::cerr << "Failed to read M1 encoder speed." << std::endl;
            }
        }

        roboClaw.Stop();
        roboClaw.closePort();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}
