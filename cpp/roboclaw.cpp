#include <iostream>
#include <SerialStream.h>
#include <cstring>
#include <exception>
#include <thread>
#include <chrono>
#include <cstdarg>

class TRoboClawException : public std::exception {
public:
    explicit TRoboClawException(const char* message) : msg_(message) {}
    virtual const char* what() const noexcept override { return msg_; }
private:
    const char* msg_;
};

class RoboClaw {
public:
    RoboClaw(const std::string& device_name) : port_(device_name) {}

    void openPort() {
        std::cout << "[RoboClaw::openPort] about to open port: " << port_ << std::endl;
        serialStream_.Open(port_);
        if (!serialStream_.IsOpen()) {
            throw TRoboClawException("Unable to open USB port");
        }
        serialStream_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
        serialStream_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serialStream_.SetParity(LibSerial::Parity::PARITY_NONE);
        serialStream_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    }

    void updateCrc(uint16_t &crc, uint8_t data) {
        crc ^= (static_cast<uint16_t>(data) << 8);
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    void writeByte(uint8_t byte) {
        serialStream_.Write(&byte, 1);
    }

    void writeN(bool sendCRC, uint8_t cnt, ...) {
        uint16_t crc = 0;
        va_list marker;
        va_start(marker, cnt);

        for (uint8_t i = 0; i < cnt; i++) {
            uint8_t byte = va_arg(marker, int);
            writeByte(byte);
            updateCrc(crc, byte);
        }

        va_end(marker);

        if (sendCRC) {
            writeByte(crc >> 8);
            writeByte(crc);
        }
    }

    void sendValues() {
        const uint8_t address = 0x80;
        const uint8_t values[] = {1, 0}; // Example values

        while (true) {
            for (uint8_t value : values) {
                writeN(true, 4, address, 13, value); // Send [Address, 13, Value, CRC(2 bytes)]
                std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait for 3 seconds
            }
        }
    }

    uint8_t readByteWithTimeout() {
        // Implementation for reading a byte with timeout (optional)
        return 0; // Placeholder for actual implementation
    }

    ~RoboClaw() {
        if (serialStream_.IsOpen()) {
            serialStream_.Close();
        }
    }

private:
    std::string port_;
    LibSerial::SerialStream serialStream_;
};

int main() {
    RoboClaw roboClaw("/dev/ttyACM0");

    try {
        roboClaw.openPort();
        std::cout << "Port opened successfully!" << std::endl;
        roboClaw.sendValues(); // Start sending values
    } catch (const TRoboClawException& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1; // Failure
    }

    return 0; // Success
}
