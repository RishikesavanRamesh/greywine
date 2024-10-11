#ifndef CLAW_API_HPP
#define CLAW_API_HPP

#include <string>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <exception>

class RoboClaw {
public:
    void connect(const std::string& device) {
        std::cout << "[RoboClaw::connect] Attempting to connect to device: " << device << std::endl;

        // Open the device (e.g., /dev/ttyACM0)
        int device_port_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (device_port_ < 0) {
            std::cerr << "[RoboClaw::connect] Unable to open USB port: " 
                      << device << ", errno: (" << errno << ") "
                      << strerror(errno) << std::endl;
            throw std::runtime_error("Unable to open USB port");
        }
        
        std::cout << "[RoboClaw::connect] Successfully connected to device!" << std::endl;
    }

    // Other member functions can be implemented here...
};

#endif // CLAW_API_HPP
