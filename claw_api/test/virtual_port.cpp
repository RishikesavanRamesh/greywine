#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>  // Include for read, write, and close
#include <stdint.h>  // Include for intptr_t

#define BUFFER_SIZE 128
#define BAUDRATE    B9600

void usage(char* cmd) {
    std::cerr << "usage: " << cmd << " slave|master [device, only in slave mode]" << std::endl;
    exit(1);
}

void* reader_thread(void* pointer) {
    intptr_t fd = reinterpret_cast<intptr_t>(pointer);  // Use intptr_t for pointer to int conversion
    char inputbyte;
    while (read(fd, &inputbyte, 1) == 1) {
        std::cout << inputbyte;
        std::cout.flush();
    }

    return nullptr;
}

int main(int argc, char** argv) {
    if (argc < 2) usage(argv[0]);

    intptr_t fd = 0;  // Use intptr_t here as well
    std::string mode = argv[1];

    if (mode == "slave") {
        if (argc < 3) usage(argv[1]);

        fd = open(argv[2], O_RDWR);
        if (fd == -1) {
            std::cerr << "error opening file" << std::endl;
            return -1;
        }

    } else if (mode == "master") {
        fd = open("/dev/ptmx", O_RDWR | O_NOCTTY);
        if (fd == -1) {
            std::cerr << "error opening file" << std::endl;
            return -1;
        }

        grantpt(fd);
        unlockpt(fd);

        char* pts_name = ptsname(fd);
        std::cerr << "ptsname: " << pts_name << std::endl;
    } else {
        usage(argv[1]);
    }

    /* serial port parameters */
    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio));
    struct termios oldtio;
    tcgetattr(fd, &oldtio);

    newtio = oldtio;
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = 0;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 0;
    tcflush(fd, TCIFLUSH);

    cfsetispeed(&newtio, BAUDRATE);
    cfsetospeed(&newtio, BAUDRATE);
    tcsetattr(fd, TCSANOW, &newtio);

    /* start reader thread */
    pthread_t thread;
    pthread_create(&thread, nullptr, reader_thread, reinterpret_cast<void*>(fd));  // Cast back to void*

    /* read from stdin and send it to the serial port */
    char c;
    while (true) {
        std::cin >> c;
        write(fd, &c, 1);
    }

    close(fd);
    return 0;
}
