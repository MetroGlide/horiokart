#pragma once

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <iomanip>

#include <fstream>
#include <unistd.h>
#include <vector>

namespace horiokart2021_sensors
{

    class SerialCommunicator
    {
    private:
        int fd1;
        std::string device_name;
        
        void open_serial(std::string device_name);

    public:
        bool is_open_serial;

        SerialCommunicator(){};
        SerialCommunicator(std::string device_name);
        ~SerialCommunicator(){};


        int serial_write(std::vector<uint8_t>);
        std::vector<uint8_t> serial_read();
        std::vector<uint8_t> serial_readwrite(std::vector<uint8_t>);

        uint8_t calc_checksum(std::vector<uint8_t>);
    };
}