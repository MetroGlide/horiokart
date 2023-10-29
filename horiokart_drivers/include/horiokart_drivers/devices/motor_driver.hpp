#pragma once

// #include "horiokart_drivers/base/serial_communicator.hpp"
#include "horiokart_drivers/base/serial_communicator2.hpp"

#include <string>

#include <fstream>
#include <unistd.h>
#include <vector>
#include <time.h>

namespace horiokart_drivers
{
    struct MotorDriverResponse{
        std::vector<uint8_t> raw;
        SerialError error;
    };
    struct SpeedParameter{
        int16_t right_wheel_speed, left_wheel_speed; // mm/s
    };

    class MotorDriver
    {
    private:
        std::string device_name_;

        // SerialCommunicator serial_;
        SerialCommunicator2 serial_;

        int sleep_usec_ = 40000; // usec

        int send_speed_command_size_ = 7;
        int receive_speed_command_size_ = 4;
        std::vector<uint8_t> send_speed_command_header_{0x96, 0x47};

    public:
        MotorDriver(){};
        MotorDriver(std::string device_name);

        MotorDriverResponse send_speed_command(SpeedParameter param);
        std::vector<uint8_t> encode(SpeedParameter param);

        bool is_alive();
    };
}
