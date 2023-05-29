#pragma once

#include "horiokart_devices/base/serial_communicator.hpp"

#include <string>

#include <fstream>
#include <unistd.h>
#include <vector>
#include <time.h>

namespace horiokart_devices
{
    struct OdometryData
    {
        double x, y, th;
        double vx, vy, vth;
        std::vector<uint8_t> raw;

        SerialError error;
    };

    class WheelOdometry
    {
    private:
        std::string device_name_;

        SerialCommunicator serial_;

        const int sleep_usec_ = 1000; // usec

        int get_odom_ret_size_ = 19;
        std::vector<uint8_t> zero_buf_{0x24, 0x73};
        std::vector<uint8_t> odom_buf_{0x24, 0x75};

    public:
        explicit WheelOdometry() {};
        explicit WheelOdometry(std::string device_name);
        virtual ~WheelOdometry(){};

        OdometryData get_data();
        SerialError send_zero_reset(int retry = 1);
        OdometryData decode(std::vector<uint8_t> ret);

        bool is_alive();
    };
}
