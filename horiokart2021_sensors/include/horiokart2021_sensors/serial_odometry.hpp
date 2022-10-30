#pragma once

#include "horiokart2021_sensors/serial_communicator.hpp"

#include <string>

#include <fstream>
#include <unistd.h>
#include <vector>
#include <time.h>

namespace horiokart2021_sensors
{
    enum class OdometryError
    {
        NoError,
        WriteError,
        ReceiveSizeError,
        InvalidHeader,
        ChecksumError,
        OtherError
    };

    struct OdometryData{
        double x, y, th;
        double vx, vy, vth;
        std::vector<uint8_t> raw;

        OdometryError error;
    };

    class SerialOdometry
    {
    private:
        std::string device_name;

        SerialCommunicator serial;

        int sleep_usec;

        // const int GET_ODOM_RET_SIZE = 19;
        // const std::vector<uint8_t> ZeroBuf{0x24, 0x73};
        // const std::vector<uint8_t> OdomBuf{0x24, 0x75};
        int GET_ODOM_RET_SIZE = 19;
        std::vector<uint8_t> ZeroBuf{0x24, 0x73};
        std::vector<uint8_t> OdomBuf{0x24, 0x75};

        OdometryError checkError(std::vector<uint8_t> ret);
        OdometryData parse(std::vector<uint8_t> ret);

    public:
        SerialOdometry(){};
        SerialOdometry(std::string device_name, int read_sleep_usec);

        OdometryData getData();
        OdometryError sendZeroReset(int retry=1);

        bool isAlive();
    };
}