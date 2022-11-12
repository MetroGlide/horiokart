#pragma once

#include "horiokart2021_sensors/serial_communicator.hpp"

#include <string>

#include <fstream>
#include <unistd.h>
#include <vector>
#include <time.h>

namespace horiokart2021_sensors
{
    struct MotorDriverResponse{
        std::vector<uint8_t> raw;
        SerialError error;
    };
    struct SetSpeedRequest{
        int16_t rightWheelSpeed, leftWheelSpeed;
    };

    class SerialMotorDriver
    {
    private:
        std::string device_name;

        SerialCommunicator serial;

        int sleep_usec;

        int SET_SPEED_BUF_SIZE = 7;
        int SET_SPEED_BUF_RET_SIZE = 4;
        std::vector<uint8_t> SetSpeedBufBase{0x96, 0x47};

        // SerialError checkError(std::vector<uint8_t> ret);

    public:
        SerialMotorDriver(){};
        SerialMotorDriver(std::string device_name, int read_sleep_usec);

        MotorDriverResponse setSpeedData(SetSpeedRequest data);
        std::vector<uint8_t> encode(SetSpeedRequest data);

        bool isAlive();
    };
}
