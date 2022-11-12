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
#include <algorithm>
#include <time.h>

namespace horiokart2021_sensors
{
    enum class SerialError
    {
        NoError,
        WriteError,
        ReceiveSizeError,
        InvalidHeader,
        ChecksumError,
        OtherError
    };

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


        int serial_write(std::vector<uint8_t> buf, bool input_flush=true, bool output_flush=true);
        std::vector<uint8_t> serial_read(int sleep_usec=5000);
        std::vector<uint8_t> serial_readwrite(std::vector<uint8_t> buf, int sleep_usec=5000, bool input_flush=true, bool output_flush=true);

        uint8_t calc_checksum(std::vector<uint8_t>);

        SerialError checkError(std::vector<uint8_t> ret, std::vector<uint8_t> header, int size);


        template <typename T>
        std::vector<uint8_t> encode(T data, bool big = true)
        {
            size_t s = sizeof(T);
            std::vector<uint8_t> ret(s);

            for (int i = 0; i < s;i++)
            {
                ret[i] = static_cast<uint8_t>(data >> (8 * (s-i-1)));
            }

            return ret;
        }

        template <typename T>
        T decode(std::vector<uint8_t> buf, bool big = true)
        {
            if (buf.size() < 2)
            {
                // Exception
            }
            reverse(buf.begin(), buf.end());

            T ret = static_cast<T>(buf[0]);
            for (int i = 1; i < buf.size(); i++)
            {
                ret |= static_cast<T>(buf[i]) << 8 * i;
            }
            return ret;
        }
    };
}